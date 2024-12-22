#include "file_cache.h"
#include "main.h"
#include "storage.h"
#include "things.h"

#include <errno.h>
#include <string.h>
#include <sys/queue.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_random.h"

#include <com/amazonaws/kinesis/video/common/Include.h>
#include <psa/crypto_sizes.h>

#define FILE_CACHE_PREFIX STORAGE_MOUNTPOINT "/cache"

static SemaphoreHandle_t subscriber_mutex = NULL;
struct subscriber {
  file_cache_refresh_cb callback;
  void *arg;
  TAILQ_ENTRY(subscriber) entries;
};
TAILQ_HEAD(subscriber_list, subscriber);
static struct subscriber_list subscribers = TAILQ_HEAD_INITIALIZER(subscribers);

typedef struct {
  const char *name;
  const char *url;
  const char *hash;
  const char *hash_filename;
  size_t size;
} file_cache_manifest_entry;

typedef struct {
  file_cache_manifest_entry *entries;
  size_t entry_count;
} file_cache_manifest;

static SemaphoreHandle_t manifest_url_mutex = NULL;
static char *manifest_url = NULL;

static SemaphoreHandle_t manifest_cache_mutex = NULL;
static uint8_t manifest_hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {};
static file_cache_manifest *manifest_cache = NULL;
static time_t manifest_cache_last_update = 0;
static esp_err_t manifest_cache_last_update_err = ESP_OK;

static StaticTask_t file_cache_task_buffer;
static EXT_RAM_BSS_ATTR StackType_t file_cache_task_stack[6144];
static TaskHandle_t task_handle = NULL;
static size_t telemetry_index = 0;

static void telemetry_generator(void) {
  xSemaphoreTake(manifest_cache_mutex, portMAX_DELAY);

  things_send_telemetry_int("file_cache_entry_count",
                            manifest_cache != NULL ? manifest_cache->entry_count
                                                   : 0);
  things_send_telemetry_int("file_cache_last_update",
                            manifest_cache_last_update);
  things_send_telemetry_int("file_cache_last_error",
                            manifest_cache_last_update_err);

  char hash_str[PSA_HASH_MAX_SIZE * 2 + 1];
  for (size_t i = 0; i < PSA_HASH_LENGTH(PSA_ALG_SHA_256); i++) {
    snprintf(hash_str + i * 2, 3, "%02x", manifest_hash[i]);
  }
  things_send_telemetry_string("file_cache_hash", hash_str);

  xSemaphoreGive(manifest_cache_mutex);
}

static int manifest_entry_compare_name(const void *a, const void *b) {
  return strcmp(((file_cache_manifest_entry *)a)->name,
                ((file_cache_manifest_entry *)b)->name);
}

static int compare_hash(const void *a, const void *b) {
  return strcmp(*(const char **)a, *(const char **)b);
}

static void free_file_cache_manifest(file_cache_manifest *manifest) {
  if (manifest == NULL) {
    return;
  }

  for (size_t i = 0; i < manifest->entry_count; i++) {
    free((char *)manifest->entries[i].name);
    free((char *)manifest->entries[i].url);
    free((char *)manifest->entries[i].hash);
    free((char *)manifest->entries[i].hash_filename);
  }

  free(manifest->entries);
  free(manifest);
}

static void url_callback(const char *key, const things_attribute_t *attr) {
  xSemaphoreTake(manifest_url_mutex, portMAX_DELAY);
  if (manifest_url == NULL && attr->type != THINGS_ATTRIBUTE_TYPE_STRING) {
    goto cleanup;
  }

  if (manifest_url != NULL && attr->type == THINGS_ATTRIBUTE_TYPE_STRING &&
      strcmp(manifest_url, attr->value.string) == 0) {
    goto cleanup;
  }

  if (manifest_url != NULL) {
    free(manifest_url);
    manifest_url = NULL;
  }

  if (attr->type == THINGS_ATTRIBUTE_TYPE_STRING) {
    manifest_url = strdup(attr->value.string);
  }

  xTaskNotifyGive(task_handle);

cleanup:
  xSemaphoreGive(manifest_url_mutex);
}

struct http_data {
  int fd;
  char **contents;
  size_t len;
  bool failed;
};

static esp_err_t event_handler(esp_http_client_event_t *evt) {
  struct http_data *data = evt->user_data;
  if (evt->event_id == HTTP_EVENT_ON_DATA) {
    if (data->contents != NULL && *data->contents == NULL) {
      int status = esp_http_client_get_status_code(evt->client);
      if (status != 200) {
        ESP_LOGE(RADIO_TAG, "Received HTTP status code while fetching file: %d",
                 status);
        esp_http_client_close(evt->client);
        data->failed = true;
        return ESP_FAIL;
      }

      int64_t length = esp_http_client_get_content_length(evt->client);
      if (length < 0) {
        ESP_LOGE(RADIO_TAG,
                 "Could not get content length of manifest: %" PRId64, length);
        esp_http_client_close(evt->client);
        data->failed = true;
        return ESP_FAIL;
      }

      *data->contents = calloc(1, length + 1);
      if (!*data->contents) {
        ESP_LOGE(RADIO_TAG, "Could not allocate buffer for manifest");
        esp_http_client_close(evt->client);
        data->failed = true;
        return ESP_ERR_NO_MEM;
      }
      data->len = 0;
    }

    const void *http_data = evt->data;
    size_t http_len = evt->data_len;
    if (data->contents != NULL && *data->contents != NULL) {
      memcpy(*data->contents + data->len, http_data, http_len);
      data->len += http_len;
    }
    while (data->fd >= 0 && http_len > 0) {
      ssize_t ret = write(data->fd, http_data, http_len);
      if (ret < 0) {
        ESP_LOGE(RADIO_TAG, "Error writing to file: %d (%s)", errno,
                 strerror(errno));
        esp_http_client_close(evt->client);
        data->failed = true;
        return ESP_FAIL;
      }
      http_data = (const char *)http_data + ret;
      http_len -= ret;
    }
  }

  return ESP_OK;
}

static esp_err_t fetch_file(const char *url, const char *path, char **contents,
                            size_t *length, const char *base_url) {
  esp_err_t ret = ESP_OK;
  esp_http_client_handle_t http_client = NULL;
  struct http_data data = {
      .fd = -1,
      .contents = contents,
  };

  if (path != NULL) {
    data.fd = open(path, O_CREAT | O_TRUNC | O_WRONLY, 0666);
    if (data.fd == -1) {
      ESP_LOGE(RADIO_TAG, "Error creating file (%s): %d (%s)", path, errno,
               strerror(errno));
      ret = ESP_FAIL;
      goto cleanup;
    }
  }

  const char *initial_url = base_url != NULL ? base_url : url;

  esp_http_client_config_t cfg = {
      .url = initial_url,
      .method = HTTP_METHOD_GET,
      .transport_type = strcasecmp("https://", initial_url) == 0
                            ? HTTP_TRANSPORT_OVER_SSL
                            : HTTP_TRANSPORT_OVER_TCP,
      .crt_bundle_attach = esp_crt_bundle_attach,
      .user_data = &data,
      .event_handler = event_handler,
      .disable_auto_redirect = true,
      .buffer_size = 4096,
  };
  http_client = esp_http_client_init(&cfg);
  if (!http_client) {
    ESP_LOGE(RADIO_TAG, "Failed to initialize HTTP client");
    ret = ESP_FAIL;
    goto cleanup;
  }

  if (base_url != NULL) {
    ESP_GOTO_ON_ERROR(esp_http_client_set_url(http_client, url), cleanup,
                      RADIO_TAG, "Failed to set URL (%s): %d", url, err_rc_);
  }

  ESP_GOTO_ON_ERROR(esp_http_client_perform(http_client), cleanup, RADIO_TAG,
                    "Failed to fetch file (%s): %d", url, err_rc_);
  if (data.failed) {
    ret = ESP_FAIL;
    goto cleanup;
  }
  int http_status = esp_http_client_get_status_code(http_client);
  ESP_GOTO_ON_FALSE(http_status < 300, ESP_FAIL, cleanup, RADIO_TAG,
                    "Cache request returned HTTP status (%s): %d", url,
                    http_status);

cleanup:
  if (http_client != NULL) {
    esp_http_client_cleanup(http_client);
  }

  if (data.fd != -1) {
    fsync(data.fd);
    close(data.fd);
  }

  if (length != NULL) {
    *length = data.len;
  }

  return ret;
}

// Given a pointer to a token in the value position, return how many tokens need
// to be skipped to skip this value (including the one passed in - i.e. the
// return should never be 0)
static int jsmn_value_len(jsmntok_t *tokens) {
  int len = 1;
  if (tokens->type == JSMN_OBJECT) {
    int objlen = tokens->size;
    for (int i = 0; i < objlen; i++) {
      // Skip the key
      len++;
      // Skip the value
      len += jsmn_value_len(tokens + len);
    }
  } else {
    // Must be an array (or primitive, in which case size is 0 and this is a
    // no-op)
    int arrlen = tokens->size;
    for (int i = 0; i < arrlen; i++) {
      len += jsmn_value_len(tokens + len);
    }
  }

  return len;
}

/**
 * Our manifest has the following format:
 *
 * {
 *   "files": [
 *     {"name": "logical-file-name.opus", "url":
 * "https://example.com/file.opus", "hash": "abcd1234", "size": 12345},
 *     [...]
 *   ]
 * }
 *
 * Hash can be any string, but is expected to be unique for a given file. We may
 * use "name" for file type detection.
 */
static file_cache_manifest *parse_manifest(const char *manifest_contents,
                                           size_t manifest_len) {
  bool success = false;
  file_cache_manifest *manifest = NULL;
  jsmntok_t *tokens = NULL;

  jsmn_parser parser;
  jsmn_init(&parser);

  int tokcount = jsmn_parse(&parser, manifest_contents, manifest_len, NULL, 0);
  if (tokcount <= 0) {
    ESP_LOGE(RADIO_TAG, "Error while parsing manifest: %d", tokcount);
    goto cleanup;
  }

  tokens = calloc(tokcount, sizeof(jsmntok_t));
  if (tokens == NULL) {
    ESP_LOGE(RADIO_TAG, "Error while allocating tokens for parsing manifest");
    goto cleanup;
  }

  jsmn_init(&parser);
  tokcount =
      jsmn_parse(&parser, manifest_contents, manifest_len, tokens, tokcount);
  if (tokcount <= 0) {
    ESP_LOGE(RADIO_TAG, "Error while parsing manifest: %d", tokcount);
    goto cleanup;
  }

  if (tokens[0].type != JSMN_OBJECT) {
    ESP_LOGE(RADIO_TAG, "Root of manifest is not an object");
    goto cleanup;
  }

  jsmntok_t *files = &tokens[1];
  while (files < tokens + tokcount) {
    if (files->type != JSMN_STRING) {
      ESP_LOGE(RADIO_TAG, "Expected string key in JSON object, got %d",
               files->type);
      goto cleanup;
    }

    if (strncmp(manifest_contents + files->start, "files",
                files->end - files->start) == 0) {
      files++;
      break;
    }

    // Otherwise skip the key and the value
    files++;
    files += jsmn_value_len(files);
  }

  // files should now point to a JSMN_ARRAY token of objects
  if (files->type != JSMN_ARRAY) {
    ESP_LOGE(RADIO_TAG, "Expected files manifest key to be array, got %d",
             files->type);
    goto cleanup;
  }

  manifest = calloc(1, sizeof(file_cache_manifest));
  if (manifest == NULL) {
    ESP_LOGE(RADIO_TAG, "Error while allocating manifest");
    goto cleanup;
  }
  manifest->entry_count = files->size;
  manifest->entries =
      calloc(manifest->entry_count, sizeof(file_cache_manifest_entry));
  if (manifest->entries == NULL) {
    ESP_LOGE(RADIO_TAG, "Error while allocating manifest entries");
    goto cleanup;
  }

  jsmntok_t *file = files + 1;
  for (int i = 0; i < manifest->entry_count;
       i++, file += jsmn_value_len(file)) {
    if (file->type != JSMN_OBJECT) {
      ESP_LOGE(RADIO_TAG,
               "Expected object in array of files in manifest, got %d for "
               "array entry %d",
               file->type, i);
      goto cleanup;
    }

    // For a given file, we need to find the "name", "url", "hash", and "size"
    // keys
    jsmntok_t *key = file + 1;
    for (int j = 0; j < file->size; j++, key += 1 + jsmn_value_len(key + 1)) {
      if (key->type != JSMN_STRING) {
        ESP_LOGE(RADIO_TAG, "Expected string key in file object, got %d",
                 key->type);
        goto cleanup;
      }

      if (strncmp("name", manifest_contents + key->start,
                  key->end - key->start) == 0) {
        manifest->entries[i].name = strndup(manifest_contents + key[1].start,
                                            key[1].end - key[1].start);
      } else if (strncmp("url", manifest_contents + key->start,
                         key->end - key->start) == 0) {
        manifest->entries[i].url = strndup(manifest_contents + key[1].start,
                                           key[1].end - key[1].start);
      } else if (strncmp("hash", manifest_contents + key->start,
                         key->end - key->start) == 0) {
        manifest->entries[i].hash = strndup(manifest_contents + key[1].start,
                                            key[1].end - key[1].start);
      } else if (strncmp("size", manifest_contents + key->start,
                         key->end - key->start) == 0) {
        char size_str[key[1].end - key[1].start + 1];
        memcpy(size_str, manifest_contents + key[1].start,
               key[1].end - key[1].start);
        size_str[key[1].end - key[1].start] = '\0';
        manifest->entries[i].size = strtoul(size_str, NULL, 10);
      }
    }

    if (manifest->entries[i].name == NULL || manifest->entries[i].url == NULL ||
        manifest->entries[i].hash == NULL || manifest->entries[i].size == 0) {
      ESP_LOGE(RADIO_TAG, "Missing required key in files array entry %d", i);
      goto cleanup;
    }

    size_t hash_filename_length = strlen(manifest->entries[i].hash) + 1;
    const char *extension = strrchr(manifest->entries[i].name, '.');
    if (extension != NULL) {
      hash_filename_length += strlen(extension);
    }
    manifest->entries[i].hash_filename = calloc(1, hash_filename_length);
    if (manifest->entries[i].hash_filename == NULL) {
      ESP_LOGE(RADIO_TAG, "Error allocating hash filename");
      goto cleanup;
    }
    snprintf((char *)manifest->entries[i].hash_filename, hash_filename_length,
             "%s%s", manifest->entries[i].hash,
             extension != NULL ? extension : "");
  }

  qsort(manifest->entries, manifest->entry_count,
        sizeof(file_cache_manifest_entry), manifest_entry_compare_name);

  success = true;

cleanup:
  if (tokens != NULL) {
    free(tokens);
  }

  if (!success) {
    free_file_cache_manifest(manifest);
    manifest = NULL;
  }

  return manifest;
}

static esp_err_t refresh() {
  esp_err_t ret = ESP_OK;
  char *current_manifest_url = NULL;
  char *manifest_contents = NULL;
  file_cache_manifest *manifest = NULL;
  size_t manifest_length = 0;
  char **hashes = NULL;
  DIR *cache_dir = NULL;

  xSemaphoreTake(manifest_url_mutex, portMAX_DELAY);
  if (manifest_url != NULL) {
    current_manifest_url = strdup(manifest_url);
  }
  xSemaphoreGive(manifest_url_mutex);

  ESP_GOTO_ON_FALSE(current_manifest_url != NULL, ESP_OK, cleanup, RADIO_TAG,
                    "No file manifest URL set");

  // Here's the process:
  // - Ensure /data/cache exists
  // - Fetch the manifest file to /data/cache/manifest.json.new
  // - Parse the manifest
  // - Iterate over the manifest and fetch any files that we're missing
  // - Move the manifest to /data/cache/manifest.json
  // - Delete any files in /data/cache that are not in the manifest

  int error = mkdir(FILE_CACHE_PREFIX, 0777);
  ESP_GOTO_ON_FALSE(error == 0 || errno == EEXIST, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error creating cache directory: %d (%s)", errno,
                    strerror(errno));

  ESP_LOGD(RADIO_TAG, "Fetching manifest from %s", current_manifest_url);
  size_t manifest_len;
  ESP_GOTO_ON_ERROR(fetch_file(current_manifest_url, NULL, &manifest_contents,
                               &manifest_len, NULL),
                    cleanup, RADIO_TAG, "Failed to fetch manifest: %d",
                    err_rc_);
  ESP_GOTO_ON_FALSE(manifest_contents != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                    "Unable to fetch manfiest file contents");

  uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
  size_t hash_length;
  psa_status_t status =
      psa_hash_compute(PSA_ALG_SHA_256, (const uint8_t *)manifest_contents,
                       manifest_len, hash, sizeof(hash), &hash_length);
  ESP_GOTO_ON_FALSE(status == PSA_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error computing manifest hash: %" PRId32, status);
  if (memcmp(hash, manifest_hash, sizeof(hash)) == 0) {
    ESP_LOGD(RADIO_TAG, "Manifest hash matches, skipping refresh");
    goto cleanup;
  }

  int fd = open(FILE_CACHE_PREFIX "/manifest.json.new",
                O_CREAT | O_TRUNC | O_WRONLY, 0666);
  ESP_GOTO_ON_FALSE(fd != -1, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error creating manifest file: %d (%s)", errno,
                    strerror(errno));
  ssize_t written = write(fd, manifest_contents, manifest_len);
  fsync(fd);
  close(fd);
  ESP_GOTO_ON_FALSE(written == manifest_len, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error writing manifest file: %d (%s)", errno,
                    strerror(errno));

  manifest = parse_manifest(manifest_contents, manifest_len);
  ESP_GOTO_ON_FALSE(manifest != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                    "Unable to parse file manifest");

  free(manifest_contents);
  manifest_contents = NULL;

  for (int i = 0; i < manifest->entry_count; i++) {
    file_cache_manifest_entry *entry = &manifest->entries[i];
    char path[128];
    char tmppath[128];
    snprintf(path, sizeof(path), FILE_CACHE_PREFIX "/%s", entry->hash_filename);
    snprintf(tmppath, sizeof(tmppath), FILE_CACHE_PREFIX "/%s.new",
             entry->hash_filename);
    struct stat st;
    if (stat(path, &st) == 0 && st.st_size == entry->size) {
      ESP_LOGV(RADIO_TAG, "Skipping %s (%s) as it already exists", entry->url,
               entry->name);
      continue;
    }

    unlink(tmppath);
    ESP_LOGD(RADIO_TAG, "Fetching %s (%s) to %s", entry->url, entry->name,
             tmppath);
    ESP_GOTO_ON_ERROR(
        fetch_file(entry->url, tmppath, NULL, NULL, current_manifest_url),
        cleanup, RADIO_TAG, "Failed to fetch file %s: %d", entry->name,
        err_rc_);

    ESP_GOTO_ON_FALSE(stat(tmppath, &st) == 0 && st.st_size == entry->size,
                      ESP_FAIL, cleanup, RADIO_TAG,
                      "File %s (%s) fetched to %s is not the expected size",
                      entry->url, entry->name, tmppath);

    error = rename(tmppath, path);
    ESP_GOTO_ON_FALSE(error == 0, ESP_FAIL, cleanup, RADIO_TAG,
                      "Error moving file %s to %s: %d (%s)", path, path, errno,
                      strerror(errno));
  }

  manifest_length = manifest->entry_count;
  hashes = calloc(manifest_length, sizeof(char *));
  ESP_GOTO_ON_FALSE(hashes != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error allocating hash array");
  for (int i = 0; i < manifest_length; i++) {
    hashes[i] = strdup(manifest->entries[i].hash_filename);
    ESP_GOTO_ON_FALSE(hashes[i] != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                      "Error allocating hash string");
  }

  xSemaphoreTake(manifest_cache_mutex, portMAX_DELAY);
  if (manifest_cache != NULL) {
    free_file_cache_manifest(manifest_cache);
  }
  manifest_cache = manifest;
  manifest = NULL;
  xSemaphoreGive(manifest_cache_mutex);

  error = rename(FILE_CACHE_PREFIX "/manifest.json.new",
                 FILE_CACHE_PREFIX "/manifest.json");
  ESP_GOTO_ON_FALSE(error == 0, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error moving manifest file: %d (%s)", errno,
                    strerror(errno));

  cache_dir = opendir(FILE_CACHE_PREFIX);
  ESP_GOTO_ON_FALSE(cache_dir != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error opening cache directory: %d (%s)", errno,
                    strerror(errno));

  qsort(hashes, manifest_length, sizeof(char *), compare_hash);
  const struct dirent *entry;
  while ((entry = readdir(cache_dir)) != NULL) {
    if (entry->d_type != DT_REG ||
        strcmp(entry->d_name, "manifest.json") == 0) {
      continue;
    }

    const char *key = &entry->d_name[0];
    char **found =
        bsearch(&key, hashes, manifest_length, sizeof(char *), compare_hash);
    if (found == NULL) {
      char path[strlen(FILE_CACHE_PREFIX) + 1 + strlen(entry->d_name) + 1];
      snprintf(path, sizeof(path), FILE_CACHE_PREFIX "/%s", entry->d_name);
      ESP_LOGV(RADIO_TAG, "Deleting unreferenced file %s", path);
      unlink(path);
    }
  }

  ESP_LOGI(RADIO_TAG, "File cache refreshed");

cleanup:
  if (cache_dir) {
    closedir(cache_dir);
  }

  if (hashes) {
    for (int i = 0; i < manifest_length; i++) {
      free(hashes[i]);
    }
    free(hashes);
  }

  if (manifest) {
    free_file_cache_manifest(manifest);
  }

  if (manifest_contents) {
    free(manifest_contents);
  }

  if (current_manifest_url) {
    free(current_manifest_url);
  }

  return ret;
}

static esp_err_t load_manifest() {
  esp_err_t ret = ESP_OK;
  int fd = -1;
  char *manifest_contents = NULL;

  fd = open(FILE_CACHE_PREFIX "/manifest.json", O_RDONLY);
  if (fd == -1) {
    ESP_LOGW(
        RADIO_TAG,
        "Unable to open manifest.json file (may not yet be fetched): %d (%s)",
        errno, strerror(errno));
    ret = ESP_OK;
    goto cleanup;
  }

  struct stat st;
  ESP_GOTO_ON_FALSE(fstat(fd, &st) == 0, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error getting manifest.json file size: %d (%s)", errno,
                    strerror(errno));

  manifest_contents = calloc(1, st.st_size + 1);
  ESP_GOTO_ON_FALSE(manifest_contents != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error allocating buffer for manifest.json");

  ssize_t read_len = read(fd, manifest_contents, st.st_size);
  ESP_GOTO_ON_FALSE(read_len == st.st_size, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error reading manifest.json file: %d (%s)", errno,
                    strerror(errno));

  uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
  size_t hash_len = 0;
  psa_status_t status =
      psa_hash_compute(PSA_ALG_SHA_256, (const uint8_t *)manifest_contents,
                       st.st_size, hash, sizeof(hash), &hash_len);
  ESP_GOTO_ON_FALSE(status == PSA_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error computing manifest hash: %" PRId32, status);

  file_cache_manifest *manifest = parse_manifest(manifest_contents, st.st_size);
  ESP_GOTO_ON_FALSE(manifest != NULL, ESP_FAIL, cleanup, RADIO_TAG,
                    "Error parsing manifest.json");

  xSemaphoreTake(manifest_cache_mutex, portMAX_DELAY);
  if (manifest_cache != NULL) {
    free_file_cache_manifest(manifest_cache);
  }
  manifest_cache = manifest;
  memcpy(manifest_hash, hash, sizeof(manifest_hash));
  xSemaphoreGive(manifest_cache_mutex);

cleanup:
  if (manifest_contents) {
    free(manifest_contents);
  }

  if (fd != -1) {
    close(fd);
  }

  return ret;
}

static void file_cache_task(void *context) {
  things_subscribe_attribute("file_manifest", url_callback);

  while (true) {
    xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    esp_err_t err = refresh();
    xSemaphoreTake(manifest_cache_mutex, portMAX_DELAY);
    manifest_cache_last_update = time(NULL);
    manifest_cache_last_update_err = err;
    xSemaphoreGive(manifest_cache_mutex);
    xSemaphoreTake(subscriber_mutex, portMAX_DELAY);
    struct subscriber *sub;
    TAILQ_FOREACH(sub, &subscribers, entries) { sub->callback(sub->arg); }
    xSemaphoreGive(subscriber_mutex);
    if (err != ESP_OK) {
      uint32_t wait = 10000 + esp_random() % 5000;
      ESP_LOGE(RADIO_TAG,
               "Failed to refresh file cache: %d; retrying in %" PRIu32
               " seconds",
               err, wait / 1000);
      xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(wait));
      continue;
    }
    things_force_telemetry(telemetry_index);

    // Wait until we get woken up, indicating a new value for the manifest URL
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
  }

  vTaskDelete(NULL);
}

esp_err_t file_cache_init(void) {
  subscriber_mutex = xSemaphoreCreateMutex();
  manifest_url_mutex = xSemaphoreCreateMutex();
  manifest_cache_mutex = xSemaphoreCreateMutex();

  // load_manifest will log its own failures, and failures are soft anyway
  load_manifest();

  // Push all work to a task because we need to wait until storage has been
  // mounted
  task_handle = xTaskCreateStaticPinnedToCore(
      file_cache_task, "file_cache",
      sizeof(file_cache_task_stack) / sizeof(StackType_t), NULL, 4,
      file_cache_task_stack, &file_cache_task_buffer, 0);
  ESP_RETURN_ON_FALSE(task_handle, ESP_FAIL, RADIO_TAG,
                      "Failed to create file cache task");

  things_register_telemetry_generator(telemetry_generator, "file_cache",
                                      &telemetry_index);

  return ESP_OK;
}

char *file_cache_get_hash(const char *name) {
  char *ret = NULL;

  xSemaphoreTake(manifest_cache_mutex, portMAX_DELAY);
  if (manifest_cache == NULL) {
    goto cleanup;
  }

  file_cache_manifest_entry key = {
      .name = name,
  };
  const file_cache_manifest_entry *entry =
      bsearch(&key, manifest_cache->entries, manifest_cache->entry_count,
              sizeof(file_cache_manifest_entry), manifest_entry_compare_name);
  if (entry == NULL) {
    goto cleanup;
  }

  ret = strdup(entry->hash);

cleanup:
  xSemaphoreGive(manifest_cache_mutex);
  return ret;
}

int file_cache_open_file(const char *name, char (*hash)[256]) {
  int ret = -1;

  xSemaphoreTake(manifest_cache_mutex, portMAX_DELAY);
  if (manifest_cache == NULL) {
    errno = ENOENT;
    goto cleanup;
  }

  file_cache_manifest_entry key = {
      .name = name,
  };
  const file_cache_manifest_entry *entry =
      bsearch(&key, manifest_cache->entries, manifest_cache->entry_count,
              sizeof(file_cache_manifest_entry), manifest_entry_compare_name);
  if (entry == NULL) {
    errno = ENOENT;
    goto cleanup;
  }

  char path[128];
  snprintf(path, sizeof(path), FILE_CACHE_PREFIX "/%s", entry->hash_filename);
  ret = open(path, O_RDONLY);

  if (ret < 0) {
    goto cleanup;
  }

  if (hash != NULL) {
    strncpy(hash[0], entry->hash, 256);
  }

cleanup:
  xSemaphoreGive(manifest_cache_mutex);
  return ret;
}

esp_err_t file_cache_subscribe_refresh(file_cache_refresh_cb cb, void *ctx) {
  xSemaphoreTake(subscriber_mutex, portMAX_DELAY);
  struct subscriber *sub = calloc(1, sizeof(struct subscriber));
  ESP_RETURN_ON_FALSE(sub != NULL, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate subscriber");
  sub->callback = cb;
  sub->arg = ctx;
  TAILQ_INSERT_TAIL(&subscribers, sub, entries);
  xSemaphoreGive(subscriber_mutex);
  return ESP_OK;
}