#include "captive_http_server.h"
#include "http_utils.h"
#include "main.h"
#include "wifi.h"

#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define FILE_LIST()                                                            \
  FILE_LIST_ITEM("/", index_html_gz, "text/html; charset=utf-8")               \
  FILE_LIST_ITEM("/style.css", style_css_gz, "text/css")

#define FILE_LIST_ITEM(uri, name, mime)                                        \
  extern const char _binary_##name##_start[];                                  \
  extern const char _binary_##name##_end[];
FILE_LIST()
#undef FILE_LIST_ITEM

typedef struct {
  const char *uri;
  const char *mime;
  const char *start;
  const char *end;
} static_file_t;

static static_file_t files[] = {
#define FILE_LIST_ITEM(uri, name, mime)                                        \
  {uri, mime, _binary_##name##_start, _binary_##name##_end},
    FILE_LIST()};

static SemaphoreHandle_t http_server_lock = NULL;
static httpd_handle_t http_server = NULL;

static esp_err_t serve_file(httpd_req_t *req) {
  const static_file_t *uri = req->user_ctx;
  if (uri->mime != NULL) {
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, uri->mime), RADIO_TAG,
                        "Failed to set MIME type: %d", err_rc_);
  }
  // All of our files are gzipped
  ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Content-Encoding", "gzip"),
                      RADIO_TAG, "Failed to set Content-Encoding: %d", err_rc_);
  ESP_RETURN_ON_ERROR(httpd_resp_send(req, uri->start, uri->end - uri->start),
                      RADIO_TAG, "Failed to send file: %d", err_rc_);
  return ESP_OK;
}

static esp_err_t serve_scan(httpd_req_t *req) {
  ESP_RETURN_ON_ERROR(wifi_force_scan(), RADIO_TAG,
                      "Failed to scan wifi networks: %d", err_rc_);

  ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, "application/json"), RADIO_TAG,
                      "Failed to set MIME type: %d", err_rc_);

  uint16_t ap_count = 0;
  ESP_RETURN_ON_ERROR(esp_wifi_scan_get_ap_num(&ap_count), RADIO_TAG,
                      "Failed to get AP count: %d", err_rc_);

  // We're going to generate JSON of the form (without spaces):
  //
  // {"networks": [
  //   {"ssid": [1, 2, 3], "security": "none" | "psk", "rssi": -50},
  // ]}
  size_t max_ssid_len = 32;
  size_t response_len =
      strlen("{\"networks\":[]}") +
      ap_count * (strlen("{\"ssid\":[],\"security\":\"none\",\"rssi\":-127},") +
                  (4 * max_ssid_len)) +
      1;
  char *buffer = calloc(1, response_len);
  ESP_RETURN_ON_FALSE(buffer, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate memory for response");
  esp_err_t ret = ESP_OK;

  size_t pos = 0;
  pos += snprintf(buffer + pos, response_len - pos, "{\"networks\":[");
  bool first = true;
  while (true) {
    wifi_ap_record_t ap_info;
    ret = esp_wifi_scan_get_ap_record(&ap_info);
    if (ret == ESP_FAIL) {
      break;
    }
    ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to get AP record: %d",
                      ret);

    if (!first) {
      pos += snprintf(buffer + pos, response_len - pos, ",");
    }
    first = false;

    const char *security = "";
    switch (ap_info.authmode) {
    case WIFI_AUTH_OPEN:
    case WIFI_AUTH_OWE:
      security = "none";
      break;
    case WIFI_AUTH_WEP:
    case WIFI_AUTH_WPA_PSK:
    case WIFI_AUTH_WPA2_PSK:
    case WIFI_AUTH_WPA_WPA2_PSK:
    case WIFI_AUTH_WPA3_PSK:
    case WIFI_AUTH_WPA2_WPA3_PSK:
    case WIFI_AUTH_WAPI_PSK:
    case WIFI_AUTH_WPA3_EXT_PSK:
    case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
      security = "psk";
      break;
    case WIFI_AUTH_ENTERPRISE:
    case WIFI_AUTH_WPA3_ENT_192:
    case WIFI_AUTH_DPP:
    case WIFI_AUTH_MAX:
      // unsupported auth mode
      continue;
    }

    pos += snprintf(buffer + pos, response_len - pos, "{\"ssid\":[");
    size_t ssid_len = strlen((const char *)ap_info.ssid);
    for (size_t i = 0; i < ssid_len; i++) {
      if (i > 0) {
        pos += snprintf(buffer + pos, response_len - pos, ",");
      }
      pos += snprintf(buffer + pos, response_len - pos, "%" PRIu8,
                      (uint8_t)ap_info.ssid[i]);
    }
    pos += snprintf(buffer + pos, response_len - pos,
                    "],\"security\":\"%s\","
                    "\"rssi\":%" PRId8 "}",
                    security, ap_info.rssi);
  }

  pos += snprintf(buffer + pos, response_len - pos, "]}");

  ret = httpd_resp_send(req, buffer, pos);
  ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to send response: %d",
                    ret);

cleanup:
  if (buffer) {
    free(buffer);
  }
  return ret;
}

static esp_err_t serve_status(httpd_req_t *req) {
  ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, "application/json"), RADIO_TAG,
                      "Failed to set MIME type: %d", err_rc_);
  const char *ssid = wifi_get_alt_ssid();

  if (ssid == NULL) {
    ESP_RETURN_ON_ERROR(
        httpd_resp_send(req, "{\"ssid\": null}", HTTPD_RESP_USE_STRLEN),
        RADIO_TAG, "Failed to send response: %d", err_rc_);
  } else {
    size_t ssid_len = strlen(ssid);
    size_t response_len = strlen("{\"ssid\":[]}") + (4 * ssid_len) + 1;
    char *buffer = calloc(1, response_len);
    ESP_RETURN_ON_FALSE(buffer, ESP_ERR_NO_MEM, RADIO_TAG,
                        "Failed to allocate memory for response");
    size_t pos = 0;
    pos += snprintf(buffer + pos, response_len - pos, "{\"ssid\":[");
    bool first = true;
    for (size_t i = 0; i < ssid_len; i++) {
      if (!first) {
        pos += snprintf(buffer + pos, response_len - pos, ",");
      }
      first = false;
      pos += snprintf(buffer + pos, response_len - pos, "%" PRIu8,
                      (uint8_t)ssid[i]);
    }
    pos += snprintf(buffer + pos, response_len - pos, "]}");

    esp_err_t err = httpd_resp_send(req, buffer, pos);
    free(buffer);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to send response: %d", err);
  }

  return ESP_OK;
}

static esp_err_t serve_save(httpd_req_t *req) {
  size_t max_length =
      strlen("ssid=") + (32 * 3) + strlen("&password=") + (64 * 3) + 1;
  if (req->content_len > max_length) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "Request too large");
    return ESP_OK;
  }
  char *buffer = calloc(1, max_length);

  esp_err_t ret = ESP_OK;

  int bytes = httpd_req_recv(req, buffer, req->content_len);
  ESP_GOTO_ON_FALSE(bytes > 0, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to receive data");

  char ssid[33] = {0};
  char password[65] = {0};

  char *ssid_start = strstr(buffer, "ssid=");
  if (ssid_start == NULL) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "SSID not found");
    ret = ESP_OK;
    goto cleanup;
  }
  ssid_start += strlen("ssid=");
  char *ssid_end = strchr(ssid_start, '&');
  if (ssid_end == NULL) {
    ssid_end = req->content_len + buffer;
  }
  ret = http_utils_percent_decode(ssid_start, ssid_end - ssid_start, ssid,
                                  sizeof(ssid));
  ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to decode SSID: %d", ret);

  char *password_start = strstr(buffer, "password=");
  if (password_start != NULL) {
    password_start += strlen("password=");
    char *password_end = req->content_len + buffer;
    ret =
        http_utils_percent_decode(password_start, password_end - password_start,
                                  password, sizeof(password));
    ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to decode password: %d",
                      ret);
  }

  ret = wifi_test_connection(ssid, password);
  if (ret != ESP_OK) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "Failed to connect to network");
    goto cleanup;
  }

  ret = wifi_set_alt_network(ssid, password);
  if (ret != ESP_OK) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_sendstr(req, "Failed to save network");
    goto cleanup;
  }

  httpd_resp_set_status(req, "200 OK");
  httpd_resp_sendstr(req, "Network saved");

cleanup:
  if (buffer) {
    free(buffer);
  }
  return ret;
}

static esp_err_t serve_not_found(httpd_req_t *req, httpd_err_code_t error) {
  httpd_resp_set_status(req, "302 Temporary Redirect");
  httpd_resp_set_hdr(req, "Location", "/");
  httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t captive_http_server_init() {
  http_server_lock = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(http_server_lock, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create HTTP server lock");

  return ESP_OK;
}

esp_err_t captive_http_server_start() {
  ESP_RETURN_ON_FALSE(http_server_lock, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Must call captive_http_server_init first");
  xSemaphoreTake(http_server_lock, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  if (http_server) {
    goto cleanup;
  }

  ESP_LOGI(RADIO_TAG, "Starting HTTP server");

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  esp_err_t err = httpd_start(&http_server, &config);
  ESP_GOTO_ON_ERROR(err, cleanup, RADIO_TAG, "Failed to start HTTP server: %d",
                    err);

  for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
    httpd_uri_t uri = {
        .uri = files[i].uri,
        .method = HTTP_GET,
        .handler = serve_file,
        .user_ctx = &files[i],
    };
    err = httpd_register_uri_handler(http_server, &uri);
    ESP_GOTO_ON_ERROR(err, cleanup, RADIO_TAG,
                      "Failed to register URI handler");
  }

  httpd_uri_t scan_uri = {
      .uri = "/api/scan",
      .method = HTTP_GET,
      .handler = serve_scan,
  };
  err = httpd_register_uri_handler(http_server, &scan_uri);
  ESP_GOTO_ON_ERROR(err, cleanup, RADIO_TAG, "Failed to register scan URI");
  httpd_uri_t status_uri = {
      .uri = "/api/status",
      .method = HTTP_GET,
      .handler = serve_status,
  };
  err = httpd_register_uri_handler(http_server, &status_uri);
  ESP_GOTO_ON_ERROR(err, cleanup, RADIO_TAG, "Failed to register status URI");
  httpd_uri_t save_uri = {
      .uri = "/api/save",
      .method = HTTP_POST,
      .handler = serve_save,
  };
  err = httpd_register_uri_handler(http_server, &save_uri);

  err = httpd_register_err_handler(http_server, HTTPD_404_NOT_FOUND,
                                   serve_not_found);
  ESP_GOTO_ON_ERROR(err, cleanup, RADIO_TAG, "Failed to register 404 handler");

cleanup:
  xSemaphoreGive(http_server_lock);
  return ret;
}

esp_err_t captive_http_server_stop() {
  esp_err_t ret = ESP_OK;
  xSemaphoreTake(http_server_lock, portMAX_DELAY);

  if (!http_server) {
    goto cleanup;
  }

  ESP_LOGI(RADIO_TAG, "Stopping HTTP server");
  ret = httpd_stop(http_server);
  http_server = NULL;

cleanup:
  xSemaphoreGive(http_server_lock);
  return ret;
}
