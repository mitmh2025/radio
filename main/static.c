#include "static.h"
#include "file_cache.h"
#include "main.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "opusfile.h"

#include "audio_element.h"
#include "esp_check.h"
#include "esp_log.h"

#define STATIC_SAMPLE_COUNT (5760)                   // 120ms of audio at 48kHz
#define STATIC_BUFFER_SIZE (STATIC_SAMPLE_COUNT * 2) // 16-bit mono

static SemaphoreHandle_t mutex = NULL;
static char opus_hash[256] = {};
static unsigned char *opus_data = NULL;
static size_t opus_data_len = 0;
static OggOpusFile *opus_file = NULL;
static ringbuf_handle_t rb = NULL;

static StaticTask_t task_buffer;
static EXT_RAM_BSS_ATTR StackType_t task_stack[30 * 1024 / sizeof(StackType_t)];

static void cache_updated(void *ctx) {
  char new_opus_hash[256] = {};
  unsigned char *new_opus_data = NULL;
  size_t new_opus_data_len = 0;
  int fd = -1;

  fd = file_cache_open_file("static.opus", &new_opus_hash);
  if (fd < 0) {
    ESP_LOGD(RADIO_TAG, "Failed to open static.opus: %d", errno);
    goto cleanup;
  }

  bool changed = false;
  xSemaphoreTake(mutex, portMAX_DELAY);
  changed = strncmp(opus_hash, new_opus_hash, sizeof(opus_hash)) != 0;
  xSemaphoreGive(mutex);
  if (!changed) {
    goto cleanup;
  }

  struct stat st;
  if (fstat(fd, &st) < 0) {
    ESP_LOGE(RADIO_TAG, "Failed to stat static.opus: %d", errno);
    goto cleanup;
  }

  new_opus_data = malloc(st.st_size);
  if (!new_opus_data) {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for static.opus");
    goto cleanup;
  }

  if (read(fd, new_opus_data, st.st_size) != st.st_size) {
    ESP_LOGE(RADIO_TAG, "Failed to read static.opus: %d", errno);
    goto cleanup;
  }

  new_opus_data_len = st.st_size;

  xSemaphoreTake(mutex, portMAX_DELAY);
  if (opus_file) {
    op_free(opus_file);
    opus_file = NULL;
  }

  if (opus_data) {
    free(opus_data);
    opus_data = NULL;
  }

  memcpy(opus_hash, new_opus_hash, sizeof(opus_hash));
  opus_data = new_opus_data;
  opus_data_len = new_opus_data_len;
  xSemaphoreGive(mutex);

  new_opus_data = NULL;

cleanup:
  if (new_opus_data) {
    free(new_opus_data);
  }

  if (fd >= 0) {
    close(fd);
  }
}

static uint16_t lookback = 0;
static int static_read_random(void *ctx, char *data, int len,
                              TickType_t ticks_to_wait) {
  int16_t *samples = (int16_t *)data;
  for (int i = 0; i < len / 2; i++) {
    uint16_t sample = rand() & 0xFFFF;
    lookback = lookback - (lookback >> 2) + (sample >> 2);

    // Hard code a gain of 1/8
    samples[i] = lookback >> 3;
  }
  return len;
}

static int static_read_opusfile(void *ctx, char *data, int len,
                                TickType_t ticks_to_wait) {
  if (!opus_file) {
    int error = 0;
    opus_file = op_open_memory(opus_data, opus_data_len, &error);
    if (!opus_file) {
      ESP_LOGE(RADIO_TAG, "Failed to open static.opus: %d", error);
      free(opus_data);
      opus_data = NULL;
      return static_read_random(ctx, data, len, ticks_to_wait);
    }
    op_set_gain_offset(opus_file, OP_HEADER_GAIN, -18 * 256);
  }

  int bytes_read = 0;
  while (true) {
    opus_int16 *samples = (opus_int16 *)(data + bytes_read);
    int ret = op_read(opus_file, samples, len - bytes_read, NULL);
    if (ret < 0) {
      ESP_LOGE(RADIO_TAG, "Failed to read opus file: %d", ret);
      return AEL_IO_FAIL;
    }

    bytes_read += ret * 2;
    if (len - bytes_read < 960) {
      break;
    }

    if (ret == 0) {
      // We must have hit the end of the file, so loop back to the beginning
      op_raw_seek(opus_file, 0);
    }
  }

  return bytes_read;
}

static void static_task(void *ctx) {
  char buffer[STATIC_BUFFER_SIZE];

  while (true) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int ret;
    if (opus_data) {
      ret =
          static_read_opusfile(ctx, buffer, STATIC_BUFFER_SIZE, portMAX_DELAY);
    } else {
      ret = static_read_random(ctx, buffer, STATIC_BUFFER_SIZE, portMAX_DELAY);
    }
    xSemaphoreGive(mutex);

    rb_write(rb, buffer, ret, portMAX_DELAY);
  }
}

esp_err_t static_init(void) {
  mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(mutex != NULL, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create mutex");

  rb = rb_create(STATIC_BUFFER_SIZE, 2);

  file_cache_subscribe_refresh(cache_updated, NULL);
  cache_updated(NULL);

  TaskHandle_t ret = xTaskCreateStaticPinnedToCore(
      static_task, "static", sizeof(task_stack) / sizeof(StackType_t), NULL, 10,
      task_stack, &task_buffer, 1);
  ESP_RETURN_ON_FALSE(ret != NULL, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create static task");

  return ESP_OK;
}

int static_read_audio(void *ctx, char *data, int len,
                      TickType_t ticks_to_wait) {
  return rb_read(rb, data, len, ticks_to_wait);
}
