#include "mixer.h"
#include "main.h"
#include "static.h"

#include "audio_pipeline.h"
#include "esp_check.h"
#include "esp_downmix.h"
#include "esp_log.h"
#include "i2s_stream.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdatomic.h>
#include <string.h>
#include <sys/queue.h>

#define MIXER_SAMPLE_SIZE (960)                   // 20ms of audio at 48kHz
#define MIXER_BUFFER_SIZE (MIXER_SAMPLE_SIZE * 2) // 16-bit mono

#define MIXER_DUCK_GAIN (-10) // dB

#define MIXER_MUTEX_LOCK()                                                     \
  do {                                                                         \
    int64_t start = esp_timer_get_time();                                      \
    TaskHandle_t holder = xSemaphoreGetMutexHolder(mixer_mutex);               \
    xSemaphoreTake(mixer_mutex, portMAX_DELAY);                                \
    int64_t end = esp_timer_get_time();                                        \
    if (end - start > 10000) {                                                 \
      TaskStatus_t status;                                                     \
      vTaskGetInfo(holder, &status, pdTRUE, eInvalid);                         \
      ESP_LOGW("radio:board",                                                  \
               "%s(%d): Acquiring mixer mutex took %lldus (held by %s)",       \
               __FUNCTION__, __LINE__, end - start, status.pcTaskName);        \
    }                                                                          \
  } while (0)

static SemaphoreHandle_t mixer_mutex = NULL;
static atomic_bool default_static = true;
static void *downmix_handle = NULL;
static EXT_RAM_BSS_ATTR esp_downmix_input_info_t
    downmix_source_info[SOURCE_NUM_MAX] = {};
static esp_downmix_info_t downmix_info = {
    .source_info = downmix_source_info,
    .source_num = 0,
    .out_ctx = ESP_DOWNMIX_OUT_CTX_NORMAL,
    .mode = ESP_DOWNMIX_WORK_MODE_SWITCH_ON,
    .output_type = ESP_DOWNMIX_OUTPUT_TYPE_ONE_CHANNEL,
};
static EXT_RAM_BSS_ATTR unsigned char downmix_buffer[MIXER_BUFFER_SIZE];
static ringbuf_handle_t mixer_rb = NULL;

struct mixer_channel {
  TAILQ_ENTRY(mixer_channel)
  entries;
  char in_buffer[MIXER_BUFFER_SIZE];
  mixer_read_callback_t callback;
  void *ctx;
  bool duck_others;
};

TAILQ_HEAD(mixer_channel_list, mixer_channel);

static struct mixer_channel_list mixer_channels =
    TAILQ_HEAD_INITIALIZER(mixer_channels);

// TODO: add telemetry

// Must be called while holding mixer_mutex
static esp_err_t mixer_reopen() {
  if (downmix_handle) {
    esp_downmix_close(downmix_handle);
    downmix_handle = NULL;
  }

  bool was_ducked = false;
  for (int i = 0;
       i < sizeof(downmix_source_info) / sizeof(downmix_source_info[0]); i++) {
    if (downmix_source_info[i].gain[0] != 0 ||
        downmix_source_info[i].gain[1] != 0) {
      was_ducked = true;
      break;
    }
  }

  if (downmix_info.source_num < 1) {
    return ESP_OK;
  }

  bool will_duck = false;
  mixer_channel_t chan;
  TAILQ_FOREACH(chan, &mixer_channels, entries) {
    if (chan->duck_others) {
      will_duck = true;
      break;
    }
  }

  memset(downmix_source_info, 0, sizeof(downmix_source_info));
  int i;
  chan = TAILQ_FIRST(&mixer_channels);
  for (i = 0; i < downmix_info.source_num && chan != NULL;
       i++, chan = TAILQ_NEXT(chan, entries)) {
    downmix_source_info[i].samplerate = 48000;
    downmix_source_info[i].channel = 1;
    downmix_source_info[i].bits_num = 16;
    downmix_source_info[i].gain[0] =
        was_ducked && !chan->duck_others ? MIXER_DUCK_GAIN : 0;
    downmix_source_info[i].gain[1] =
        will_duck && !chan->duck_others ? MIXER_DUCK_GAIN : 0;
    downmix_source_info[i].transit_time = 250;
  }
  if (i != downmix_info.source_num) {
    ESP_LOGE(RADIO_TAG, "Mismatch between source info and channels");
    return ESP_FAIL;
  }

  downmix_handle = esp_downmix_open(&downmix_info);
  if (!downmix_handle) {
    ESP_LOGE(RADIO_TAG, "Failed to reopen downmix handle");
    return ESP_FAIL;
  }
  return ESP_OK;
}

// Run fetching and downmixing in a separate task so that it can run while I2S
// buffer is playing out
static void mixer_task(void *arg) {
  unsigned char *inbufs[SOURCE_NUM_MAX] = {};
  while (true) {
    xSemaphoreTake(mixer_mutex, portMAX_DELAY);

    if (downmix_info.source_num == 0) {
      memset(downmix_buffer, 0, sizeof(downmix_buffer));

      if (atomic_load(&default_static)) {
        int ret = static_read_audio(NULL, (char *)downmix_buffer,
                                    sizeof(downmix_buffer), portMAX_DELAY);
        if (ret < 0) {
          ESP_LOGE(RADIO_TAG, "Failed to read static audio: %d", ret);
        }
      }
    } else {
      mixer_channel_t chan = TAILQ_FIRST(&mixer_channels);
      for (int i = 0; i < downmix_info.source_num && chan != NULL;
           i++, chan = TAILQ_NEXT(chan, entries)) {
        inbufs[i] = (unsigned char *)chan->in_buffer;
        memset(chan->in_buffer, 0, sizeof(chan->in_buffer));
        int read = chan->callback(chan->ctx, chan->in_buffer, MIXER_BUFFER_SIZE,
                                  portMAX_DELAY);
        if (read < 0) {
          ESP_LOGE(RADIO_TAG,
                   "Failed to read audio data from mixer channel: %d", read);
        }
      }

      int ret = esp_downmix_process(
          downmix_handle, inbufs, (unsigned char *)downmix_buffer,
          sizeof(downmix_buffer) / 2, ESP_DOWNMIX_WORK_MODE_SWITCH_ON);
      if (ret < 0) {
        ESP_LOGE(RADIO_TAG, "Failed to downmix audio: %d", ret);
      }
    }

    xSemaphoreGive(mixer_mutex);
    rb_write(mixer_rb, (char *)downmix_buffer, sizeof(downmix_buffer),
             portMAX_DELAY);
  }
}

static audio_element_err_t mixer_read_cb(audio_element_handle_t self,
                                         char *buffer, int len,
                                         TickType_t ticks_to_wait,
                                         void *context) {
  return rb_read(mixer_rb, buffer, len, ticks_to_wait);
}

esp_err_t mixer_init() {
  mixer_mutex = xSemaphoreCreateMutex();

  ESP_RETURN_ON_ERROR(mixer_reopen(), RADIO_TAG, "Failed to reopen mixer");
  mixer_rb = rb_create(MIXER_BUFFER_SIZE, 1);
  xTaskCreatePinnedToCore(mixer_task, "mixer_task", 30 * 1024, NULL, 15, NULL,
                          1);

  // Create pipeline
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
  ESP_RETURN_ON_FALSE(pipeline, ESP_FAIL, RADIO_TAG,
                      "Failed to create audio pipeline");

  // Create I2S output
  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
      I2S_NUM_0, 48000, 16, AUDIO_STREAM_WRITER);
  // Need space for 20ms of audio at 48kHz, 16-bit, mono
  // TODO: adapt to packet sizes?
  i2s_cfg.buffer_len = MIXER_BUFFER_SIZE;
  i2s_cfg.chan_cfg.dma_frame_num = MIXER_SAMPLE_SIZE; // 20ms of audio
  i2s_cfg.task_core = 1;
  audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);
  ESP_RETURN_ON_FALSE(i2s_stream_writer, ESP_FAIL, RADIO_TAG,
                      "Failed to create I2S stream");
  i2s_stream_set_clk(i2s_stream_writer, 48000, 16, 1);
  audio_element_set_read_cb(i2s_stream_writer, mixer_read_cb, NULL);

  // Connect I2S output to pipeline and start
  ESP_RETURN_ON_ERROR(
      audio_pipeline_register(pipeline, i2s_stream_writer, "i2s"), RADIO_TAG,
      "Failed to register I2S stream to pipeline");
  ESP_RETURN_ON_ERROR(audio_pipeline_link(pipeline, (const char *[]){"i2s"}, 1),
                      RADIO_TAG, "Failed to link I2S stream to pipeline");
  ESP_RETURN_ON_ERROR(audio_pipeline_run(pipeline), RADIO_TAG,
                      "Failed to start audio pipeline");

  return ESP_OK;
}

esp_err_t mixer_play_audio(mixer_read_callback_t callback, void *ctx,
                           int sample_rate, int bits, int channels,
                           bool duck_others, mixer_channel_t *slot) {
  ESP_RETURN_ON_FALSE(mixer_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "mixer_play_audio must be called after mixer_init");
  ESP_RETURN_ON_FALSE(callback, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "mixer_play_audio callback must not be NULL");
  ESP_RETURN_ON_FALSE(slot, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "mixer_play_audio slot must not be NULL");
  ESP_RETURN_ON_FALSE(sample_rate == 48000, ESP_ERR_NOT_SUPPORTED, RADIO_TAG,
                      "Only 48kHz sample rate is supported");
  ESP_RETURN_ON_FALSE(bits == 16, ESP_ERR_NOT_SUPPORTED, RADIO_TAG,
                      "Only 16-bit samples are supported");
  ESP_RETURN_ON_FALSE(channels == 1, ESP_ERR_NOT_SUPPORTED, RADIO_TAG,
                      "Only mono audio is supported");

  mixer_channel_t channel = calloc(1, sizeof(struct mixer_channel));
  ESP_RETURN_ON_FALSE(channel, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate memory for mixer channel");
  channel->callback = callback;
  channel->ctx = ctx;
  channel->duck_others = duck_others;

  esp_err_t ret = ESP_OK;

  xSemaphoreTake(mixer_mutex, portMAX_DELAY);
  ESP_GOTO_ON_FALSE(downmix_info.source_num + 1 <= SOURCE_NUM_MAX,
                    ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "No available downmix sources");
  TAILQ_INSERT_TAIL(&mixer_channels, channel, entries);
  downmix_info.source_num++;
  *slot = channel;
  channel = NULL;
  ESP_GOTO_ON_ERROR(mixer_reopen(), cleanup, RADIO_TAG,
                    "Failed to reopen mixer");

cleanup:
  xSemaphoreGive(mixer_mutex);
  if (channel) {
    free(channel);
  }
  return ret;
}

esp_err_t mixer_stop_audio(mixer_channel_t slot) {
  ESP_RETURN_ON_FALSE(mixer_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "mixer_play_audio must be called after mixer_init");
  esp_err_t ret = ESP_OK;

  xSemaphoreTake(mixer_mutex, portMAX_DELAY);
  TAILQ_REMOVE(&mixer_channels, slot, entries);
  downmix_info.source_num--;
  ESP_GOTO_ON_ERROR(mixer_reopen(), cleanup, RADIO_TAG,
                    "Failed to reopen mixer");

cleanup:
  xSemaphoreGive(mixer_mutex);
  free(slot);
  return ret;
}

esp_err_t mixer_set_default_static(bool enable) {
  ESP_RETURN_ON_FALSE(mixer_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "mixer_play_audio must be called after mixer_init");

  atomic_store(&default_static, enable);

  return ESP_OK;
}