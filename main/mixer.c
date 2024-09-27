#include "mixer.h"
#include "main.h"

#include "esp_log.h"
#include "esp_check.h"
#include "i2s_stream.h"
#include "audio_pipeline.h"
#include "esp_downmix.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <string.h>
#include <sys/queue.h>

#define MIXER_SAMPLE_NUM 960 // 20ms of audio at 48kHz

static SemaphoreHandle_t mixer_mutex = NULL;
static void *downmix_handle = NULL;
static EXT_RAM_BSS_ATTR esp_downmix_input_info_t downmix_source_info[SOURCE_NUM_MAX] = {};
static esp_downmix_info_t downmix_info = {
    .source_info = downmix_source_info,
    .source_num = 0,
    .out_ctx = ESP_DOWNMIX_OUT_CTX_NORMAL,
    .mode = ESP_DOWNMIX_WORK_MODE_SWITCH_ON,
    .output_type = ESP_DOWNMIX_OUTPUT_TYPE_TWO_CHANNEL,
};
static EXT_RAM_BSS_ATTR unsigned char downmix_buffer[MIXER_SAMPLE_NUM * 2 * 2]; // 20ms of audio at 48kHz, 16-bit, stereo
static TaskHandle_t mixer_task_handle = NULL;

struct mixer_channel
{
  TAILQ_ENTRY(mixer_channel)
  entries;
  char in_buffer[MIXER_SAMPLE_NUM * 2 * 2]; // 20ms of audio at 48kHz, 16-bit, stereo
  mixer_read_callback_t callback;
  void *ctx;
};

TAILQ_HEAD(mixer_channel_list, mixer_channel);

struct mixer_channel_list mixer_channels = TAILQ_HEAD_INITIALIZER(mixer_channels);

// Must be called while holding mixer_mutex
static esp_err_t mixer_reopen()
{
  if (downmix_handle)
  {
    esp_downmix_close(downmix_handle);
    downmix_handle = NULL;
  }

  if (downmix_info.source_num < 2)
  {
    return ESP_OK;
  }

  memset(downmix_source_info, 0, sizeof(downmix_source_info));
  int i;
  mixer_channel_t chan = TAILQ_FIRST(&mixer_channels);
  for (i = 0; i < downmix_info.source_num && chan != NULL; i++, chan = TAILQ_NEXT(chan, entries))
  {
    downmix_source_info[i].samplerate = 48000;
    downmix_source_info[i].channel = 2;
    downmix_source_info[i].bits_num = 16;
    downmix_source_info[i].gain[0] = 0;
    downmix_source_info[i].gain[1] = 0;
    downmix_source_info[i].transit_time = 0;
  }
  if (i != downmix_info.source_num)
  {
    ESP_LOGE(RADIO_TAG, "Mismatch between source info and channels");
    return ESP_FAIL;
  }

  downmix_handle = esp_downmix_open(&downmix_info);
  if (!downmix_handle)
  {
    ESP_LOGE(RADIO_TAG, "Failed to reopen downmix handle");
    return ESP_FAIL;
  }
  return ESP_OK;
}

// Run fetching and downmixing in a separate task so that it can run while I2S buffer is playing out
static void mixer_task(void *arg)
{
  unsigned char *inbufs[SOURCE_NUM_MAX] = {};
  while (true)
  {
    xSemaphoreTake(mixer_mutex, portMAX_DELAY);

    switch (downmix_info.source_num)
    {
    case 0:
      memset(downmix_buffer, 0, sizeof(downmix_buffer));
      break;

    case 1:
      mixer_channel_t chan = TAILQ_FIRST(&mixer_channels);
      memset(downmix_buffer, 0, sizeof(downmix_buffer));
      int read = chan->callback(chan->ctx, (char *)downmix_buffer, MIXER_SAMPLE_NUM * 2 * 2, portMAX_DELAY);
      if (read < 0)
      {
        ESP_LOGE(RADIO_TAG, "Failed to read audio data from mixer channel: %d", read);
      }
      break;

    default:
    {
      mixer_channel_t chan = TAILQ_FIRST(&mixer_channels);
      for (int i = 0; i < downmix_info.source_num && chan != NULL; i++, chan = TAILQ_NEXT(chan, entries))
      {
        inbufs[i] = (unsigned char *)chan->in_buffer;
        memset(chan->in_buffer, 0, sizeof(chan->in_buffer));
        int read = chan->callback(chan->ctx, chan->in_buffer, MIXER_SAMPLE_NUM * 2 * 2, portMAX_DELAY);
        if (read < 0)
        {
          ESP_LOGE(RADIO_TAG, "Failed to read audio data from mixer channel: %d", read);
        }
      }

      int ret = esp_downmix_process(downmix_handle, inbufs, (unsigned char *)downmix_buffer, sizeof(downmix_buffer) / (2 * 2), ESP_DOWNMIX_WORK_MODE_SWITCH_ON);
      if (ret < 0)
      {
        ESP_LOGE(RADIO_TAG, "Failed to downmix audio: %d", ret);
      }
    }
    }

    xSemaphoreGive(mixer_mutex);
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
  }
}

static audio_element_err_t mixer_read_cb(
    audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
  xSemaphoreTake(mixer_mutex, portMAX_DELAY);
  memcpy(buffer, downmix_buffer, len);
  xTaskNotifyGive(mixer_task_handle);
  xSemaphoreGive(mixer_mutex);
  return len;
}

esp_err_t mixer_init()
{
  mixer_mutex = xSemaphoreCreateMutex();

  ESP_RETURN_ON_ERROR(mixer_reopen(), RADIO_TAG, "Failed to reopen mixer");
  xTaskCreatePinnedToCore(mixer_task, "mixer_task", 30 * 1024, NULL, 5, &mixer_task_handle, 1);

  // Create pipeline
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
  ESP_RETURN_ON_FALSE(pipeline, ESP_FAIL, RADIO_TAG, "Failed to create audio pipeline");

  // Create I2S output
  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(I2S_NUM_0, 48000, 16, AUDIO_STREAM_WRITER);
  // Need space for 20ms of audio at 48kHz, 16-bit, stereo
  // TODO: adapt to packet sizes?
  i2s_cfg.buffer_len = MIXER_SAMPLE_NUM * 2 * 2;
  i2s_cfg.chan_cfg.dma_frame_num = MIXER_SAMPLE_NUM; // 20ms of audio
  i2s_cfg.task_core = 0;
  audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);
  ESP_RETURN_ON_FALSE(i2s_stream_writer, ESP_FAIL, RADIO_TAG, "Failed to create I2S stream");
  i2s_stream_set_clk(i2s_stream_writer, 48000, 16, 2);
  audio_element_set_read_cb(i2s_stream_writer, mixer_read_cb, NULL);

  // Connect I2S output to pipeline and start
  ESP_RETURN_ON_ERROR(audio_pipeline_register(pipeline, i2s_stream_writer, "i2s"), RADIO_TAG, "Failed to register I2S stream to pipeline");
  ESP_RETURN_ON_ERROR(audio_pipeline_link(pipeline, (const char *[]){"i2s"}, 1), RADIO_TAG, "Failed to link I2S stream to pipeline");
  ESP_RETURN_ON_ERROR(audio_pipeline_run(pipeline), RADIO_TAG, "Failed to start audio pipeline");

  return ESP_OK;
}

esp_err_t mixer_play_audio(mixer_read_callback_t callback, void *ctx, mixer_channel_t *slot)
{
  ESP_RETURN_ON_FALSE(mixer_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG, "mixer_play_audio must be called after mixer_init");
  ESP_RETURN_ON_FALSE(callback, ESP_ERR_INVALID_ARG, RADIO_TAG, "mixer_play_audio callback must not be NULL");
  ESP_RETURN_ON_FALSE(slot, ESP_ERR_INVALID_ARG, RADIO_TAG, "mixer_play_audio slot must not be NULL");

  mixer_channel_t channel = calloc(1, sizeof(struct mixer_channel));
  ESP_RETURN_ON_FALSE(channel, ESP_ERR_NO_MEM, RADIO_TAG, "Failed to allocate memory for mixer channel");
  channel->callback = callback;
  channel->ctx = ctx;

  esp_err_t ret = ESP_OK;

  xSemaphoreTake(mixer_mutex, portMAX_DELAY);
  ESP_GOTO_ON_FALSE(downmix_info.source_num + 1 <= SOURCE_NUM_MAX, ESP_ERR_NO_MEM, cleanup, RADIO_TAG, "No available downmix sources");
  TAILQ_INSERT_TAIL(&mixer_channels, channel, entries);
  downmix_info.source_num++;
  *slot = channel;
  channel = NULL;
  ESP_GOTO_ON_ERROR(mixer_reopen(), cleanup, RADIO_TAG, "Failed to reopen mixer");

cleanup:
  xSemaphoreGive(mixer_mutex);
  if (channel)
  {
    free(channel);
  }
  return ret;
}

esp_err_t mixer_stop_audio(mixer_channel_t slot)
{
  ESP_RETURN_ON_FALSE(mixer_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG, "mixer_play_audio must be called after mixer_init");
  esp_err_t ret = ESP_OK;

  xSemaphoreTake(mixer_mutex, portMAX_DELAY);
  TAILQ_REMOVE(&mixer_channels, slot, entries);
  downmix_info.source_num--;
  ESP_GOTO_ON_ERROR(mixer_reopen(), cleanup, RADIO_TAG, "Failed to reopen mixer");

cleanup:
  xSemaphoreGive(mixer_mutex);
  free(slot);
  return ret;
}
