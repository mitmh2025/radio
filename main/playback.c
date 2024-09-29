#include "playback.h"
#include "main.h"
#include "file_cache.h"
#include "mixer.h"

#include "file_stream.h"

#include <string.h>
#include <unistd.h>

#include "esp_check.h"

#include "audio_pipeline.h"
#include "opus_decoder.h"
#include "wav_decoder.h"

esp_err_t playback_file(const char *path, bool duck_others)
{
  int fd = -1;
  audio_pipeline_handle_t pipeline = NULL;
  audio_event_iface_handle_t evt = NULL;
  mixer_channel_t channel = NULL;
  ringbuf_handle_t output = NULL;

  esp_err_t ret = ESP_OK;

  fd = file_cache_open_file(path);
  ESP_RETURN_ON_FALSE(fd >= 0, ESP_FAIL, RADIO_TAG, "Failed to open file: %d", fd);

  audio_pipeline_cfg_t pipeline_cfg = {
      .rb_size = 2048,
  };
  pipeline = audio_pipeline_init(&pipeline_cfg);
  ESP_RETURN_ON_FALSE(pipeline, ESP_FAIL, RADIO_TAG, "Failed to create audio pipeline");

  output = rb_create(960 * 2, 1);

  file_stream_cfg_t file_cfg = FILE_STREAM_CFG_DEFAULT();
  file_cfg.fd = fd;
  audio_element_handle_t file_stream = file_stream_init(&file_cfg);
  ESP_GOTO_ON_FALSE(file_stream, ESP_FAIL, cleanup, RADIO_TAG, "Failed to create file stream");
  ESP_GOTO_ON_ERROR(audio_pipeline_register(pipeline, file_stream, "file"), cleanup, RADIO_TAG, "Failed to register file stream to pipeline");

  audio_element_handle_t decoder = NULL;
  const char *ext = strrchr(path, '.');
  if (ext != NULL && strcasecmp(ext, ".opus") == 0)
  {
    opus_decoder_cfg_t opus_cfg = DEFAULT_OPUS_DECODER_CONFIG();
    opus_cfg.task_core = 0;
    decoder = decoder_opus_init(&opus_cfg);
    ESP_GOTO_ON_FALSE(decoder, ESP_FAIL, cleanup, RADIO_TAG, "Failed to create Opus decoder");
  }
  else
  {
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    decoder = wav_decoder_init(&wav_cfg);
    ESP_GOTO_ON_FALSE(decoder, ESP_FAIL, cleanup, RADIO_TAG, "Failed to create WAV decoder");
  }
  ESP_GOTO_ON_ERROR(audio_pipeline_register(pipeline, decoder, "decoder"), cleanup, RADIO_TAG, "Failed to register decoder to pipeline");
  ESP_GOTO_ON_ERROR(audio_element_set_output_ringbuf(decoder, output), cleanup, RADIO_TAG, "Failed to set output ringbuf for decoder");

  ESP_GOTO_ON_ERROR(audio_pipeline_link(pipeline, (const char *[]){"file", "decoder"}, 2), cleanup, RADIO_TAG, "Failed to link file and decoder to pipeline");

  audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
  evt = audio_event_iface_init(&evt_cfg);
  ESP_GOTO_ON_FALSE(evt, ESP_FAIL, cleanup, RADIO_TAG, "Failed to create event interface");
  ESP_GOTO_ON_ERROR(audio_pipeline_set_listener(pipeline, evt), cleanup, RADIO_TAG, "Failed to set listener for pipeline");

  ESP_GOTO_ON_ERROR(audio_pipeline_run(pipeline), cleanup, RADIO_TAG, "Failed to start audio pipeline");

  while (1)
  {
    audio_event_iface_msg_t msg;
    esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
    if (ret != ESP_OK)
    {
      ESP_LOGW(RADIO_TAG, "Error listening to event during playback: %d (%s)", ret, esp_err_to_name(ret));
      continue;
    }

    // We only care about the decoder element
    if (msg.source_type != AUDIO_ELEMENT_TYPE_ELEMENT || msg.source != (void *)decoder)
    {
      continue;
    }

    // Don't plug into mixer until we get a REPORT_MUSIC_INFO event, since that
    // tells us the audio format
    if (msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO)
    {
      audio_element_info_t info;
      ESP_GOTO_ON_ERROR(audio_element_getinfo(decoder, &info), cleanup, RADIO_TAG, "Failed to get decoder info");
      ESP_LOGD(RADIO_TAG, "Playback audio format (%s): %d Hz, %d bits, %s", path, info.sample_rates, info.bits, info.channels == 1 ? "mono" : "stereo");

      if (!channel)
      {
        ESP_GOTO_ON_ERROR(mixer_play_audio((mixer_read_callback_t)rb_read, output, info.sample_rates, info.bits, info.channels, duck_others, &channel), cleanup, RADIO_TAG, "Failed to add audio to mixer");
      }

      continue;
    }

    if (msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
        ((int)msg.data == AEL_STATUS_STATE_FINISHED || (int)msg.data == AEL_STATUS_STATE_STOPPED))
    {
      break;
    }
  }

cleanup:
  if (channel)
  {
    mixer_stop_audio(channel);
  }

  if (pipeline)
  {
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    // This will deinit all registered elements, too
    audio_pipeline_deinit(pipeline);
  }

  if (output)
  {
    rb_destroy(output);
  }

  if (evt)
  {
    audio_event_iface_destroy(evt);
  }

  if (fd >= 0)
  {
    close(fd);
  }

  return ret;
}
