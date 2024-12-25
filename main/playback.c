#include "playback.h"
#include "file_cache.h"
#include "main.h"
#include "mixer.h"

#include "file_stream.h"
#include "opusfile_stream.h"

#include "opusfile.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "esp_check.h"

#include "audio_pipeline.h"
#include "opus_decoder.h"
#include "wav_decoder.h"

#define PLAYBACK_PAUSE_TOGGLE (INT_MAX)

struct playback_handle {
  char *path;
  bool duck_others;
  int fd;
  audio_pipeline_handle_t pipeline;
  audio_element_handle_t decoder;
  audio_event_iface_handle_t evt;
  ringbuf_handle_t output;
  bool tuned;
  audio_element_info_t info;
  mixer_channel_t channel;
};

esp_err_t playback_file(const playback_cfg_t *cfg,
                        playback_handle_t *return_handle) {
  ESP_RETURN_ON_FALSE(return_handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid handle");

  esp_err_t ret = ESP_OK;

  playback_handle_t handle = calloc(1, sizeof(struct playback_handle));
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate memory for playback handle");
  handle->path = strdup(cfg->path);
  handle->duck_others = cfg->duck_others;
  handle->tuned = cfg->tuned;

  handle->fd = file_cache_open_file(handle->path, NULL);
  ESP_RETURN_ON_FALSE(handle->fd >= 0, ESP_FAIL, RADIO_TAG,
                      "Failed to open file \"%s\": %d", handle->path, errno);

  audio_pipeline_cfg_t pipeline_cfg = {};
  handle->pipeline = audio_pipeline_init(&pipeline_cfg);
  ESP_RETURN_ON_FALSE(handle->pipeline, ESP_FAIL, RADIO_TAG,
                      "Failed to create audio pipeline");

  // Buffer 2 seconds of audio
  handle->output = rb_create(960 * 2 * 100, 1);
  ESP_GOTO_ON_ERROR(handle->output == NULL, cleanup, RADIO_TAG,
                    "Failed to create output ringbuf");

  const char *ext = strchr(handle->path, '.');
  if (ext != NULL && strcasecmp(ext, ".opus") == 0) {
    opusfile_stream_cfg_t opusfile_cfg = OPUSFILE_STREAM_CFG_DEFAULT();
    opusfile_cfg.fd = handle->fd;
    handle->decoder = opusfile_stream_init(&opusfile_cfg);
    ESP_GOTO_ON_FALSE(handle->decoder, ESP_FAIL, cleanup, RADIO_TAG,
                      "Failed to create Opus decoder");

    if (cfg->skip_samples > 0) {
      ESP_GOTO_ON_ERROR(
          audio_element_set_byte_pos(handle->decoder, cfg->skip_samples),
          cleanup, RADIO_TAG, "Failed to set skip samples for decoder");
    }

    ESP_GOTO_ON_ERROR(
        audio_pipeline_register(handle->pipeline, handle->decoder, "decoder"),
        cleanup, RADIO_TAG, "Failed to register decoder to pipeline");
    ESP_GOTO_ON_ERROR(
        audio_element_set_output_ringbuf(handle->decoder, handle->output),
        cleanup, RADIO_TAG, "Failed to set output ringbuf for decoder");

    ESP_GOTO_ON_ERROR(
        audio_pipeline_link(handle->pipeline, (const char *[]){"decoder"}, 1),
        cleanup, RADIO_TAG, "Failed to link decoder to pipeline");  
  } else {
    ESP_GOTO_ON_FALSE(cfg->skip_samples == 0, ESP_ERR_INVALID_ARG, cleanup,
                      RADIO_TAG, "skip_samples not supported for WAV file");

    file_stream_cfg_t file_cfg = FILE_STREAM_CFG_DEFAULT();
    file_cfg.fd = handle->fd;
    audio_element_handle_t file_stream = file_stream_init(&file_cfg);
    ESP_GOTO_ON_FALSE(file_stream, ESP_FAIL, cleanup, RADIO_TAG,
                      "Failed to create file stream");
    ESP_GOTO_ON_ERROR(
        audio_pipeline_register(handle->pipeline, file_stream, "file"), cleanup,
        RADIO_TAG, "Failed to register file stream to pipeline");

    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    handle->decoder = wav_decoder_init(&wav_cfg);
    ESP_GOTO_ON_FALSE(handle->decoder, ESP_FAIL, cleanup, RADIO_TAG,
                      "Failed to create WAV decoder");
    ESP_GOTO_ON_ERROR(
        audio_pipeline_register(handle->pipeline, handle->decoder, "decoder"),
        cleanup, RADIO_TAG, "Failed to register decoder to pipeline");
    ESP_GOTO_ON_ERROR(
        audio_element_set_output_ringbuf(handle->decoder, handle->output),
        cleanup, RADIO_TAG, "Failed to set output ringbuf for decoder");

    ESP_GOTO_ON_ERROR(
        audio_pipeline_link(handle->pipeline,
                            (const char *[]){"file", "decoder"}, 2),
        cleanup, RADIO_TAG, "Failed to link file and decoder to pipeline");
  }

  audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
  handle->evt = audio_event_iface_init(&evt_cfg);
  ESP_GOTO_ON_FALSE(handle->evt, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to create event interface");
  ESP_GOTO_ON_ERROR(audio_pipeline_set_listener(handle->pipeline, handle->evt),
                    cleanup, RADIO_TAG, "Failed to set listener for pipeline");
  // Need the event interface to listen to itself (i.e. plug its external queue
  // into its queue set)
  ESP_GOTO_ON_ERROR(audio_event_iface_set_listener(handle->evt, handle->evt),
                    cleanup, RADIO_TAG,
                    "Failed to set listener for event interface");

  ESP_GOTO_ON_ERROR(audio_pipeline_run(handle->pipeline), cleanup, RADIO_TAG,
                    "Failed to start audio pipeline");

cleanup:
  if (ret != ESP_OK) {
    playback_free(handle);
    handle = NULL;
  }

  *return_handle = handle;
  return ret;
}

esp_err_t playback_wait_for_completion(playback_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid playback handle");

  bool finished = false;

  while (1) {
    audio_event_iface_msg_t msg;
    esp_err_t ret = audio_event_iface_listen(
        handle->evt, &msg,
        finished && handle->tuned ? pdMS_TO_TICKS(20) : portMAX_DELAY);
    if (ret == ESP_OK) {
      if (msg.source_type == AUDIO_ELEMENT_TYPE_PLAYER &&
          msg.source == (void *)handle) {
        if (msg.cmd == AEL_MSG_CMD_PAUSE) {
          handle->tuned = false;
        } else if (msg.cmd == AEL_MSG_CMD_RESUME) {
          handle->tuned = true;
        } else if (msg.cmd == AEL_MSG_CMD_STOP) {
          ESP_LOGD(RADIO_TAG, "Playback interrupted");
          break;
        } else if (msg.cmd == PLAYBACK_PAUSE_TOGGLE) {
          handle->tuned = !handle->tuned;
        }
      }

      if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
          msg.source == (void *)handle->decoder) {
        // Don't plug into mixer until we get a REPORT_MUSIC_INFO event, since
        // that tells us the audio format
        if (msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
          ESP_RETURN_ON_ERROR(
              audio_element_getinfo(handle->decoder, &handle->info), RADIO_TAG,
              "Failed to get decoder info");
          ESP_LOGD(RADIO_TAG, "Playback audio format (%s): %d Hz, %d bits, %s",
                   handle->path, handle->info.sample_rates, handle->info.bits,
                   handle->info.channels == 1 ? "mono" : "stereo");
        }

        if (msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
            ((int)msg.data == AEL_STATUS_STATE_FINISHED ||
             (int)msg.data == AEL_STATUS_STATE_STOPPED)) {
          finished = true;
        }
      }
    }

    bool currently_playing = handle->channel;
    bool should_play = handle->tuned && handle->info.sample_rates;
    if (currently_playing && !should_play) {
      mixer_stop_audio(handle->channel);
      handle->channel = NULL;
    } else if (!currently_playing && should_play) {
      ESP_RETURN_ON_ERROR(
          mixer_play_audio((mixer_read_callback_t)rb_read, handle->output,
                           handle->info.sample_rates, handle->info.bits,
                           handle->info.channels, handle->duck_others,
                           &handle->channel),
          RADIO_TAG, "Failed to add audio to mixer");
    }

    if (finished && rb_bytes_filled(handle->output) <
                        960 * handle->info.channels * handle->info.bits / 8) {
      ESP_LOGD(RADIO_TAG, "Playback finished");
      break;
    }
  }

  if (handle->channel) {
    mixer_stop_audio(handle->channel);
    handle->channel = NULL;
  }

  return ESP_OK;
}

esp_err_t playback_pause(playback_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid playback handle");
  ESP_RETURN_ON_ERROR(
      audio_event_iface_sendout(handle->evt,
                                &(audio_event_iface_msg_t){
                                    .source = handle,
                                    .source_type = AUDIO_ELEMENT_TYPE_PLAYER,
                                    .cmd = AEL_MSG_CMD_PAUSE,
                                }),
      RADIO_TAG, "Failed to detune playback");
  return ESP_OK;
}

esp_err_t playback_resume(playback_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid playback handle");

  ESP_RETURN_ON_ERROR(
      audio_event_iface_sendout(handle->evt,
                                &(audio_event_iface_msg_t){
                                    .source = handle,
                                    .source_type = AUDIO_ELEMENT_TYPE_PLAYER,
                                    .cmd = AEL_MSG_CMD_RESUME,
                                }),
      RADIO_TAG, "Failed to entune playback");
  return ESP_OK;
}

esp_err_t playback_pause_toggle(playback_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid playback handle");

  ESP_RETURN_ON_ERROR(
      audio_event_iface_sendout(handle->evt,
                                &(audio_event_iface_msg_t){
                                    .source = handle,
                                    .source_type = AUDIO_ELEMENT_TYPE_PLAYER,
                                    .cmd = PLAYBACK_PAUSE_TOGGLE,
                                }),
      RADIO_TAG, "Failed to entune playback");
  return ESP_OK;
}

esp_err_t playback_stop(playback_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid playback handle");

  ESP_RETURN_ON_ERROR(
      audio_event_iface_sendout(handle->evt,
                                &(audio_event_iface_msg_t){
                                    .source = handle,
                                    .source_type = AUDIO_ELEMENT_TYPE_PLAYER,
                                    .cmd = AEL_MSG_CMD_STOP,
                                }),
      RADIO_TAG, "Failed to stop playback");
  return ESP_OK;
}

void playback_free(playback_handle_t handle) {
  if (!handle) {
    return;
  }

  if (handle->channel) {
    mixer_stop_audio(handle->channel);
  }

  if (handle->pipeline) {
    audio_pipeline_stop(handle->pipeline);
    audio_pipeline_wait_for_stop(handle->pipeline);
    audio_pipeline_deinit(handle->pipeline);
  }

  if (handle->output) {
    rb_destroy(handle->output);
  }

  if (handle->evt) {
    audio_event_iface_destroy(handle->evt);
  }

  if (handle->fd >= 0) {
    close(handle->fd);
  }

  if (handle->path) {
    free(handle->path);
  }

  free(handle);
}

struct playback_duration_args {
  TaskHandle_t caller;
  int fd;
  int64_t *duration;
  esp_err_t ret;
};

static void playback_duration_task(void *ctx) {
  struct playback_duration_args *args = ctx;

  OggOpusFile *opus_file = NULL;

  OpusFileCallbacks cb = {};
  void *opus_stream = op_fdopen(&cb, args->fd, "rb");
  if (!opus_stream) {
    ESP_LOGE(RADIO_TAG, "Failed to open Opus stream: %d", errno);
    args->ret = ESP_FAIL;
    goto cleanup;
  }
  args->fd = -1;

  int err;
  opus_file = op_open_callbacks(opus_stream, &cb, NULL, 0, &err);
  if (!opus_file) {
    ESP_LOGE(RADIO_TAG, "Failed to open Opus file: %d", err);
    args->ret = ESP_FAIL;
    goto cleanup;
  }
  opus_stream = NULL;

  int64_t pcm_duration = op_pcm_total(opus_file, -1);
  if (pcm_duration < 0) {
    ESP_LOGE(RADIO_TAG, "Failed to get PCM duration: %" PRId64, pcm_duration);
    args->ret = ESP_FAIL;
    goto cleanup;
  }

  *args->duration = pcm_duration;

cleanup:
  if (opus_file != NULL) {
    op_free(opus_file);
  }

  if (opus_stream != NULL) {
    fclose(opus_stream);
  }

  xTaskNotifyGive(args->caller);
  vTaskDelete(NULL);
}

esp_err_t playback_duration(const char *path, int64_t *duration) {
  ESP_RETURN_ON_FALSE(path, ESP_ERR_INVALID_ARG, RADIO_TAG, "Invalid path");
  ESP_RETURN_ON_FALSE(duration, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid duration");

  int fd = file_cache_open_file(path, NULL);
  ESP_RETURN_ON_FALSE(fd >= 0, ESP_FAIL, RADIO_TAG, "Failed to open file: %d",
                      errno);

  struct playback_duration_args args = {
      .caller = xTaskGetCurrentTaskHandle(),
      .fd = fd,
      .duration = duration,
      .ret = ESP_OK,
  };

  BaseType_t result = xTaskCreate(playback_duration_task, "playback_duration",
                                  20 * 1024, &args, 11, NULL);

  if (result != pdPASS) {
    ESP_LOGE(RADIO_TAG, "Failed to create playback duration task: %d", result);
    args.ret = ESP_FAIL;
    goto cleanup;
  }

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

cleanup:
  if (args.fd >= 0) {
    close(args.fd);
  }

  return args.ret;
}
