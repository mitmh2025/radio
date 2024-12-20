#include "opusfile_stream.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "opusfile.h"

#include "audio_error.h"
#include "audio_mem.h"
#include "esp_log.h"

static const char *TAG = "OPUSFILE_STREAM";

typedef struct opusfile_stream {
  int fd;
  OggOpusFile *opus_file;
  OpusDecoder *opus_decoder;
} opusfile_stream_t;

static int _opusfile_decode(void *ctx, OpusMSDecoder *decoder, void *pcm,
                            const ogg_packet *op, int nsamples, int nchannels,
                            int format, int li) {
  opusfile_stream_t *file = (opusfile_stream_t *)ctx;
  if (file == NULL) {
    ESP_LOGE(TAG, "Invalid context");
    return OP_DEC_USE_DEFAULT;
  }

  if (file->opus_decoder == NULL) {
    ESP_LOGE(TAG, "Opus decoder is not initialized");
    return OP_DEC_USE_DEFAULT;
  }

  int ret =
      opus_decode(file->opus_decoder, op->packet, op->bytes, pcm, nsamples, 0);
  if (ret < 0) {
    ESP_LOGE(TAG, "opus_decode failed: %d", ret);
    return ret;
  }

  return 0;
}

static esp_err_t _opusfile_open(audio_element_handle_t self) {
  opusfile_stream_t *file = (opusfile_stream_t *)audio_element_getdata(self);
  audio_element_info_t info;
  audio_element_getinfo(self, &info);
  ESP_LOGD(TAG, "_opusfile_open");

  if (file->opus_decoder == NULL) {
    int err;
    file->opus_decoder = opus_decoder_create(48000, 1, &err);
    if (err != OPUS_OK) {
      ESP_LOGE(TAG, "Failed to create Opus decoder: %d", err);
      return ESP_FAIL;
    }
  }

  if (file->opus_file == NULL) {
    OpusFileCallbacks cb = {};
    void *opus_stream = op_fdopen(&cb, file->fd, "rb");
    if (opus_stream == NULL) {
      ESP_LOGE(TAG, "Failed to open Opus stream: %d", errno);
      return ESP_FAIL;
    }

    int err;
    file->opus_file = op_open_callbacks(opus_stream, &cb, NULL, 0, &err);
    if (file->opus_file == NULL) {
      ESP_LOGE(TAG, "Failed to open Opus file: %d", err);
      return ESP_FAIL;
    }

    op_set_decode_callback(file->opus_file, _opusfile_decode, file);

    file->fd = -1;
  }

  opus_int64 size = op_pcm_total(file->opus_file, -1);
  if (size < 0) {
    ESP_LOGE(TAG, "Failed to get total size: %d", (int)size);
    return ESP_FAIL;
  }

  info.total_bytes = size;
  ESP_LOGI(TAG, "File size is %d byte, pos:%d", (int)size, (int)info.byte_pos);
  if (info.byte_pos > 0) {
    int ret = op_pcm_seek(file->opus_file, info.byte_pos);
    if (ret < 0) {
      ESP_LOGE(TAG, "op_pcm_seek failed: %d", ret);
      return ESP_FAIL;
    }
  }

  esp_err_t err = audio_element_set_total_bytes(self, info.total_bytes);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set total bytes: %d", err);
    return err;
  }

  // Our audio files are always 48kHz, 16-bit, mono
  audio_element_set_music_info(self, 48000, 1, 16);
  audio_element_report_info(self);

  return ESP_OK;
}

static esp_err_t _opusfile_destroy(audio_element_handle_t self) {
  opusfile_stream_t *file = (opusfile_stream_t *)audio_element_getdata(self);
  if (file->opus_file != NULL) {
    op_free(file->opus_file);
  }
  if (file->opus_decoder != NULL) {
    opus_decoder_destroy(file->opus_decoder);
  }
  audio_free(file);
  return ESP_OK;
}

static int _opusfile_process(audio_element_handle_t self, char *in_buffer,
                             int in_len) {
  int r_size = audio_element_input(self, in_buffer, in_len);
  int w_size = 0;
  if (r_size > 0) {
    w_size = audio_element_output(self, in_buffer, r_size);
  } else {
    w_size = r_size;
  }
  return w_size;
}

static int _opusfile_read(audio_element_handle_t self, char *buffer, int len,
                          TickType_t ticks_to_wait, void *context) {
  opusfile_stream_t *file = (opusfile_stream_t *)audio_element_getdata(self);
  audio_element_info_t info;
  audio_element_getinfo(self, &info);

  ESP_LOGD(TAG, "read len=%d, pos=%d/%d", len, (int)info.byte_pos,
           (int)info.total_bytes);

  int rlen = op_read(file->opus_file, (opus_int16 *)buffer, len, NULL);
  if (rlen == 0) {
    ESP_LOGD(TAG, "No more data, ret:%d", rlen);
    return AEL_IO_DONE;
  } else if (rlen < 0) {
    ESP_LOGE(TAG, "read failed: %d", rlen);
  } else {
    audio_element_update_byte_pos(self, rlen);
  }

  return rlen * 2;
}

audio_element_handle_t opusfile_stream_init(opusfile_stream_cfg_t *config) {
  opusfile_stream_t *file = audio_calloc(1, sizeof(opusfile_stream_t));
  AUDIO_MEM_CHECK(TAG, file, return NULL);
  file->fd = config->fd;

  audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
  cfg.open = _opusfile_open;
  cfg.destroy = _opusfile_destroy;
  cfg.process = _opusfile_process;
  cfg.read = _opusfile_read;
  cfg.task_stack = config->task_stack;
  cfg.task_prio = config->task_prio;
  cfg.task_core = config->task_core;
  cfg.out_rb_size = config->out_rb_size;
  cfg.buffer_len = config->buf_sz;
  cfg.stack_in_ext = true;
  if (cfg.buffer_len == 0) {
    cfg.buffer_len = OPUSFILE_STREAM_BUF_SIZE;
  }
  cfg.tag = TAG;

  audio_element_handle_t el = audio_element_init(&cfg);
  AUDIO_MEM_CHECK(TAG, el, goto _opusfile_init_exit);
  audio_element_setdata(el, file);

  return el;
_opusfile_init_exit:
  audio_free(file);
  return NULL;
}