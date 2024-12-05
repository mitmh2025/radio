#include "file_stream.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "audio_error.h"
#include "audio_mem.h"
#include "esp_log.h"

static const char *TAG = "FILE_STREAM";

typedef struct file_stream {
  int fd;
} file_stream_t;

static esp_err_t _file_open(audio_element_handle_t self) {
  file_stream_t *file = (file_stream_t *)audio_element_getdata(self);
  audio_element_info_t info;
  audio_element_getinfo(self, &info);
  ESP_LOGD(TAG, "_file_open %d", file->fd);
  if (file->fd == -1) {
    ESP_LOGE(TAG, "No file descriptor");
    return ESP_FAIL;
  }

  struct stat st;
  if (fstat(file->fd, &st) != 0) {
    ESP_LOGE(TAG, "fstat failed: %d (%s)", errno, strerror(errno));
    return ESP_FAIL;
  }

  info.total_bytes = st.st_size;
  ESP_LOGI(TAG, "File size is %d byte, pos:%d", (int)st.st_size,
           (int)info.byte_pos);
  if (info.byte_pos > 0) {
    if (lseek(file->fd, info.byte_pos, SEEK_SET) < 0) {
      ESP_LOGE(TAG, "lseek failed: %d (%s)", errno, strerror(errno));
      return ESP_FAIL;
    }
  }

  return audio_element_set_total_bytes(self, info.total_bytes);
}

static esp_err_t _file_destroy(audio_element_handle_t self) {
  file_stream_t *file = (file_stream_t *)audio_element_getdata(self);
  if (file->fd != -1) {
    close(file->fd);
  }
  audio_free(file);
  return ESP_OK;
}

static int _file_process(audio_element_handle_t self, char *in_buffer,
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

static int _file_read(audio_element_handle_t self, char *buffer, int len,
                      TickType_t ticks_to_wait, void *context) {
  file_stream_t *file = (file_stream_t *)audio_element_getdata(self);
  audio_element_info_t info;
  audio_element_getinfo(self, &info);

  ESP_LOGD(TAG, "read len=%d, pos=%d/%d", len, (int)info.byte_pos,
           (int)info.total_bytes);
  ringbuf_handle_t rb = audio_element_get_output_ringbuf(self);
  ESP_LOGD(TAG, "ringbuf status %d/%d", rb_bytes_filled(rb), rb_get_size(rb));

  int rlen = read(file->fd, buffer, len);
  if (rlen == 0) {
    ESP_LOGD(TAG, "No more data, ret:%d", rlen);
    return AEL_IO_DONE;
  } else if (rlen < 0) {
    ESP_LOGE(TAG, "read failed: %d (%s)", errno, strerror(errno));
  } else {
    audio_element_update_byte_pos(self, rlen);
  }
  return rlen;
}

audio_element_handle_t file_stream_init(file_stream_cfg_t *config) {
  file_stream_t *file = audio_calloc(1, sizeof(file_stream_t));
  AUDIO_MEM_CHECK(TAG, file, return NULL);
  file->fd = config->fd;

  audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
  cfg.open = _file_open;
  cfg.destroy = _file_destroy;
  cfg.process = _file_process;
  cfg.read = _file_read;
  cfg.task_stack = config->task_stack;
  cfg.task_prio = config->task_prio;
  cfg.task_core = config->task_core;
  cfg.out_rb_size = config->out_rb_size;
  cfg.buffer_len = config->buf_sz;
  if (cfg.buffer_len == 0) {
    cfg.buffer_len = FILE_STREAM_BUF_SIZE;
  }
  cfg.tag = TAG;

  audio_element_handle_t el = audio_element_init(&cfg);
  AUDIO_MEM_CHECK(TAG, el, goto _file_init_exit);
  audio_element_setdata(el, file);

  return el;
_file_init_exit:
  audio_free(file);
  return NULL;
}