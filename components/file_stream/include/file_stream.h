#pragma once

#include "audio_element.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FILE_STREAM_BUF_SIZE (4096)
#define FILE_STREAM_TASK_STACK (8192)
#define FILE_STREAM_TASK_CORE (0)
#define FILE_STREAM_TASK_PRIO (4)
#define FILE_STREAM_RINGBUFFER_SIZE (32 * 1024)

typedef struct {
  int fd;          /*!< File descriptor */
  int buf_sz;      /*!< Audio Element Buffer size */
  int out_rb_size; /*!< Size of output ringbuffer */
  int task_stack;  /*!< Task stack size */
  int task_core;   /*!< Task running in core (0 or 1) */
  int task_prio;   /*!< Task priority (based on freeRTOS priority) */
} file_stream_cfg_t;

#define FILE_STREAM_CFG_DEFAULT()                                              \
  {                                                                            \
      .buf_sz = FILE_STREAM_BUF_SIZE,                                          \
      .out_rb_size = FILE_STREAM_RINGBUFFER_SIZE,                              \
      .task_stack = FILE_STREAM_TASK_STACK,                                    \
      .task_core = FILE_STREAM_TASK_CORE,                                      \
      .task_prio = FILE_STREAM_TASK_PRIO,                                      \
  }

audio_element_handle_t file_stream_init(file_stream_cfg_t *config);

#ifdef __cplusplus
}
#endif
