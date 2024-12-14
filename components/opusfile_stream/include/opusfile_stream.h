#pragma once

#include "audio_element.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPUSFILE_STREAM_BUF_SIZE (11520)
#define OPUSFILE_STREAM_TASK_STACK (30 * 1024)
#define OPUSFILE_STREAM_TASK_CORE (0)
#define OPUSFILE_STREAM_TASK_PRIO (11)
#define OPUSFILE_STREAM_RINGBUFFER_SIZE                                        \
  (8 * 60 * 1024) // roughly 60 seconds of audio at 8kbps

typedef struct {
  int fd;          /*!< File descriptor */
  int buf_sz;      /*!< Audio Element Buffer size */
  int out_rb_size; /*!< Size of output ringbuffer */
  int task_stack;  /*!< Task stack size */
  int task_core;   /*!< Task running in core (0 or 1) */
  int task_prio;   /*!< Task priority (based on freeRTOS priority) */
} opusfile_stream_cfg_t;

#define OPUSFILE_STREAM_CFG_DEFAULT()                                          \
  {                                                                            \
      .buf_sz = OPUSFILE_STREAM_BUF_SIZE,                                      \
      .out_rb_size = OPUSFILE_STREAM_RINGBUFFER_SIZE,                          \
      .task_stack = OPUSFILE_STREAM_TASK_STACK,                                \
      .task_core = OPUSFILE_STREAM_TASK_CORE,                                  \
      .task_prio = OPUSFILE_STREAM_TASK_PRIO,                                  \
  }

audio_element_handle_t opusfile_stream_init(opusfile_stream_cfg_t *config);

#ifdef __cplusplus
}
#endif
