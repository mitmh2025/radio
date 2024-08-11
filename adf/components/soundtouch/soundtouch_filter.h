#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "audio_element.h"

  typedef struct
  {
    int bits;
    int sample_rate;
    int channels;
    int buffer_len;    /*!< Buffer length use for an Element */
    int out_rb_size;   /*!< Output ringbuffer size*/
    int task_stack;    /*!< Task stack size */
    int task_core;     /*!< Task running on core */
    int task_prio;     /*!< Task priority */
    bool stack_in_ext; /*!< Try to allocate stack in external memory */
  } soundtouch_filter_cfg_t;

#define SOUNDTOUCH_FILTER_TASK_STACK (4 * 1024)
#define SOUNDTOUCH_FILTER_TASK_CORE (0)
#define SOUNDTOUCH_FILTER_TASK_PRIO (5)
#define SOUNDTOUCH_FILTER_RINGBUFFER_SIZE (2 * 1024)
#define SOUNDTOUCH_FILTER_BUFFER_LEN (6720)

#define DEFAULT_SOUNDTOUCH_FILTER_CONFIG() {          \
    .sample_rate = 48000,                             \
    .channels = 2,                                    \
    .bits = 16,                                       \
    .buffer_len = SOUNDTOUCH_FILTER_BUFFER_LEN,       \
    .out_rb_size = SOUNDTOUCH_FILTER_RINGBUFFER_SIZE, \
    .task_stack = SOUNDTOUCH_FILTER_TASK_STACK,       \
    .task_core = SOUNDTOUCH_FILTER_TASK_CORE,         \
    .task_prio = SOUNDTOUCH_FILTER_TASK_PRIO,         \
    .stack_in_ext = true,                             \
}

  audio_element_handle_t soundtouch_filter_init(soundtouch_filter_cfg_t *config);
  esp_err_t soundtouch_filter_change_src_info(audio_element_handle_t self, int sample_rates, int channels, int bits);
  esp_err_t soundtouch_filter_set_tempo(audio_element_handle_t self, double tempo);

#ifdef __cplusplus
}
#endif
