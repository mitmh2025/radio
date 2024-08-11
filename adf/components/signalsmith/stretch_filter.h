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
  } stretch_filter_cfg_t;

#define STRETCH_FILTER_TASK_STACK (4 * 1024)
#define STRETCH_FILTER_TASK_CORE (0)
#define STRETCH_FILTER_TASK_PRIO (5)
#define STRETCH_FILTER_RINGBUFFER_SIZE (2 * 1024)
#define STRETCH_FILTER_BUFFER_LEN (2 * 1024)

#define DEFAULT_STRETCH_FILTER_CONFIG() {          \
    .sample_rate = 48000,                             \
    .channels = 2,                                    \
    .bits = 16,                                       \
    .buffer_len = STRETCH_FILTER_BUFFER_LEN,       \
    .out_rb_size = STRETCH_FILTER_RINGBUFFER_SIZE, \
    .task_stack = STRETCH_FILTER_TASK_STACK,       \
    .task_core = STRETCH_FILTER_TASK_CORE,         \
    .task_prio = STRETCH_FILTER_TASK_PRIO,         \
    .stack_in_ext = true,                             \
}

  audio_element_handle_t stretch_filter_init(stretch_filter_cfg_t *config);
  esp_err_t stretch_filter_change_src_info(audio_element_handle_t self, int sample_rates, int channels, int bits);
  esp_err_t stretch_filter_set_tempo(audio_element_handle_t self, double tempo);

#ifdef __cplusplus
}
#endif