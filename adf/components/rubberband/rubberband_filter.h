#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "audio_element.h"

typedef struct {
    int bits;
    int sample_rate;
    int channels;
    int out_rb_size;           /*!< Output ringbuffer size*/
    int task_stack;            /*!< Task stack size */
    int task_core;             /*!< Task running on core */
    int task_prio;             /*!< Task priority */
    bool stack_in_ext;         /*!< Try to allocate stack in external memory */
} rubberband_filter_cfg_t;

#define RUBBERBAND_FILTER_TASK_STACK               (4 * 1024)
#define RUBBERBAND_FILTER_TASK_CORE                (0)
#define RUBBERBAND_FILTER_TASK_PRIO                (5)
#define RUBBERBAND_FILTER_RINGBUFFER_SIZE          (2 * 1024)

#define DEFAULT_RUBBERBAND_FILTER_CONFIG() { \
    .sample_rate = 48000, \
    .channels = 2, \
    .bits = 16, \
    .out_rb_size = RUBBERBAND_FILTER_RINGBUFFER_SIZE, \
    .task_stack = RUBBERBAND_FILTER_TASK_STACK, \
    .task_core = RUBBERBAND_FILTER_TASK_CORE, \
    .task_prio = RUBBERBAND_FILTER_TASK_PRIO, \
    .stack_in_ext = true, \
}

audio_element_handle_t rubberband_filter_init(rubberband_filter_cfg_t *config);
esp_err_t rubberband_filter_change_src_info(audio_element_handle_t self, int sample_rates, int channels, int bits);

#ifdef __cplusplus
}
#endif
