#pragma once

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t frequency;
  int64_t attack_time;    // in microseconds
  int64_t decay_time;     // in microseconds
  uint16_t sustain_level; // 0-0xffff
  // (no decay because we don't know how long the note will be)
} tone_generator_config_t;

typedef struct tone_generator *tone_generator_t;

esp_err_t tone_generator_init(tone_generator_config_t *config,
                              tone_generator_t *generator);
int tone_generator_play(void *ctx, char *data, int len,
                        TickType_t ticks_to_wait);
void tone_generator_free(tone_generator_t generator);

#ifdef __cplusplus
}
#endif
