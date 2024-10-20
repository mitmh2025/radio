#pragma once

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool entuned;
  float frequency;
  int64_t attack_time;    // in microseconds
  int64_t decay_time;     // in microseconds
  uint16_t sustain_level; // 0-0xffff
  // In microseconds. Note that once a generator is released, it will no longer
  // repsect entune/detune calls, so release time should be fairly short
  int64_t release_time;
} tone_generator_config_t;

typedef struct tone_generator *tone_generator_t;

esp_err_t tone_generator_init(tone_generator_config_t *config,
                              tone_generator_t *generator);
void tone_generator_release(tone_generator_t generator);
void tone_generator_entune(tone_generator_t generator);
void tone_generator_detune(tone_generator_t generator);
void tone_generator_set_entuned(tone_generator_t generator, bool entuned);

#ifdef __cplusplus
}
#endif
