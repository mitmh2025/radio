#pragma once

#include "calibration.h"

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct frequency_handle *frequency_handle_t;

typedef struct {
  float frequency;
  bool enabled;
  void (*entune)(void *ctx);
  void (*detune)(void *ctx);
  void *ctx;
} frequency_config_t;

esp_err_t tuner_register_pm_frequency(frequency_config_t *config,
                                      frequency_handle_t *handle);
esp_err_t tuner_init(radio_calibration_t *calibration);

#ifdef __cplusplus
}
#endif
