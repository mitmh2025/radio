#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t volume_min;
  uint32_t volume_max;
  uint32_t frequency_min;
  uint32_t frequency_max;
} radio_calibration_t;

esp_err_t calibration_load(radio_calibration_t *calibration);
esp_err_t calibration_calibrate(radio_calibration_t *calibration);
esp_err_t calibration_erase();

#ifdef __cplusplus
}
#endif
