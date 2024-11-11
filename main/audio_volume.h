#pragma once

#include "calibration.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t audio_volume_init(radio_calibration_t *calibration);
void audio_volume_set_floor(uint8_t floor);
void audio_volume_clear_floor();

#ifdef __cplusplus
}
#endif
