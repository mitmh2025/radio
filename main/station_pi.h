#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t station_pi_init();
esp_err_t station_pi_enable();
esp_err_t station_pi_set_stage(uint8_t stage);

#ifdef __cplusplus
}
#endif
