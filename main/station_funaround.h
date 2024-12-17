#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t station_funaround_init();
esp_err_t station_funaround_set_ratchet(uint16_t new_ratchet);

#ifdef __cplusplus
}
#endif
