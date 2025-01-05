#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t station_2pi_init();
esp_err_t station_2pi_need_wifi_announcement(bool need_announcement);

#ifdef __cplusplus
}
#endif
