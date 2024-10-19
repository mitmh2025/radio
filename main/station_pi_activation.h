#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t station_pi_activation_init();
esp_err_t station_pi_activation_enable(bool enable);

#ifdef __cplusplus
}
#endif
