#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bluetooth_init(void);
// interval and window are in units of 0.625ms. Pass 0 to reset to the default
esp_err_t bluetooth_set_scan_frequency(uint16_t interval, uint16_t window);

#ifdef __cplusplus
}
#endif
