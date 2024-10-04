#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_init();
esp_err_t led_set_pixel(uint32_t index, uint32_t red, uint32_t green,
                        uint32_t blue);

#ifdef __cplusplus
}
#endif
