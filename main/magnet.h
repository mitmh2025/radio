#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t magnet_init(void);
esp_err_t magnet_read(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif
