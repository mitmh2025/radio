#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t fm_init(void);
esp_err_t fm_enable(void);
esp_err_t fm_disable(void);
esp_err_t fm_tune(uint16_t channel);

#ifdef __cplusplus
}
#endif
