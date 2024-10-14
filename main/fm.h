#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t fm_channel_to_frequency(uint16_t channel, long long *frequency);
esp_err_t fm_frequency_to_channel(long long frequency, uint16_t *channel);
esp_err_t fm_init(void);
esp_err_t fm_enable(void);
esp_err_t fm_disable(void);
esp_err_t fm_tune(uint16_t channel);

#ifdef __cplusplus
}
#endif
