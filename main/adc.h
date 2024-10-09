#pragma once

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t adc_init();
esp_err_t adc_subscribe(adc_digi_pattern_config_t *config,
                        void (*callback)(void *user_data,
                                         adc_digi_output_data_t *result),
                        void *user_data);
esp_err_t adc_unsubscribe(uint8_t channel);

#ifdef __cplusplus
}
#endif
