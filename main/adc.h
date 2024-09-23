#pragma once

#include "esp_err.h"
#include "esp_adc/adc_continuous.h"

#ifdef __cplusplus
extern "C" {
#endif

  esp_err_t adc_init();
  esp_err_t adc_subscribe(adc_digi_pattern_config_t *config, void (*callback)(adc_digi_output_data_t *result));

#ifdef __cplusplus
}
#endif
