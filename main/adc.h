#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t adc_init();
adc_oneshot_unit_handle_t adc_get_handle();

#ifdef __cplusplus
}
#endif