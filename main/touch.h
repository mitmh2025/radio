#pragma once

#include "esp_err.h"

#include "driver/touch_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t touch_init();
esp_err_t touch_register_isr(intr_handler_t fn, void *arg);
esp_err_t touch_deregister_isr();

#ifdef __cplusplus
}
#endif
