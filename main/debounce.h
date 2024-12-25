#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*debounce_isr_t)(void *args, bool state);

esp_err_t debounce_init();
esp_err_t debounce_handler_add(gpio_num_t gpio_num, gpio_int_type_t intr_type,
                               debounce_isr_t isr_handler, void *args,
                               TickType_t timeout);
esp_err_t debounce_handler_remove(gpio_num_t gpio_num);

#ifdef __cplusplus
}
#endif
