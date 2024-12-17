#pragma once

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t static_init(void);
int static_read_audio(void *ctx, char *data, int len, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
