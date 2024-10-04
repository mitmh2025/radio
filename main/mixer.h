#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mixer_channel *mixer_channel_t;
typedef int (*mixer_read_callback_t)(void *ctx, char *data, int len,
                                     TickType_t ticks_to_wait);

esp_err_t mixer_init();
esp_err_t mixer_play_audio(mixer_read_callback_t callback, void *ctx,
                           int sample_rate, int bits, int channel,
                           bool duck_others, mixer_channel_t *slot);
esp_err_t mixer_stop_audio(mixer_channel_t slot);

#ifdef __cplusplus
}
#endif
