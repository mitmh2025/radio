#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include <stdbool.h>

typedef enum {
  MIXER_STATIC_MODE_DEFAULT,
  MIXER_STATIC_MODE_COMFORT,
  MIXER_STATIC_MODE_NONE,
  MIXER_STATIC_MODE_MAX,
} mixer_static_mode_t;

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
// Configure whether the mixer should play static when there are no active audio
// sources
esp_err_t mixer_set_static(mixer_static_mode_t mode);

#ifdef __cplusplus
}
#endif
