#pragma once

#include "playback.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*playback_queue_empty_cb_t)();

esp_err_t playback_queue_init();
esp_err_t playback_queue_add(playback_cfg_t *cfg);
esp_err_t playback_queue_subscribe_empty(playback_queue_empty_cb_t cb);
esp_err_t playback_queue_unsubscribe_empty(playback_queue_empty_cb_t cb);
esp_err_t playback_queue_skip();
esp_err_t playback_queue_drain();
bool playback_queue_active();

#ifdef __cplusplus
}
#endif
