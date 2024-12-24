#pragma once

#include "playback.h"

#include "esp_err.h"

typedef struct {
  char path[128];
  bool duck_others;
  bool tuned;
  int skip_samples;
} playback_queue_entry_t;

typedef void (*playback_queue_empty_cb_t)();

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t playback_queue_init();
esp_err_t playback_queue_add(playback_queue_entry_t *cfg);
esp_err_t playback_queue_subscribe_empty(playback_queue_empty_cb_t cb);
esp_err_t playback_queue_unsubscribe_empty(playback_queue_empty_cb_t cb);
esp_err_t playback_queue_skip();
esp_err_t playback_queue_drain();
esp_err_t playback_queue_pause();
esp_err_t playback_queue_resume();
esp_err_t playback_queue_pause_toggle();
bool playback_queue_active();

#ifdef __cplusplus
}
#endif
