#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const char *path;
  bool duck_others;
  bool tuned;
  int skip_samples;
} playback_cfg_t;

typedef struct playback_handle *playback_handle_t;

esp_err_t playback_file(const playback_cfg_t *cfg, playback_handle_t *handle);
esp_err_t playback_wait_for_completion(playback_handle_t handle);
esp_err_t playback_detune(playback_handle_t handle);
esp_err_t playback_entune(playback_handle_t handle);
esp_err_t playback_stop(playback_handle_t handle);
void playback_free(playback_handle_t handle);

// Must be called from a task with a large enough stack for Opus decoding (e.g.
// at least 20KB or so)
esp_err_t playback_duration(const char *path, int64_t *duration);

#ifdef __cplusplus
}
#endif
