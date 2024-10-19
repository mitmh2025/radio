#include "extreme.h"
#include "main.h"

#include "esp_check.h"
#include "esp_log.h"

struct extreme_handle {
  extreme_mode_t mode;
  uint32_t (*comparator)(uint32_t, uint32_t);

  size_t buckets;
  int64_t interval;

  int64_t start;
  int64_t last_update;

  uint32_t cached_extreme;
  uint32_t values[];
};

static inline uint32_t default_extreme(extreme_mode_t mode) {
  return mode == EXTREME_MODE_MIN ? UINT32_MAX : 0;
}

static inline uint32_t extreme_min(uint32_t a, uint32_t b) {
  return a < b ? a : b;
}

static inline uint32_t extreme_max(uint32_t a, uint32_t b) {
  return a > b ? a : b;
}

esp_err_t extreme_init(const extreme_config_t *config,
                       extreme_handle_t *return_handle) {
  ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "config is NULL");
  ESP_RETURN_ON_FALSE(config->buckets > 0, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "buckets must be greater than 0");
  ESP_RETURN_ON_FALSE(config->interval > 0, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "interval must be greater than 0");
  ESP_RETURN_ON_FALSE(return_handle != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "return_handle is NULL");

  extreme_handle_t handle = calloc(1, sizeof(struct extreme_handle) +
                                          config->buckets * sizeof(uint32_t));
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate handle");

  handle->mode = config->mode;
  handle->comparator =
      config->mode == EXTREME_MODE_MIN ? extreme_min : extreme_max;
  handle->buckets = config->buckets;
  handle->interval = config->interval;
  handle->start = esp_timer_get_time();
  handle->last_update = handle->start;

  handle->cached_extreme = default_extreme(handle->mode);
  for (size_t i = 0; i < handle->buckets; i++) {
    handle->values[i] = handle->cached_extreme;
  }

  *return_handle = handle;
  return ESP_OK;
}

void extreme_free(extreme_handle_t handle) { free(handle); }

esp_err_t extreme_update(extreme_handle_t handle, uint32_t value, int64_t now) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "handle is NULL");

  int64_t elapsed = now - handle->start;

  size_t bucket = (elapsed / handle->interval);
  size_t last_bucket = (handle->last_update / handle->interval);

  if (bucket - last_bucket >= handle->buckets) {
    // We've fully wrapped around, reset everything
    handle->cached_extreme = default_extreme(handle->mode);
    for (size_t i = 0; i < handle->buckets; i++) {
      handle->values[i] = handle->cached_extreme;
    }
    last_bucket = bucket;
  }

  last_bucket %= handle->buckets;
  bucket %= handle->buckets;

  if (bucket != last_bucket) {
    for (size_t i = (last_bucket + 1) % handle->buckets; i != bucket;
         i = (i + 1) % handle->buckets) {
      handle->values[i] = default_extreme(handle->mode);
    }

    handle->cached_extreme = default_extreme(handle->mode);
    for (size_t i = 0; i < handle->buckets; i++) {
      handle->cached_extreme =
          handle->comparator(handle->cached_extreme, handle->values[i]);
    }
  }

  handle->values[bucket] = handle->comparator(handle->values[bucket], value);
  handle->cached_extreme =
      handle->comparator(handle->cached_extreme, handle->values[bucket]);

  handle->last_update = now;

  return ESP_OK;
}

uint32_t extreme_get(extreme_handle_t handle) { return handle->cached_extreme; }