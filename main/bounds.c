#include "bounds.h"
#include "main.h"

#include "esp_check.h"
#include "esp_log.h"

struct bound {
  uint32_t min;
  uint32_t max;
};

static const struct bound default_bound = {
    .min = UINT32_MAX,
    .max = 0,
};

struct bounds_handle {
  size_t buckets;
  int64_t interval;

  int64_t start;
  int64_t last_update;

  struct bound cached;
  struct bound values[];
};

static inline uint32_t min(uint32_t a, uint32_t b) { return a < b ? a : b; }

static inline uint32_t max(uint32_t a, uint32_t b) { return a > b ? a : b; }

esp_err_t bounds_init(const bounds_config_t *config,
                      bounds_handle_t *return_handle) {
  ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "config is NULL");
  ESP_RETURN_ON_FALSE(config->buckets > 0, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "buckets must be greater than 0");
  ESP_RETURN_ON_FALSE(config->interval > 0, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "interval must be greater than 0");
  ESP_RETURN_ON_FALSE(return_handle != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "return_handle is NULL");

  bounds_handle_t handle = calloc(
      1, sizeof(struct bounds_handle) + config->buckets * sizeof(struct bound));
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate handle");

  handle->buckets = config->buckets;
  handle->interval = config->interval;
  handle->start = esp_timer_get_time();
  handle->last_update = handle->start;

  handle->cached = default_bound;
  for (size_t i = 0; i < handle->buckets; i++) {
    handle->values[i] = default_bound;
  }

  *return_handle = handle;
  return ESP_OK;
}

void bounds_free(bounds_handle_t handle) { free(handle); }
esp_err_t bounds_update(bounds_handle_t handle, uint32_t value) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "handle is NULL");

  int64_t now = esp_timer_get_time();

  int64_t elapsed = now - handle->start;

  size_t bucket = (elapsed / handle->interval);
  size_t last_bucket = (handle->last_update / handle->interval);

  if (bucket - last_bucket >= handle->buckets) {
    // We've fully wrapped around, reset everything
    handle->cached = default_bound;
    for (size_t i = 0; i < handle->buckets; i++) {
      handle->values[i] = default_bound;
    }
    last_bucket = bucket;
  }

  last_bucket %= handle->buckets;
  bucket %= handle->buckets;

  if (bucket != last_bucket) {
    for (size_t i = (last_bucket + 1) % handle->buckets;
         i != (bucket + 1) % handle->buckets; i = (i + 1) % handle->buckets) {
      handle->values[i] = default_bound;
    }

    handle->cached = default_bound;
    for (size_t i = 0; i < handle->buckets; i++) {
      handle->cached.min = min(handle->cached.min, handle->values[i].min);
      handle->cached.max = max(handle->cached.max, handle->values[i].max);
    }
  }

  handle->values[bucket].min = min(handle->values[bucket].min, value);
  handle->values[bucket].max = max(handle->values[bucket].max, value);
  handle->cached.min = min(handle->cached.min, value);
  handle->cached.max = max(handle->cached.max, value);

  handle->last_update = elapsed;

  return ESP_OK;
}

uint32_t bounds_get_min(bounds_handle_t handle) { return handle->cached.min; }

uint32_t bounds_get_max(bounds_handle_t handle) { return handle->cached.max; }
