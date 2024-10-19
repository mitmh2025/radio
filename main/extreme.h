#pragma once

#include "esp_err.h"

typedef enum {
  EXTREME_MODE_MIN = 0,
  EXTREME_MODE_MAX = 1,
} extreme_mode_t;

typedef struct {
  extreme_mode_t mode;
  size_t buckets;
  int64_t interval;
} extreme_config_t;

typedef struct extreme_handle *extreme_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t extreme_init(const extreme_config_t *config,
                       extreme_handle_t *return_handle);
void extreme_free(extreme_handle_t handle);
esp_err_t extreme_update(extreme_handle_t handle, uint32_t value, int64_t now);
uint32_t extreme_get(extreme_handle_t handle);

#ifdef __cplusplus
}
#endif
