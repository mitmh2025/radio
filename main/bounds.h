#pragma once

#include "esp_err.h"

typedef struct {
  size_t buckets;
  int64_t interval;
} bounds_config_t;

typedef struct bounds_handle *bounds_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bounds_init(const bounds_config_t *config,
                      bounds_handle_t *return_handle);
void bounds_free(bounds_handle_t handle);
esp_err_t bounds_update(bounds_handle_t handle, uint32_t value);
uint32_t bounds_get_min(bounds_handle_t handle);
uint32_t bounds_get_max(bounds_handle_t handle);

#ifdef __cplusplus
}
#endif
