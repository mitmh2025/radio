#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*file_cache_refresh_cb)(void *ctx);

esp_err_t file_cache_init(void);
int file_cache_open_file(const char *name);
char *file_cache_get_hash(const char *name);
esp_err_t file_cache_subscribe_refresh(file_cache_refresh_cb cb, void *ctx);

#ifdef __cplusplus
}
#endif
