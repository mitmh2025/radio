#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t file_cache_init(void);
int file_cache_open_file(const char *name);

#ifdef __cplusplus
}
#endif
