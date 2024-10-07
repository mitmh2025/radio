#pragma once

#include "esp_err.h"

#include <stdbool.h>

#define STORAGE_MOUNTPOINT "/data"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t storage_init(void);

#ifdef __cplusplus
}
#endif
