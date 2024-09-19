#pragma once

#include "esp_err.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  esp_err_t storage_init(void);
  esp_err_t storage_mount(void);

#ifdef __cplusplus
}
#endif
