#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  esp_err_t playback_file(const char *path, bool duck_others);

#ifdef __cplusplus
}
#endif
