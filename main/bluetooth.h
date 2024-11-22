#pragma once

#include "esp_err.h"

typedef enum {
  BLUETOOTH_MODE_DEFAULT,
  BLUETOOTH_MODE_DISABLED,
  BLUETOOTH_MODE_AGGRESSIVE,
  BLUETOOTH_MODE_MAX,
} bluetooth_mode_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bluetooth_init(void);
esp_err_t bluetooth_set_mode(bluetooth_mode_t mode);

#ifdef __cplusplus
}
#endif
