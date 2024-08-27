#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  esp_err_t wifi_get_mac(uint8_t *mac);
  esp_err_t wifi_init();

#ifdef __cplusplus
}
#endif
