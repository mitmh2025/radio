#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  esp_err_t things_init();
  esp_err_t things_provision(const char *token);
  esp_err_t things_deprovision();

#ifdef __cplusplus
}
#endif
