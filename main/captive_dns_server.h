#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t captive_dns_server_start();
esp_err_t captive_dns_server_stop();

#ifdef __cplusplus
}
#endif
