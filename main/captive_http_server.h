#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t captive_http_server_init();
esp_err_t captive_http_server_start();
esp_err_t captive_http_server_stop();

#ifdef __cplusplus
}
#endif
