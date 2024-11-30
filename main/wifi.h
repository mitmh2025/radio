#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_get_mac(uint8_t *mac);
esp_err_t wifi_init();
esp_err_t wifi_force_reconnect();
esp_err_t wifi_set_alt_network(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif
