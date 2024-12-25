#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_get_mac(uint8_t *mac);
esp_err_t wifi_get_ap_network_mac(uint8_t *mac);
esp_err_t wifi_init();
esp_err_t wifi_force_reconnect();
esp_err_t wifi_test_connection(const char *ssid, const char *password);
esp_err_t wifi_set_alt_network(const char *ssid, const char *password);
const char *wifi_get_alt_ssid();
esp_err_t wifi_force_scan();
esp_err_t wifi_enable_ap();
esp_err_t wifi_disable_ap();

#ifdef __cplusplus
}
#endif
