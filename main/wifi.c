#include "wifi.h"
#include "../config.h"
#include "main.h"
#include "things.h"

#include <string.h>
#include <sys/time.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#ifndef RADIO_WIFI_SSID
#error "RADIO_WIFI_SSID is not defined"
#else
static_assert(
    strlen(RADIO_WIFI_SSID) > 0,
    "RADIO_WIFI_SSID is empty (make sure config.h exists and is populated)");
#endif
#ifndef RADIO_WIFI_PASSWORD
#error "RADIO_WIFI_PASSWORD is not defined"
#else
static_assert(strlen(RADIO_WIFI_PASSWORD) > 0,
              "RADIO_WIFI_PASSWORD is empty (make sure config.h exists and is "
              "populated)");
#endif

static esp_netif_t *wifi_netif = NULL;

static void wifi_clock_synced(struct timeval *tv) {
  // Format time as iso8601 string
  struct tm timeinfo;
  gmtime_r(&tv->tv_sec, &timeinfo);
  char strftime_buf[sizeof("2000-01-01T00:00:00Z") + 1];
  strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  ESP_LOGI(RADIO_TAG, "SNTP time synced to %s", strftime_buf);
}

static void wifi_report_telemetry() {
  wifi_ap_record_t ap_info;
  esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to get AP info: %d", err);
    return;
  }

  char bssid_str[18];
  snprintf(bssid_str, sizeof(bssid_str), MACSTR, MAC2STR(ap_info.bssid));

  things_send_telemetry_string("wifi_ssid", (const char *)ap_info.ssid);
  things_send_telemetry_int("wifi_rssi", ap_info.rssi);
  things_send_telemetry_int("wifi_channel", ap_info.primary);
  things_send_telemetry_string("wifi_bssid", bssid_str);
}

esp_err_t wifi_get_mac(uint8_t *mac) {
  ESP_RETURN_ON_FALSE(wifi_netif != NULL, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Wifi is not initialized");
  return esp_netif_get_mac(wifi_netif, mac);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGD(RADIO_TAG, "Wifi started");
      ESP_ERROR_CHECK(esp_wifi_connect());
      break;
    case WIFI_EVENT_STA_STOP:
      ESP_LOGI(RADIO_TAG, "Wifi stopped");
      xEventGroupClearBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
      xEventGroupSetBits(radio_event_group,
                         RADIO_EVENT_GROUP_WIFI_DISCONNECTED);
      break;
    case WIFI_EVENT_STA_CONNECTED: {
      wifi_event_sta_connected_t *event =
          (wifi_event_sta_connected_t *)event_data;
      ESP_LOGI(RADIO_TAG, "Wifi connected ssid='%.*s' channel=%u",
               event->ssid_len, event->ssid, event->channel);
      break;
    }
    case WIFI_EVENT_STA_DISCONNECTED:
      xEventGroupClearBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
      xEventGroupSetBits(radio_event_group,
                         RADIO_EVENT_GROUP_WIFI_DISCONNECTED);
      // Try again
      // TODO: handle error
      esp_wifi_connect();
      break;
    }
  } else if (event_base == IP_EVENT) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(RADIO_TAG, "Got IP address ip=" IPSTR,
               IP2STR(&event->ip_info.ip));
      esp_err_t err = esp_netif_sntp_start();
      if (err != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "Failed to start SNTP: %d", err);
      }

      xEventGroupClearBits(radio_event_group,
                           RADIO_EVENT_GROUP_WIFI_DISCONNECTED);
      xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
      break;
    }
    case IP_EVENT_STA_LOST_IP:
      xEventGroupClearBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
      xEventGroupSetBits(radio_event_group,
                         RADIO_EVENT_GROUP_WIFI_DISCONNECTED);
      break;
    }
  } else {
    ESP_LOGI(RADIO_TAG, "Unknown event base %s", event_base);
  }
}

esp_err_t wifi_init() {
  ESP_LOGD(RADIO_TAG, "Setting up wifi");

  xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_DISCONNECTED);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_err_t err = esp_wifi_init(&cfg);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize wifi: %d", err);

  // TODO: reevaluate if this is what we actually want (or if, e.g., we only
  // want it while streaming audio)
  err = esp_wifi_set_ps(WIFI_PS_NONE);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi power save mode: %d",
                      err);

  err = esp_netif_init();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize netif: %d", err);
  wifi_netif = esp_netif_create_default_wifi_sta();

  err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register wifi event handler: %d", err);
  err = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register ip event handler: %d",
                      err);

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = RADIO_WIFI_SSID,
              .password = RADIO_WIFI_PASSWORD,
              .ft_enabled = true,
          },
  };

  // TODO: AP/captive portal mode, retries, etc.
  err = esp_wifi_set_mode(WIFI_MODE_STA);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi mode: %d", err);
  err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi config: %d", err);
  err = esp_wifi_start();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to start wifi: %d", err);

  // Setup NTP
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  config.start = false;
  config.sync_cb = wifi_clock_synced;
  err = esp_netif_sntp_init(&config);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize SNTP: %d", err);

  things_register_telemetry_generator(&wifi_report_telemetry, "wifi", NULL);

  return ESP_OK;
}

esp_err_t wifi_force_reconnect() {
  // TODO: implement
  return ESP_OK;
}