#include "wifi.h"
#include "../config.h"
#include "captive_dns_server.h"
#include "captive_http_server.h"
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

static bool already_scanned = false;
static bool ignore_scan_results = false;

static SemaphoreHandle_t test_connection_mutex = NULL;
static TaskHandle_t test_connection_task = NULL;

static nvs_handle_t wifi_nvs = 0;
static char *wifi_alt_ssid = NULL;
static char *wifi_alt_password = NULL;

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
  return esp_read_mac(mac, ESP_MAC_WIFI_STA);
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

      xSemaphoreTake(test_connection_mutex, portMAX_DELAY);
      if (test_connection_task) {
        ESP_LOGI(RADIO_TAG, "Successfully made test connection to network");
        xTaskNotify(test_connection_task, 1, eSetValueWithOverwrite);
        test_connection_task = NULL;
      }
      xSemaphoreGive(test_connection_mutex);
      break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
      xEventGroupClearBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
      xEventGroupSetBits(radio_event_group,
                         RADIO_EVENT_GROUP_WIFI_DISCONNECTED);

      bool skip_scan = false;

      xSemaphoreTake(test_connection_mutex, portMAX_DELAY);
      if (test_connection_task) {
        ESP_LOGI(RADIO_TAG,
                 "Disconnected from (or failed to connect to) network");
        xTaskNotify(test_connection_task, 0, eSetValueWithOverwrite);
        test_connection_task = NULL;
        skip_scan = true;
      }
      xSemaphoreGive(test_connection_mutex);

      if (already_scanned) {
        // TODO: handle failure
        skip_scan = true;
      }

      if (skip_scan) {
        break;
      }

      // We couldn't connect to the network we wanted to connect to, so try
      // scanning
      ESP_LOGI(RADIO_TAG,
               "Unabled to connect to previously configured network, scanning");
      esp_wifi_scan_start(NULL, false);
      already_scanned = true;
      break;
    }

    case WIFI_EVENT_SCAN_DONE: {
      if (ignore_scan_results) {
        ignore_scan_results = false;
        break;
      }

      int8_t strongest_rssi = INT8_MIN;
      char strongest_ssid[33] = {0};
      uint8_t strongest_channel = 0;

      wifi_ap_record_t ap_info;
      while (true) {
        esp_err_t err = esp_wifi_scan_get_ap_record(&ap_info);
        if (err == ESP_FAIL) {
          break;
        }
        if (err != ESP_OK) {
          ESP_LOGE(RADIO_TAG, "Failed to get AP record: %d", err);
          break;
        }

        if (strcmp((const char *)ap_info.ssid, RADIO_WIFI_SSID) != 0 &&
            (wifi_alt_ssid == NULL ||
             strcmp((const char *)ap_info.ssid, wifi_alt_ssid) != 0)) {
          continue;
        }

        if (ap_info.rssi > strongest_rssi) {
          strongest_rssi = ap_info.rssi;
          strncpy(strongest_ssid, (const char *)ap_info.ssid,
                  sizeof(strongest_ssid));
          strongest_channel = ap_info.primary;
        }
      }

      if (strongest_rssi == INT8_MIN) {
        // TODO: handle failure
        ESP_LOGI(RADIO_TAG, "No known networks found");
        break;
      }

      ESP_LOGI(RADIO_TAG,
               "Strongest network is '%s' on channel %" PRIu8 " (RSSI %" PRId8
               ")",
               strongest_ssid, strongest_channel, strongest_rssi);

      const char *password = NULL;
      if (strcmp(strongest_ssid, RADIO_WIFI_SSID) == 0) {
        password = RADIO_WIFI_PASSWORD;
      } else {
        password = wifi_alt_password;
      }

      wifi_config_t config = {};
      strcpy((char *)config.sta.ssid, (const char *)strongest_ssid);
      strcpy((char *)config.sta.password, password);
      config.sta.channel = strongest_channel;
      config.sta.ft_enabled = true;
      esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &config);
      if (err != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "Failed to set wifi config: %d", err);
        break;
      }
      err = esp_wifi_connect();
      if (err != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "Failed to connect to wifi: %d", err);
      }

      break;
    }
    case WIFI_EVENT_AP_START: {
      ESP_LOGI(RADIO_TAG, "AP started");
      ESP_ERROR_CHECK_WITHOUT_ABORT(captive_dns_server_start());
      ESP_ERROR_CHECK_WITHOUT_ABORT(captive_http_server_start());
      break;
    }
    case WIFI_EVENT_AP_STOP: {
      ESP_LOGI(RADIO_TAG, "AP stopped");
      ESP_ERROR_CHECK_WITHOUT_ABORT(captive_http_server_stop());
      ESP_ERROR_CHECK_WITHOUT_ABORT(captive_dns_server_stop());
      break;
    }
    case WIFI_EVENT_AP_STACONNECTED: {
      wifi_event_ap_staconnected_t *event =
          (wifi_event_ap_staconnected_t *)event_data;
      ESP_LOGD(RADIO_TAG, "Station connected to AP MAC=" MACSTR,
               MAC2STR(event->mac));
      break;
    }
    }
  } else if (event_base == IP_EVENT) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
      already_scanned = false;

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

  test_connection_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(test_connection_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create test connection mutex");

  ESP_RETURN_ON_ERROR(nvs_open("radio:wifi", NVS_READWRITE, &wifi_nvs),
                      RADIO_TAG, "Failed to open wifi NVS storage: %d",
                      err_rc_);
  size_t len;
  esp_err_t err = nvs_get_str(wifi_nvs, "alt_ssid", NULL, &len);
  if (err == ESP_OK) {
    wifi_alt_ssid = calloc(1, len);
    ESP_RETURN_ON_FALSE(wifi_alt_ssid, ESP_ERR_NO_MEM, RADIO_TAG,
                        "Failed to allocate memory for alt_ssid");
    err = nvs_get_str(wifi_nvs, "alt_ssid", wifi_alt_ssid, &len);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get alt_ssid: %d", err);
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get alt_ssid: %d", err);
  }

  err = nvs_get_str(wifi_nvs, "alt_password", NULL, &len);
  if (err == ESP_OK) {
    wifi_alt_password = calloc(1, len);
    ESP_RETURN_ON_FALSE(wifi_alt_password, ESP_ERR_NO_MEM, RADIO_TAG,
                        "Failed to allocate memory for alt_password");
    err = nvs_get_str(wifi_nvs, "alt_password", wifi_alt_password, &len);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get alt_password: %d", err);
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get alt_password: %d", err);
  }

  xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_DISCONNECTED);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize wifi: %d", err);

  err = esp_netif_init();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize netif: %d", err);
  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();

  err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register wifi event handler: %d", err);
  err = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register ip event handler: %d",
                      err);

  // Setup NTP
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  config.start = false;
  config.sync_cb = wifi_clock_synced;
  err = esp_netif_sntp_init(&config);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize SNTP: %d", err);

  err = esp_wifi_set_mode(WIFI_MODE_STA);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi mode: %d", err);

  wifi_config_t wifi_config = {};
  err = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get wifi config: %d", err);

  // Configuration (including last connected network) gets stored in NVS, so if
  // we are able to read any config back, we have something we can try
  // re-connecting to. If not, we'll seed the config with the default network
  if (strlen((char *)wifi_config.sta.ssid) == 0) {
    ESP_LOGI(RADIO_TAG, "Seeding initial wifi config");
    // Wifi has not been configured previously, so populate defaults
    strcpy((char *)wifi_config.sta.ssid, RADIO_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, RADIO_WIFI_PASSWORD);
    wifi_config.sta.ft_enabled = true;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi mode: %d", err);
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi config: %d", err);
    // It's possible that we should be using power saving, at least in cases
    // (like funaround mode) where we don't care as much about a reliable wifi
    // connection, but in practice our battery life seems to be good enough that
    // it's not worth it.
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                        "Failed to set wifi power save mode: %d", err);
  }

  err = esp_wifi_start();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to start wifi: %d", err);

  things_register_telemetry_generator(&wifi_report_telemetry, "wifi", NULL);

  return ESP_OK;
}

esp_err_t wifi_force_reconnect() {
  already_scanned = false;
  esp_wifi_disconnect();
  return ESP_OK;
}

esp_err_t wifi_test_connection(const char *ssid, const char *password) {
  ESP_RETURN_ON_FALSE(ssid != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "SSID cannot be NULL");

  bool locked = false;
  esp_err_t ret = ESP_OK;

  // First disconnect and wait for the notification that we've disconnected
  xSemaphoreTake(test_connection_mutex, portMAX_DELAY);
  locked = true;
  ESP_GOTO_ON_FALSE(test_connection_task == NULL, ESP_ERR_INVALID_STATE,
                    cleanup, RADIO_TAG, "Test connection already in progress");
  test_connection_task = xTaskGetCurrentTaskHandle();
  ESP_GOTO_ON_ERROR(esp_wifi_disconnect(), cleanup, RADIO_TAG,
                    "Failed to disconnect from wifi: %d", err_rc_);

  ulTaskNotifyValueClear(NULL, UINT32_MAX);
  xSemaphoreGive(test_connection_mutex);
  locked = false;

  xTaskNotifyWait(0, UINT32_MAX, NULL, pdMS_TO_TICKS(5000));

  // Then configure and attempt to connect to the network
  xSemaphoreTake(test_connection_mutex, portMAX_DELAY);
  locked = true;
  test_connection_task = xTaskGetCurrentTaskHandle();
  wifi_config_t config = {.sta = {
                              .ft_enabled = true,
                          }};
  strncpy((char *)config.sta.ssid, ssid, sizeof(config.sta.ssid) - 1);
  if (password) {
    strncpy((char *)config.sta.password, password,
            sizeof(config.sta.password) - 1);
  }
  ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), cleanup,
                    RADIO_TAG, "Failed to set wifi config: %d", err_rc_);

  ESP_GOTO_ON_ERROR(esp_wifi_connect(), cleanup, RADIO_TAG,
                    "Failed to connect to wifi: %d", err_rc_);

  ulTaskNotifyValueClear(NULL, UINT32_MAX);
  xSemaphoreGive(test_connection_mutex);
  locked = false;

  uint32_t notif = 0;
  xTaskNotifyWait(0, UINT32_MAX, &notif, pdMS_TO_TICKS(30000));
  ESP_GOTO_ON_FALSE(notif == 1, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed or timed out connecting to network");

cleanup:
  if (!locked) {
    xSemaphoreTake(test_connection_mutex, portMAX_DELAY);
    locked = true;
  }

  if (test_connection_task == xTaskGetCurrentTaskHandle()) {
    test_connection_task = NULL;
  }

  xSemaphoreGive(test_connection_mutex);

  return ret;
}

esp_err_t wifi_set_alt_network(const char *ssid, const char *password) {
  ESP_RETURN_ON_FALSE(wifi_nvs != 0, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Wifi is not initialized");

  ESP_RETURN_ON_FALSE(ssid != NULL || password == NULL, ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "Password cannot be set without SSID");

  if (wifi_alt_ssid) {
    free(wifi_alt_ssid);
    wifi_alt_ssid = NULL;
  }

  if (wifi_alt_password) {
    free(wifi_alt_password);
    wifi_alt_password = NULL;
  }

  if (ssid) {
    esp_err_t err = nvs_set_str(wifi_nvs, "alt_ssid", ssid);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set alt_ssid: %d", err);
    wifi_alt_ssid = strdup(ssid);
  } else {
    esp_err_t err = nvs_erase_key(wifi_nvs, "alt_ssid");
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to erase alt_ssid: %d", err);
  }

  if (password) {
    esp_err_t err = nvs_set_str(wifi_nvs, "alt_password", password);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set alt_password: %d", err);
    wifi_alt_password = strdup(password);
  } else {
    esp_err_t err = nvs_erase_key(wifi_nvs, "alt_password");
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to erase alt_password: %d",
                        err);
  }

  return ESP_OK;
}

const char *wifi_get_alt_ssid() { return wifi_alt_ssid; }

esp_err_t wifi_force_scan() {
  ignore_scan_results = true;
  return esp_wifi_scan_start(NULL, true);
}

esp_err_t wifi_enable_ap() {
  uint8_t wifi_mac[6];
  esp_err_t err = esp_read_mac(wifi_mac, ESP_MAC_WIFI_STA);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to read wifi mac: %d", err);

  wifi_mode_t mode;
  err = esp_wifi_get_mode(&mode);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get wifi mode: %d", err);
  if (mode == WIFI_MODE_APSTA) {
    return ESP_OK;
  }

  err = esp_wifi_stop();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to stop wifi: %d", err);

  err = esp_wifi_set_mode(WIFI_MODE_APSTA);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi mode: %d", err);

  wifi_config_t ap_config = {
      .ap =
          {
              .max_connection = CONFIG_LWIP_DHCPS_MAX_STATION_NUM,
          },
  };
  snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid),
           "two-pi-radio-%02x%02x%02x", wifi_mac[3], wifi_mac[4], wifi_mac[5]);
  err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set AP config: %d", err);

  err = esp_wifi_start();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to start wifi: %d", err);

  return ESP_OK;
}

esp_err_t wifi_disable_ap() {
  wifi_mode_t mode;
  esp_err_t err = esp_wifi_get_mode(&mode);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get wifi mode: %d", err);
  if (mode == WIFI_MODE_STA) {
    return ESP_OK;
  }

  err = esp_wifi_stop();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to stop wifi: %d", err);

  err = esp_wifi_set_mode(WIFI_MODE_STA);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set wifi mode: %d", err);

  err = esp_wifi_start();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to start wifi: %d", err);

  return ESP_OK;
}
