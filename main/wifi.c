#include "wifi.h"
#include "../config.h"
#include "captive_dns_server.h"
#include "captive_http_server.h"
#include "main.h"
#include "station_2pi.h"
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

typedef enum {
  WIFI_STATE_DEFAULT_CONNECTION,
  WIFI_STATE_TARGETED_CONNECTION,
  WIFI_STATE_FORCE_DISCONNECT,
  WIFI_STATE_TEST_CONNECTION,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_FAILED,
} wifi_state_t;

#define WIFI_DEFAULT_CONNECT_TIMEOUT (5 * 1000 * 1000)
#define WIFI_TARGETED_CONNECT_TIMEOUT (15 * 1000 * 1000)
#define WIFI_FAILED_RETRY_TIMEOUT (1000 * 1000)

static SemaphoreHandle_t mutex = NULL;
static wifi_state_t state = WIFI_STATE_DEFAULT_CONNECTION;
static esp_timer_handle_t watchdog_timer = NULL;
static bool manual_scan = false;
static TaskHandle_t test_connection_task = NULL;

static nvs_handle_t wifi_nvs = 0;
static char *wifi_alt_ssid = NULL;
static char *wifi_alt_password = NULL;

static const char *wifi_state_to_string(wifi_state_t wifi_state) {
  switch (wifi_state) {
  case WIFI_STATE_DEFAULT_CONNECTION:
    return "WIFI_STATE_DEFAULT_CONNECTION";
  case WIFI_STATE_TARGETED_CONNECTION:
    return "WIFI_STATE_TARGETED_CONNECTION";
  case WIFI_STATE_FORCE_DISCONNECT:
    return "WIFI_STATE_FORCE_DISCONNECT";
  case WIFI_STATE_TEST_CONNECTION:
    return "WIFI_STATE_TEST_CONNECTION";
  case WIFI_STATE_CONNECTED:
    return "WIFI_STATE_CONNECTED";
  case WIFI_STATE_FAILED:
    return "WIFI_STATE_FAILED";
  default:
    return "unknown";
  }
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

esp_err_t wifi_get_ap_network_mac(uint8_t *mac) {
  return esp_read_mac(mac, ESP_MAC_WIFI_STA);
}

// Must call with mutex
static void set_state(wifi_state_t new_state) {
  ESP_LOGD(RADIO_TAG, "Changing wifi state from %s to %s",
           wifi_state_to_string(state), wifi_state_to_string(new_state));
  esp_timer_stop(watchdog_timer);

  switch (new_state) {
  case WIFI_STATE_DEFAULT_CONNECTION:
    // Attempt to connect with the current config and hope for the best
    esp_wifi_connect();
    esp_timer_start_once(watchdog_timer, WIFI_DEFAULT_CONNECT_TIMEOUT);
    break;
  case WIFI_STATE_TARGETED_CONNECTION:
    // Trigger a scan
    esp_wifi_scan_start(NULL, false);
    esp_timer_start_once(watchdog_timer, WIFI_TARGETED_CONNECT_TIMEOUT);
    break;
  case WIFI_STATE_FORCE_DISCONNECT:
    // Anything using this state is triggering a disconnect, so we don't need to
    break;
  case WIFI_STATE_TEST_CONNECTION:
    // wifi_test_connection triggers the connection so we just need to set a
    // watchdog
    esp_timer_start_once(watchdog_timer, WIFI_TARGETED_CONNECT_TIMEOUT);
    break;
  case WIFI_STATE_CONNECTED:
    xEventGroupClearBits(radio_event_group,
                         RADIO_EVENT_GROUP_WIFI_DISCONNECTED);
    xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
    station_2pi_need_wifi_announcement(false);
    if (test_connection_task) {
      xTaskNotify(test_connection_task, 1, eSetValueWithOverwrite);
      test_connection_task = NULL;
    }
    break;
  case WIFI_STATE_FAILED:
    station_2pi_need_wifi_announcement(true);
    esp_timer_start_once(watchdog_timer, WIFI_FAILED_RETRY_TIMEOUT);
    break;
  }

  state = new_state;
}

static void connection_failed() {
  ESP_LOGI(RADIO_TAG, "Wifi connection failed");

  // Cancel watchdog timer if still pending (ignore errors in case it's not)
  esp_timer_stop(watchdog_timer);

  xSemaphoreTake(mutex, portMAX_DELAY);

  xEventGroupClearBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED);
  xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_DISCONNECTED);

  // change what to do based on state:
  // - If we're in TEST_CONNECTION_DISCONNECT, notify the task and go to
  //   TEST_CONNECTION (the task will trigger the actual connection)
  // - If we're in TEST_CONNECTION, notify the task and fallthrough
  // - If we're in FAILED, fallthrough
  // - If we're in DEFAULT_CONNECTION, set to TARGETED_CONNECTION and trigger a
  //   scan
  // - If we're in TARGETED_CONNECTION, set to FAILED
  // - If we're in SCANNING, ignore because we didn't set a watchdog
  // - If we're in CONNECTED, reset to DEFAULT_CONNECTION
  switch (state) {
  case WIFI_STATE_FORCE_DISCONNECT:
    if (test_connection_task) {
      xTaskNotify(test_connection_task, 0, eSetValueWithOverwrite);
      test_connection_task = NULL;
    }
    break;
  case WIFI_STATE_TEST_CONNECTION:
    // We timed out, so send a negative response to the task then scan
    if (test_connection_task) {
      xTaskNotify(test_connection_task, 0, eSetValueWithOverwrite);
      test_connection_task = NULL;
    }
    set_state(WIFI_STATE_TARGETED_CONNECTION);
    break;
  case WIFI_STATE_FAILED:
    // We've been in the failed state for a while so scan
    set_state(WIFI_STATE_TARGETED_CONNECTION);
    break;
  case WIFI_STATE_DEFAULT_CONNECTION:
    // We timed out, so trigger a scan
    set_state(WIFI_STATE_TARGETED_CONNECTION);
    break;
  case WIFI_STATE_TARGETED_CONNECTION:
    // We timed out on a targeted connection so we're in the failed state
    set_state(WIFI_STATE_FAILED);
    break;
  case WIFI_STATE_CONNECTED:
    // We got disconnected so loop back to the beginning of the state machine
    set_state(WIFI_STATE_DEFAULT_CONNECTION);
    break;
  default:
    ESP_LOGE(RADIO_TAG, "Invalid state %d", state);
    set_state(WIFI_STATE_FAILED);
    break;
  }
  xSemaphoreGive(mutex);
}

static void wifi_clock_synced(struct timeval *tv) {
  // Format time as iso8601 string
  struct tm timeinfo;
  gmtime_r(&tv->tv_sec, &timeinfo);
  char strftime_buf[sizeof("2000-01-01T00:00:00Z") + 1];
  strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  ESP_LOGI(RADIO_TAG, "SNTP time synced to %s", strftime_buf);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGD(RADIO_TAG, "Wifi started");
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_STOP:
      ESP_LOGI(RADIO_TAG, "Wifi stopped");
      connection_failed();
      break;
    case WIFI_EVENT_STA_CONNECTED: {
      wifi_event_sta_connected_t *event =
          (wifi_event_sta_connected_t *)event_data;
      ESP_LOGI(RADIO_TAG, "Wifi connected ssid='%.*s' channel=%u",
               event->ssid_len, event->ssid, event->channel);
      break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
      wifi_event_sta_disconnected_t *event =
          (wifi_event_sta_disconnected_t *)event_data;
      ESP_LOGI(RADIO_TAG,
               "Disconnected from (or failed to connect to) network %.*s, "
               "reason %d",
               event->ssid_len, event->ssid, event->reason);

      // Ignore disconnects for events that are self-remediating
      if (event->reason == WIFI_REASON_ROAMING) {
        break;
      }

      connection_failed();
      break;
    }

    case WIFI_EVENT_SCAN_DONE: {
      xSemaphoreTake(mutex, portMAX_DELAY);
      bool was_manual_scan = manual_scan;
      manual_scan = false;
      xSemaphoreGive(mutex);

      if (was_manual_scan) {
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
        ESP_LOGI(RADIO_TAG, "No known networks found, trying default network");
        strncpy(strongest_ssid, RADIO_WIFI_SSID, sizeof(strongest_ssid));
        strongest_channel = 0;
        strongest_rssi = 0;
        break;
      } else {
        ESP_LOGI(RADIO_TAG,
                 "Strongest network is '%s' on channel %" PRIu8 " (RSSI %" PRId8
                 "), attempting to connect",
                 strongest_ssid, strongest_channel, strongest_rssi);
      }

      const char *password = NULL;
      if (strcmp(strongest_ssid, RADIO_WIFI_SSID) == 0) {
        password = RADIO_WIFI_PASSWORD;
      } else {
        password = wifi_alt_password;
      }

      wifi_config_t config = {
          .sta =
              {
                  .btm_enabled = 1,
                  .rm_enabled = 1,
                  .mbo_enabled = 1,
                  .ft_enabled = 1,
              },
      };
      strcpy((char *)config.sta.ssid, (const char *)strongest_ssid);
      strcpy((char *)config.sta.password, password);
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
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(RADIO_TAG, "Got IP address ip=" IPSTR,
               IP2STR(&event->ip_info.ip));
      esp_err_t err = esp_netif_sntp_start();
      if (err != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "Failed to start SNTP: %d", err);
      }

      xSemaphoreTake(mutex, portMAX_DELAY);
      set_state(WIFI_STATE_CONNECTED);
      xSemaphoreGive(mutex);

      break;
    }
    case IP_EVENT_STA_LOST_IP:
      ESP_LOGI(RADIO_TAG, "Lost IP address");
      connection_failed();
      break;
    }
  } else {
    ESP_LOGI(RADIO_TAG, "Unknown event base %s", event_base);
  }
}

esp_err_t wifi_init() {
  ESP_LOGD(RADIO_TAG, "Setting up wifi");

  mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create wifi mutex");

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .callback = (esp_timer_cb_t)connection_failed,
                              .name = "wifi_watchdog",
                          },
                          &watchdog_timer),
                      RADIO_TAG, "Failed to create wifi watchdog");

  ESP_RETURN_ON_ERROR(nvs_open("radio:wifi", NVS_READWRITE, &wifi_nvs),
                      RADIO_TAG, "Failed to open wifi NVS storage: %d",
                      err_rc_);
  size_t len;
  esp_err_t ret = nvs_get_str(wifi_nvs, "alt_ssid", NULL, &len);
  if (ret == ESP_OK) {
    wifi_alt_ssid = calloc(1, len);
    ESP_RETURN_ON_FALSE(wifi_alt_ssid, ESP_ERR_NO_MEM, RADIO_TAG,
                        "Failed to allocate memory for alt_ssid");
    ret = nvs_get_str(wifi_nvs, "alt_ssid", wifi_alt_ssid, &len);
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get alt_ssid: %d", ret);
  } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get alt_ssid: %d", ret);
  }

  ret = nvs_get_str(wifi_nvs, "alt_password", NULL, &len);
  if (ret == ESP_OK) {
    wifi_alt_password = calloc(1, len);
    ESP_RETURN_ON_FALSE(wifi_alt_password, ESP_ERR_NO_MEM, RADIO_TAG,
                        "Failed to allocate memory for alt_password");
    ret = nvs_get_str(wifi_nvs, "alt_password", wifi_alt_password, &len);
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get alt_password: %d", ret);
  } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get alt_password: %d", ret);
  }

  xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_DISCONNECTED);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&cfg);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to initialize wifi: %d", ret);

  ret = esp_netif_init();
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to initialize netif: %d", ret);
  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();

  ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG,
                      "Failed to register wifi event handler: %d", ret);
  ret = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to register ip event handler: %d",
                      ret);

  // Setup NTP
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  config.start = false;
  config.sync_cb = wifi_clock_synced;
  ret = esp_netif_sntp_init(&config);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to initialize SNTP: %d", ret);

  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to set wifi mode: %d", ret);

  wifi_config_t wifi_config = {};
  ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get wifi config: %d", ret);

  // It's possible that we should be using power saving, at least in cases
  // (like funaround mode) where we don't care as much about a reliable wifi
  // connection, but in practice our battery life seems to be good enough that
  // it's not worth it.
  ret = esp_wifi_set_ps(WIFI_PS_NONE);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to set wifi power save mode: %d",
                      ret);

  xSemaphoreTake(mutex, portMAX_DELAY);

  // Configuration (including last connected network) gets stored in NVS, so if
  // we are able to read any config back, we have something we can try
  // re-connecting to. If not, we'll seed the config with the default network
  if (strlen((char *)wifi_config.sta.ssid) == 0) {
    ESP_LOGI(RADIO_TAG, "No wifi config found, defaulting to scan");
    set_state(WIFI_STATE_TARGETED_CONNECTION);
  } else {
    set_state(WIFI_STATE_DEFAULT_CONNECTION);
  }

  xSemaphoreGive(mutex);

  ret = esp_wifi_start();
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to start wifi: %d", ret);

  things_register_telemetry_generator(&wifi_report_telemetry, "wifi", NULL);

  return ret;
}

esp_err_t wifi_force_reconnect() {
  ESP_LOGI(RADIO_TAG, "Forcing wifi reconnect");
  connection_failed();
  return ESP_OK;
}

esp_err_t wifi_test_connection(const char *ssid, const char *password) {
  ESP_RETURN_ON_FALSE(ssid != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "SSID cannot be NULL");

  bool locked = false;
  esp_err_t ret = ESP_OK;

  // First disconnect and wait for the notification that we've disconnected
  xSemaphoreTake(mutex, portMAX_DELAY);
  locked = true;
  ESP_GOTO_ON_FALSE(test_connection_task == NULL, ESP_ERR_INVALID_STATE,
                    cleanup, RADIO_TAG, "Test connection already in progress");
  test_connection_task = xTaskGetCurrentTaskHandle();
  set_state(WIFI_STATE_FORCE_DISCONNECT);
  esp_wifi_disconnect();

  ulTaskNotifyValueClear(NULL, UINT32_MAX);
  xSemaphoreGive(mutex);
  locked = false;

  xTaskNotifyWait(0, UINT32_MAX, NULL, pdMS_TO_TICKS(5000));

  // Then configure and attempt to connect to the network
  xSemaphoreTake(mutex, portMAX_DELAY);
  locked = true;
  ESP_GOTO_ON_FALSE(test_connection_task == NULL, ESP_ERR_INVALID_STATE,
                    cleanup, RADIO_TAG, "Raced with another test connection");
  test_connection_task = xTaskGetCurrentTaskHandle();
  wifi_config_t config = {
      .sta =
          {
              .ft_enabled = true,
          },
  };
  strncpy((char *)config.sta.ssid, ssid, sizeof(config.sta.ssid) - 1);
  if (password) {
    strncpy((char *)config.sta.password, password,
            sizeof(config.sta.password) - 1);
  }
  ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), cleanup,
                    RADIO_TAG, "Failed to set wifi config: %d", err_rc_);

  set_state(WIFI_STATE_TEST_CONNECTION);
  ESP_GOTO_ON_ERROR(esp_wifi_connect(), cleanup, RADIO_TAG,
                    "Failed to connect to wifi: %d", err_rc_);

  ulTaskNotifyValueClear(NULL, UINT32_MAX);
  xSemaphoreGive(mutex);
  locked = false;

  uint32_t notif = 0;
  xTaskNotifyWait(0, UINT32_MAX, &notif, pdMS_TO_TICKS(30000));
  ESP_GOTO_ON_FALSE(notif == 1, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed or timed out connecting to network");

cleanup:
  if (!locked) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    locked = true;
  }

  if (test_connection_task == xTaskGetCurrentTaskHandle()) {
    test_connection_task = NULL;
  }

  xSemaphoreGive(mutex);

  return ret;
}

esp_err_t wifi_set_alt_network(const char *ssid, const char *password) {
  ESP_RETURN_ON_FALSE(wifi_nvs != 0, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Wifi is not initialized");

  ESP_RETURN_ON_FALSE(ssid != NULL || password == NULL, ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "Password cannot be set without SSID");

  xSemaphoreTake(mutex, portMAX_DELAY);

  wifi_ap_record_t ap_info;
  esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
  if (ret == ESP_OK && strcmp((const char *)ap_info.ssid, wifi_alt_ssid) == 0) {
    ESP_LOGI(RADIO_TAG, "Forcing scan to find new network");
    set_state(WIFI_STATE_TARGETED_CONNECTION);
  }

  if (wifi_alt_ssid) {
    free(wifi_alt_ssid);
    wifi_alt_ssid = NULL;
  }

  if (wifi_alt_password) {
    free(wifi_alt_password);
    wifi_alt_password = NULL;
  }

  if (ssid) {
    ret = nvs_set_str(wifi_nvs, "alt_ssid", ssid);
    ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to set alt_ssid: %d",
                      ret);
    wifi_alt_ssid = strdup(ssid);
  } else {
    ret = nvs_erase_key(wifi_nvs, "alt_ssid");
    ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to erase alt_ssid: %d",
                      ret);
  }

  if (password) {
    ret = nvs_set_str(wifi_nvs, "alt_password", password);
    ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to set alt_password: %d",
                      ret);
    wifi_alt_password = strdup(password);
  } else {
    ret = nvs_erase_key(wifi_nvs, "alt_password");
    ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG,
                      "Failed to erase alt_password: %d", ret);
  }

cleanup:
  xSemaphoreGive(mutex);

  return ret;
}

const char *wifi_get_alt_ssid() { return wifi_alt_ssid; }

esp_err_t wifi_force_scan() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  manual_scan = true;
  esp_err_t ret = esp_wifi_scan_start(NULL, true);
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t wifi_enable_ap() {
  uint8_t wifi_mac[6];
  esp_err_t err = wifi_get_ap_network_mac(wifi_mac);
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
