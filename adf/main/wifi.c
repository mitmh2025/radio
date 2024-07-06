#include "wifi.h"

#include <string.h>

#include "sdkconfig.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#ifndef CONFIG_RADIO_WIFI_SSID
#error "CONFIG_RADIO_WIFI_SSID is not defined"
#else
static_assert(strlen(CONFIG_RADIO_WIFI_SSID) > 0, "CONFIG_RADIO_WIFI_SSID is empty");
#endif

#ifndef CONFIG_RADIO_WIFI_PASSWORD
#error "CONFIG_RADIO_WIFI_PASSWORD is not defined"
#else
static_assert(strlen(CONFIG_RADIO_WIFI_PASSWORD) > 0, "CONFIG_RADIO_WIFI_PASSWORD is empty");
#endif

EventGroupHandle_t wifi_event_group;

static const char *TAG = "radio:wifi";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT)
  {
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
      ESP_LOGI(TAG, "Wifi started");
      xEventGroupSetBits(wifi_event_group, WIFI_EVENT_GROUP_STA_STARTED);
      ESP_ERROR_CHECK(esp_wifi_connect());
      break;
    case WIFI_EVENT_STA_STOP:
      ESP_LOGI(TAG, "Wifi stopped");
      xEventGroupClearBits(wifi_event_group, WIFI_EVENT_GROUP_STA_STARTED | WIFI_EVENT_GROUP_STA_CONNECTED | WIFI_EVENT_GROUP_STA_GOT_IP);
      break;
    case WIFI_EVENT_STA_CONNECTED:
    {
      wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
      ESP_LOGI(TAG, "Wifi connected ssid='%.*s' channel=%u", event->ssid_len, event->ssid, event->channel);
      xEventGroupSetBits(wifi_event_group, WIFI_EVENT_GROUP_STA_CONNECTED);
      break;
    }
    case WIFI_EVENT_STA_DISCONNECTED:
      xEventGroupClearBits(wifi_event_group, WIFI_EVENT_GROUP_STA_CONNECTED | WIFI_EVENT_GROUP_STA_GOT_IP);
      break;
    }
  }
  else if (event_base == IP_EVENT)
  {
    switch (event_id)
    {
    case IP_EVENT_STA_GOT_IP:
    {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG, "Got IP address ip=" IPSTR, IP2STR(&event->ip_info.ip));
      xEventGroupSetBits(wifi_event_group, WIFI_EVENT_GROUP_STA_GOT_IP);
      break;
    }
    }
  }
  else
  {
    ESP_LOGI(TAG, "Unknown event base %s", event_base);
  }
}

void wifi_setup()
{
  wifi_event_group = xEventGroupCreate();
  if (wifi_event_group == NULL)
  {
    ESP_LOGE(TAG, "Failed to create event group");
    abort();
    return;
  }

  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);

  wifi_config_t wifi_config = {
      .sta = {
          .ssid = CONFIG_RADIO_WIFI_SSID,
          .password = CONFIG_RADIO_WIFI_PASSWORD,
          .threshold = {
              .authmode = WIFI_AUTH_WPA_PSK,
          }}};

  // TODO: AP/captive portal mode
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}
