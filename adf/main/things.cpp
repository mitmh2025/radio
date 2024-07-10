#include "../config.h"
#include "things.h"
#include "main.h"

#include <string>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"
#include "esp_crt_bundle.h"

#include <Espressif_MQTT_Client.h>
#include <Espressif_Updater.h>
#include <ThingsBoard.h>

#ifndef RADIO_THINGSBOARD_SERVER
#error "RADIO_THINGSBOARD_SERVER is not defined"
#else
static_assert(strlen(RADIO_THINGSBOARD_SERVER) > 0, "RADIO_THINGSBOARD_SERVER is empty");
#endif

static constexpr char CURRENT_FIRMWARE_TITLE[] = "radio";
// TODO: figure out how to generate this as a compile thing
static constexpr char CURRENT_FIRMWARE_VERSION[] = "0.0.0";

static constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
static constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;
static constexpr uint16_t MAX_MESSAGE_SIZE = 256U;

static constexpr char THINGSBOARD_SERVER[] = RADIO_THINGSBOARD_SERVER;
static constexpr uint16_t THINGSBOARD_PORT = 8883U;

static Espressif_MQTT_Client mqtt_client;
static ThingsBoard tb(mqtt_client, MAX_MESSAGE_SIZE);
static Espressif_Updater updater;

static constexpr char THINGS_NVS_NAMESPACE[] = "radio:things";
static constexpr char THINGS_NVS_TOKEN_KEY[] = "devicetoken";
static nvs_handle_t things_nvs_handle;

static TaskHandle_t things_task_handle = NULL;
static std::string things_token;
static std::vector<void (*)(void)> telemetry_generators;

static constexpr TickType_t CONNECT_TIMEOUT = pdMS_TO_TICKS(5000);

static void things_connect_callback(void)
{
  if (things_task_handle != NULL)
    xTaskNotifyGive(things_task_handle);
}

static void things_task(void *arg)
{
  // First, block until we're provisioned
  xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED, pdFALSE, pdTRUE, portMAX_DELAY);

  // Main loop is around wifi connection. If we get disconnected from wifi, wait
  // until we reconnect and then reconnect to ThingsBoard too
  while (true)
  {
    xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

    bool success = tb.connect(THINGSBOARD_SERVER, things_token.c_str(), THINGSBOARD_PORT);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to connect to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      tb.disconnect();
      continue;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t deadline = xTaskGetTickCount() + CONNECT_TIMEOUT;
    while (!tb.connected() && now < deadline)
    {
      xTaskNotifyWait(0, ULONG_MAX, NULL, deadline - now);
      now = xTaskGetTickCount();
    }

    if (!tb.connected())
    {
      ESP_LOGE(RADIO_TAG, "Failed to connect to ThingsBoard within timeout. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      tb.disconnect();
      continue;
    }

    ESP_LOGI(RADIO_TAG, "Connected to ThingsBoard");

    success = tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to send firmware info to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      tb.disconnect();
      continue;
    }
    success = tb.Firmware_Send_State("UPDATED");
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to send firmware state to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      tb.disconnect();
      continue;
    }

    // TODO: setup OTA

    while (true)
    {
      for (auto generator : telemetry_generators)
      {
        generator();
      }

      // If the wifi disconnect bit remains unset after 30 seconds, send telemetry again
      EventBits_t bits = xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(30000));
      if (bits & RADIO_EVENT_GROUP_WIFI_DISCONNECTED)
      {
        break;
      }
    }
    tb.disconnect();
  }

  vTaskDelete(NULL);
}

extern "C" esp_err_t things_init()
{
  if (things_nvs_handle != 0)
  {
    ESP_LOGE(RADIO_TAG, "ThingsBoard already initialized");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = nvs_open(THINGS_NVS_NAMESPACE, NVS_READWRITE, &things_nvs_handle);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
    return err;
  }

  mqtt_client.set_connect_callback(things_connect_callback);
  mqtt_client.set_server_crt_bundle_attach(esp_crt_bundle_attach);

  xTaskCreatePinnedToCore(things_task, "things_task", 4096, NULL, 5, &things_task_handle, 1);

  size_t length;
  err = nvs_get_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, NULL, &length);
  if (err == ESP_OK)
  {
    things_token.resize(length);
    err = nvs_get_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, things_token.data(), &length);
    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to get device token: %s", esp_err_to_name(err));
      return err;
    }
    xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED);
  }
  else if (err != ESP_ERR_NVS_NOT_FOUND)
  {
    ESP_LOGE(RADIO_TAG, "Failed to check device token: %s", esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}

extern "C" esp_err_t things_provision(const char *token)
{
  if (things_nvs_handle == 0)
  {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = nvs_set_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, token);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to set device token: %s", esp_err_to_name(err));
    return err;
  }
  things_token = token;
  xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED);

  return ESP_OK;
}

extern "C" esp_err_t things_deprovision()
{
  if (things_nvs_handle == 0)
  {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = nvs_erase_key(things_nvs_handle, THINGS_NVS_TOKEN_KEY);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to erase device token: %s", esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}

extern "C" bool things_send_telemetry_string(char const *const key, char const *const value)
{
  if (!tb.connected())
  {
    return false;
  }
  return tb.sendTelemetryData(key, value);
}

extern "C" bool things_send_telemetry_double(char const *const key, double value)
{
  if (!tb.connected())
  {
    return false;
  }
  return tb.sendTelemetryData(key, value);
}

extern "C" bool things_send_telemetry_int(char const *const key, long long value)
{
  if (!tb.connected())
  {
    return false;
  }
  return tb.sendTelemetryData(key, value);
}

extern "C" bool things_send_telemetry_bool(char const *const key, bool value)
{
  if (!tb.connected())
  {
    return false;
  }
  return tb.sendTelemetryData(key, value);
}

extern "C" void things_register_telemetry_generator(void (*generator)(void))
{
  telemetry_generators.push_back(generator);
}
