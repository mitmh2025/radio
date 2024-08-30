#include "../config.h"
#include "things.h"
#include "main.h"

#include <string>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"
#include "esp_crt_bundle.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_core_dump.h"
#include "esp_partition.h"

#include "mbedtls/base64.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <Espressif_MQTT_Client.h>
#include <Espressif_Updater.h>
#include <ThingsBoard.h>

#ifndef RADIO_THINGSBOARD_SERVER
#error "RADIO_THINGSBOARD_SERVER is not defined"
#else
static_assert(strlen(RADIO_THINGSBOARD_SERVER) > 0, "RADIO_THINGSBOARD_SERVER is empty");
#endif

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

vprintf_like_t things_orig_vprintf = NULL;

int things_vprintf(const char *format, va_list args)
{
  char *buf = NULL;
  int len = 0;
  if (tb.connected())
  {
    len = vsnprintf(NULL, 0, format, args);
    if (len < 0)
    {
      goto cleanup;
    }
    buf = (char *)malloc(len + 1);
    if (!buf)
    {
      goto cleanup;
    }

    len = vsnprintf(buf, len + 1, format, args);
    if (len < 0)
    {
      goto cleanup;
    }
    tb.sendTelemetryData("log", buf);
  }

cleanup:
  free(buf);

  if (things_orig_vprintf)
  {
    return things_orig_vprintf(format, args);
  }
  else
  {
    return len;
  }
}

static struct {
  configRUN_TIME_COUNTER_TYPE run_time_counter;
  configRUN_TIME_COUNTER_TYPE cpu0_idle_counter;
  configRUN_TIME_COUNTER_TYPE cpu1_idle_counter;
} things_last_cpu_usage = {};

static void things_telemetry_generator()
{
  // Report basic metrics
  things_send_telemetry_int("uptime", esp_timer_get_time() / (1000 * 1000));

  configRUN_TIME_COUNTER_TYPE run_time_counter = 0;
  portALT_GET_RUN_TIME_COUNTER_VALUE(run_time_counter);
  configRUN_TIME_COUNTER_TYPE cpu0_idle_counter = ulTaskGetIdleRunTimeCounterForCore(0);
  configRUN_TIME_COUNTER_TYPE cpu1_idle_counter = ulTaskGetIdleRunTimeCounterForCore(1);

  if (things_last_cpu_usage.run_time_counter != 0 && run_time_counter != things_last_cpu_usage.run_time_counter)
  {
    configRUN_TIME_COUNTER_TYPE run_time_delta = run_time_counter - things_last_cpu_usage.run_time_counter;
    configRUN_TIME_COUNTER_TYPE cpu0_idle_delta = cpu0_idle_counter - things_last_cpu_usage.cpu0_idle_counter;
    configRUN_TIME_COUNTER_TYPE cpu1_idle_delta = cpu1_idle_counter - things_last_cpu_usage.cpu1_idle_counter;

    float cpu0_usage = 100.0 * (run_time_delta - cpu0_idle_delta) / run_time_delta;
    float cpu1_usage = 100.0 * (run_time_delta - cpu1_idle_delta) / run_time_delta;

    things_send_telemetry_float("cpu0_usage_percent", cpu0_usage);
    things_send_telemetry_float("cpu1_usage_percent", cpu1_usage);
  }

  things_last_cpu_usage.run_time_counter = run_time_counter;
  things_last_cpu_usage.cpu0_idle_counter = cpu0_idle_counter;
  things_last_cpu_usage.cpu1_idle_counter = cpu1_idle_counter;

  things_send_telemetry_int("task_count", uxTaskGetNumberOfTasks());

  things_send_telemetry_int("psram_free", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  things_send_telemetry_int("dram_free", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  things_send_telemetry_int("psram_total", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
  things_send_telemetry_int("dram_total", heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
  things_send_telemetry_int("psram_lwm", heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
  things_send_telemetry_int("dram_lwm", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}

static void things_connect_callback(void)
{
  if (things_task_handle != NULL)
    xTaskNotifyGive(things_task_handle);
}

static void things_progress_callback(const size_t &currentChunk, const size_t &totalChuncks)
{
  ESP_LOGV(RADIO_TAG, "Firmware update progress %d/%d chunks (%.2f%%)", currentChunk, totalChuncks, static_cast<float>(100 * currentChunk) / totalChuncks);
}

static void things_updated_callback(const bool &success)
{
  if (!success)
  {
    ESP_LOGE(RADIO_TAG, "Failed to update firmware");
    abort();
  }

  // TODO: setup r/w lock to allow blocking updates
  ESP_LOGI(RADIO_TAG, "Successfully updated firmware; restarting now");
  esp_restart();
}

static void things_upload_coredump(size_t core_size)
{
  const esp_partition_t *partition;
  const void *core_dump_addr = NULL;
  esp_partition_mmap_handle_t handle;
  esp_err_t err = ESP_OK;
  size_t base64_size;
  char *base64_core_dump = NULL;
  int ret;

  partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
  if (partition == NULL)
  {
    ESP_LOGE(RADIO_TAG, "Failed to find coredump partition");
    return;
  }

  err = esp_partition_mmap(partition, 0, core_size, ESP_PARTITION_MMAP_DATA, &core_dump_addr, &handle);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to map coredump partition: %s", esp_err_to_name(err));
    goto cleanup;
  }

  ret = mbedtls_base64_encode(NULL, 0, &base64_size, (const unsigned char *)core_dump_addr, core_size);
  if (ret != 0)
  {
    ESP_LOGE(RADIO_TAG, "Failed to get base64 size for coredump: %d", ret);
    goto cleanup;
  }

  base64_core_dump = (char *)malloc(base64_size);
  if (base64_core_dump == NULL)
  {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for base64 coredump");
    goto cleanup;
  }

  ret = mbedtls_base64_encode((unsigned char *)base64_core_dump, base64_size, &base64_size, (const unsigned char *)core_dump_addr, core_size);
  if (ret != 0)
  {
    ESP_LOGE(RADIO_TAG, "Failed to base64 encode coredump: %d", ret);
    goto cleanup;
  }

  tb.sendTelemetryData("coredump", base64_core_dump);

  ESP_LOGI(RADIO_TAG, "Uploading coredump to ThingsBoard");

cleanup:
  if (base64_core_dump != NULL)
  {
    free(base64_core_dump);
  }

  if (core_dump_addr != NULL)
  {
    esp_partition_munmap(handle);
  }
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

    size_t core_addr, core_size;
    esp_err_t err = esp_core_dump_image_get(&core_addr, &core_size);
    if (err == ESP_OK)
    {
      ESP_LOGW(RADIO_TAG, "Core dump detected at 0x%08x, size %d bytes", core_addr, core_size);
      things_upload_coredump(core_size);
    }
    else if (err != ESP_ERR_NOT_FOUND && err != ESP_ERR_INVALID_SIZE)
    {
      ESP_LOGE(RADIO_TAG, "Failed to get core dump: %s", esp_err_to_name(err));
    }
    // No matter what happened, erase the coredump so we don't re-upload it
    err = esp_core_dump_image_erase();
    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to erase core dump: %s", esp_err_to_name(err));
    }

    // Note that the project name is generated from the top-level CMakeLists.txt
    // file; the version is automatically generated, most likely from git
    // describe - see
    // https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/api-guides/build-system.html#build-project-variables
    // for details
    const esp_app_desc_t *app = esp_app_get_description();

    success = tb.Firmware_Send_Info(app->project_name, app->version);
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

    const OTA_Update_Callback callback(
        things_progress_callback,
        things_updated_callback,
        app->project_name,
        app->version,
        &updater,
        FIRMWARE_FAILURE_RETRIES,
        FIRMWARE_PACKET_SIZE);

    // First subscribe to updates, in case there's not one already pending
    success = tb.Subscribe_Firmware_Update(callback);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to subscribe to firmware updates from ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      tb.disconnect();
      continue;
    }

    // Then manually request an update, since Subscribe_Firmware_Update is
    // edge-triggered not level-triggered
    success = tb.Start_Firmware_Update(callback);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to request an immediate firmware updates from ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      tb.disconnect();
      continue;
    }

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
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));

  things_orig_vprintf = esp_log_set_vprintf(things_vprintf);

  mqtt_client.set_connect_callback(things_connect_callback);
  mqtt_client.set_server_crt_bundle_attach(esp_crt_bundle_attach);

  xTaskCreatePinnedToCore(things_task, "things_task", 4096, NULL, 5, &things_task_handle, 1);

  size_t length;
  err = nvs_get_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, NULL, &length);
  if (err == ESP_OK)
  {
    things_token.resize(length);
    err = nvs_get_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, things_token.data(), &length);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get device token: %s", esp_err_to_name(err));
    xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED);
  }
  else if (err != ESP_ERR_NVS_NOT_FOUND)
  {
    ESP_LOGE(RADIO_TAG, "Failed to check device token: %s", esp_err_to_name(err));
    return err;
  }

  things_register_telemetry_generator(things_telemetry_generator);

  return ESP_OK;
}

extern "C" esp_err_t things_provision(const char *token)
{
  if (things_nvs_handle == 0)
  {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = nvs_set_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, token);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set device token: %s", esp_err_to_name(err));
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

extern "C" bool things_send_telemetry_float(char const *const key, float value)
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
