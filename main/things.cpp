#include "nvs_flash.h"
#include "nvs_handle.hpp"

#include "../config.h"
#include "things.h"
#include "main.h"

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <string>
#include <variant>
#include <vector>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_core_dump.h"
#include "esp_partition.h"
#include "esp_random.h"

#include "mbedtls/base64.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <Espressif_MQTT_Client.h>
#include <Espressif_Updater.h>
#include <ThingsBoard.h>

#ifndef RADIO_THINGSBOARD_SERVER
#error "RADIO_THINGSBOARD_SERVER is not defined"
#else
static_assert(strlen(RADIO_THINGSBOARD_SERVER) > 0, "RADIO_THINGSBOARD_SERVER is empty (make sure config.h exists and is populated)");
#endif

class ESPLogger
{
public:
  template <typename... Args>
  static int printfln(char const *const format, Args const &...args)
  {
    int const size = Helper::detectSize(format, args...);
    char formatted[size] = {};
    int const written = snprintf(formatted, size, format, args...);
    ESP_LOGI("ThingsBoard", "%s", formatted);
    return written;
  }

  static int println(char const *const message)
  {
    return printfln("%s", message);
  }
};

using RadioThingsBoard = ThingsBoardSized<ESPLogger>;

static constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
static constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;
static constexpr uint16_t MAX_MESSAGE_SIZE = UINT16_MAX; // Max core dump size

static constexpr char THINGSBOARD_SERVER[] = RADIO_THINGSBOARD_SERVER;
static constexpr uint16_t THINGSBOARD_PORT = 8883U;

static std::weak_ptr<RadioThingsBoard> tb;

static constexpr char THINGS_NVS_NAMESPACE[] = "radio:things";
static constexpr char THINGS_NVS_TOKEN_KEY[] = "devicetoken";
static nvs_handle_t things_nvs_handle;

static constexpr char THINGS_ATTR_NVS_NAMESPACE[] = "radio:tbattrs";
static nvs_handle_t things_attr_nvs_handle;

static std::string things_token;
static std::mutex things_telemetry_mutex;
static std::vector<void (*)(void)> things_telemetry_generators;

static constexpr TickType_t CONNECT_TIMEOUT = pdMS_TO_TICKS(5000);

static std::mutex things_attribute_mutex;
typedef std::variant<std::string, float, long long, bool> things_attribute_cache_entry_t;

static void populate_cache_update(things_attribute_cache_entry_t *cache, things_attribute_t &update)
{
  switch (cache->index())
  {
  case 0:
    update.type = THINGS_ATTRIBUTE_TYPE_STRING;
    update.value.string = std::get<std::string>(*cache).c_str();
    break;
  case 1:
    update.type = THINGS_ATTRIBUTE_TYPE_FLOAT;
    update.value.f = std::get<float>(*cache);
    break;
  case 2:
    update.type = THINGS_ATTRIBUTE_TYPE_INT;
    update.value.i = std::get<long long>(*cache);
    break;
  case 3:
    update.type = THINGS_ATTRIBUTE_TYPE_BOOL;
    update.value.b = std::get<bool>(*cache);
    break;
  default:
    update.type = THINGS_ATTRIBUTE_TYPE_UNSET;
    break;
  }
}

static std::map<std::string, things_attribute_cache_entry_t> things_attribute_cache;
static std::multimap<std::string, void (*)(const char *key, things_attribute_t *attr)> things_attribute_subscribers;

static struct
{
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

static void things_update_attribute(const char *key, JsonVariantConst value, bool cache_nvs)
{
  // Need to do 3 things: update things_attribute_cache, notify subscribers, and
  // (if caching requested) update NVS

  std::optional<things_attribute_cache_entry_t> cache;
  things_attribute_t update = {
      .type = THINGS_ATTRIBUTE_TYPE_UNSET,
      .value = {},
  };

  if (value.isNull())
  {
    cache.reset();
  }
  else if (value.is<long long>())
  {
    cache = value.as<long long>();
  }
  else if (value.is<float>())
  {
    cache = value.as<float>();
  }
  else if (value.is<bool>())
  {
    cache = value.as<bool>();
  }
  else if (value.is<const char *>())
  {
    cache = value.as<const char *>();
  }
  else
  {
    ESP_LOGE(RADIO_TAG, "Unsupported attribute type for %s", key);
    return;
  }

  bool changed = true;

  {
    std::lock_guard<std::mutex> lock(things_attribute_mutex);

    // Update cache
    if (cache.has_value())
    {
      changed = !things_attribute_cache.contains(key) || things_attribute_cache[key] != cache.value();
      things_attribute_cache[key] = cache.value();
    }
    else
    {
      changed = things_attribute_cache.contains(key);
      things_attribute_cache.erase(key);
    }

    // Notify subscribers
    if (cache.has_value())
    {
      populate_cache_update(&cache.value(), update);
    }

    auto range = things_attribute_subscribers.equal_range(key);
    for (auto it = range.first; it != range.second; ++it)
    {
      it->second(key, &update);
    }
  }

  // Update NVS
  esp_err_t err;
  if (cache_nvs && changed)
  {
    switch (update.type)
    {
    case THINGS_ATTRIBUTE_TYPE_UNSET:
      err = nvs_erase_key(things_attr_nvs_handle, key);
      break;
    case THINGS_ATTRIBUTE_TYPE_STRING:
      err = nvs_set_str(things_attr_nvs_handle, key, update.value.string);
      break;
    case THINGS_ATTRIBUTE_TYPE_FLOAT:
      // NVS doesn't support floats
      err = nvs_set_blob(things_attr_nvs_handle, key, &update.value.f, sizeof(update.value.f));
      break;
    case THINGS_ATTRIBUTE_TYPE_INT:
      err = nvs_set_i64(things_attr_nvs_handle, key, update.value.i);
      break;
    case THINGS_ATTRIBUTE_TYPE_BOOL:
      err = nvs_set_u8(things_attr_nvs_handle, key, update.value.b);
      break;
    default:
      ESP_LOGE(RADIO_TAG, "Unsupported attribute type for %s", key);
      return;
    }

    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to update attribute %s in NVS: %d (%s)", key, err, esp_err_to_name(err));
    }

    err = nvs_commit(things_attr_nvs_handle);
    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to commit NVS after updating attribute %s: %d (%s)", key, err, esp_err_to_name(err));
    }
  }
}

static esp_err_t things_preload_attributes(void)
{
  nvs_iterator_t iter = NULL;
  esp_err_t err = nvs_entry_find_in_handle(things_attr_nvs_handle, NVS_TYPE_ANY, &iter);
  if (err != ESP_ERR_NVS_NOT_FOUND)
  {
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to find attributes in NVS");
  }
  while (iter)
  {
    nvs_entry_info_t info;
    ESP_RETURN_ON_ERROR(nvs_entry_info(iter, &info), RADIO_TAG, "Failed to get attribute info from NVS");
    DynamicJsonDocument doc(4000 /* max string size */);
    switch (info.type)
    {
    case NVS_TYPE_U64:
    {
      uint64_t value;
      ESP_RETURN_ON_ERROR(nvs_get_u64(things_attr_nvs_handle, info.key, &value), RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      doc.set(value);
      break;
    }
    case NVS_TYPE_STR:
    {
      size_t length;
      ESP_RETURN_ON_ERROR(nvs_get_str(things_attr_nvs_handle, info.key, NULL, &length), RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      std::string value(length, '\0');
      ESP_RETURN_ON_ERROR(nvs_get_str(things_attr_nvs_handle, info.key, value.data(), &length), RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      doc.set(value);
      break;
    }
    case NVS_TYPE_BLOB:
    {
      float value;
      size_t length = sizeof(value);
      ESP_RETURN_ON_ERROR(nvs_get_blob(things_attr_nvs_handle, info.key, &value, &length), RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      doc.set(value);
      break;
    }
    case NVS_TYPE_U8:
    {
      uint8_t value;
      ESP_RETURN_ON_ERROR(nvs_get_u8(things_attr_nvs_handle, info.key, &value), RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      doc.set(!!value);
      break;
    }
    default:
      ESP_LOGE(RADIO_TAG, "Unsupported attribute type (%d) for %s", info.type, info.key);
      break;
    }

    things_update_attribute(info.key, doc.as<JsonVariantConst>(), false);

    err = nvs_entry_next(&iter);
    if (err != ESP_ERR_NVS_NOT_FOUND)
    {
      ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to find attributes in NVS");
    }
  }

  nvs_release_iterator(iter);

  return ESP_OK;
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

static void things_upload_coredump(RadioThingsBoard *conn, size_t core_size)
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
    ESP_LOGE(RADIO_TAG, "Failed to map coredump partition: %d (%s)", err, esp_err_to_name(err));
    goto cleanup;
  }

  mbedtls_base64_encode(NULL, 0, &base64_size, (const unsigned char *)core_dump_addr, core_size);
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

  conn->sendTelemetryData("coredump", base64_core_dump);

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

// TODO: if this callback is from an initial fetch of attributes and previously
// cached attributes are missing, we should delete them from the cache
static void things_attribute_callback(JsonObjectConst const &attrs)
{
  for (auto const &attr : attrs)
  {
    // If attributes have been deleted, their names show up as an array under
    // the "deleted" key
    if (attr.key() == "deleted")
    {
      if (!attr.value().is<JsonArrayConst>())
      {
        ESP_LOGE(RADIO_TAG, "Expected array for deleted attributes, got %s", attr.key().c_str());
        continue;
      }

      for (auto const &key : attr.value().as<JsonArrayConst>())
      {
        things_update_attribute(key.as<const char *>(), JsonVariantConst(), true);
      }

      continue;
    }

    things_update_attribute(attr.key().c_str(), attr.value(), true);
  }
}

static void things_task(void *arg)
{
  // First, try to pre-populate attributes from NVS - this doesn't require provisioning
  esp_err_t err = things_preload_attributes();
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to preload attributes: %d (%s)", err, esp_err_to_name(err));
  }

  // Otherwise block until we're provisioned
  xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED, pdFALSE, pdTRUE, portMAX_DELAY);

  // Main loop is around wifi connection. If we get disconnected from wifi, wait
  // until we reconnect and then reconnect to ThingsBoard too
  while (true)
  {
    int64_t wait = 4000 + esp_random() % 2000;
    xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    Espressif_MQTT_Client mqtt_client;
    mqtt_client.set_connect_callback([&self]()
                                     { xTaskNotifyGive(self); });
    mqtt_client.set_server_crt_bundle_attach(esp_crt_bundle_attach);
    std::shared_ptr<RadioThingsBoard> conn = std::make_shared<RadioThingsBoard>(mqtt_client, MAX_MESSAGE_SIZE);
    tb = conn;

    bool success = conn->connect(THINGSBOARD_SERVER, things_token.c_str(), THINGSBOARD_PORT);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to connect to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      conn->disconnect();
      continue;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t deadline = xTaskGetTickCount() + CONNECT_TIMEOUT;
    while (!conn->connected() && now < deadline)
    {
      xTaskNotifyWait(0, ULONG_MAX, NULL, deadline - now);
      now = xTaskGetTickCount();
    }

    if (!conn->connected())
    {
      ESP_LOGE(RADIO_TAG, "Failed to connect to ThingsBoard within timeout. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    ESP_LOGI(RADIO_TAG, "Connected to ThingsBoard");

    size_t core_addr, core_size;
    esp_err_t err = esp_core_dump_image_get(&core_addr, &core_size);
    if (err == ESP_OK)
    {
      ESP_LOGW(RADIO_TAG, "Core dump detected at 0x%08x, size %d bytes", core_addr, core_size);
      things_upload_coredump(conn.get(), core_size);
      err = esp_core_dump_image_erase();
      if (err != ESP_OK)
      {
        ESP_LOGE(RADIO_TAG, "Failed to erase core dump: %d (%s)", err, esp_err_to_name(err));
      }
    }
    else if (err != ESP_ERR_NOT_FOUND && err != ESP_ERR_INVALID_SIZE)
    {
      ESP_LOGE(RADIO_TAG, "Failed to get core dump: %d (%s)", err, esp_err_to_name(err));
    }

    // Note that the project name is generated from the top-level CMakeLists.txt
    // file; the version is automatically generated, most likely from git
    // describe - see
    // https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/api-guides/build-system.html#build-project-variables
    // for details
    const esp_app_desc_t *app = esp_app_get_description();

    success = conn->Firmware_Send_Info(app->project_name, app->version);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to send firmware info to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }
    success = conn->Firmware_Send_State("UPDATED");
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to send firmware state to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    Espressif_Updater updater;
    const OTA_Update_Callback ota_callback(
        things_progress_callback,
        things_updated_callback,
        app->project_name,
        app->version,
        &updater,
        FIRMWARE_FAILURE_RETRIES,
        FIRMWARE_PACKET_SIZE);

    // First subscribe to updates, in case there's not one already pending
    success = conn->Subscribe_Firmware_Update(ota_callback);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to subscribe to firmware updates from ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    // Then manually request an update, since Subscribe_Firmware_Update is
    // edge-triggered not level-triggered
    success = conn->Start_Firmware_Update(ota_callback);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to request an immediate firmware updates from ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    // Same with shared attributes - first subscribe, then request an update
    const Shared_Attribute_Callback attr_sub_callback(things_attribute_callback);
    success = conn->Shared_Attributes_Subscribe(attr_sub_callback);
    if (!success)
    {
      ESP_LOGE(RADIO_TAG, "Failed to subscribe to shared attributes from ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(things_attribute_mutex);
      if (!things_attribute_subscribers.empty())
      {
        auto attributes = std::views::keys(things_attribute_subscribers);
        std::vector<const char *> attribute_names;
        attribute_names.reserve(attributes.size());
        for (auto const &attr : attributes)
        {
          attribute_names.push_back(attr.c_str());
        }
        const Attribute_Request_Callback attr_req_callback(things_attribute_callback, attribute_names.cbegin(), attribute_names.cend());
        success = conn->Shared_Attributes_Request(attr_req_callback);
        if (!success)
        {
          ESP_LOGE(RADIO_TAG, "Failed to request shared attributes from ThingsBoard. Waiting and trying again...");
          vTaskDelay(pdMS_TO_TICKS(wait));
          continue;
        }
      }
    }

    while (true)
    {
      {
        std::lock_guard<std::mutex> lock(things_telemetry_mutex);
        for (auto const &generator : things_telemetry_generators)
        {
          generator();
        }
      }

      // If the wifi disconnect bit remains unset after 30 seconds, send telemetry again
      int64_t wait = 27500 + esp_random() % 5000;
      EventBits_t bits = xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(wait));
      if (bits & RADIO_EVENT_GROUP_WIFI_DISCONNECTED)
      {
        break;
      }
    }
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

  err = nvs_open(THINGS_ATTR_NVS_NAMESPACE, NVS_READWRITE, &things_attr_nvs_handle);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to open NVS handle for attributes: %s", esp_err_to_name(err));

  // TODO: Can we get log streaming to perform well enough?

  xTaskCreate(things_task, "things_task", 4096, NULL, 10, NULL);

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
    ESP_LOGE(RADIO_TAG, "Failed to check device token: %d (%s)", err, esp_err_to_name(err));
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
    ESP_LOGE(RADIO_TAG, "Failed to erase device token: %d (%s)", err, esp_err_to_name(err));
    return err;
  }

  return ESP_OK;
}

extern "C" bool things_send_telemetry_string(char const *const key, char const *const value)
{
  auto conn = tb.lock();
  if (!conn || !conn->connected())
  {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

extern "C" bool things_send_telemetry_float(char const *const key, float value)
{
  auto conn = tb.lock();
  if (!conn || !conn->connected())
  {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

extern "C" bool things_send_telemetry_int(char const *const key, long long value)
{
  auto conn = tb.lock();
  if (!conn || !conn->connected())
  {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

extern "C" bool things_send_telemetry_bool(char const *const key, bool value)
{
  auto conn = tb.lock();
  if (!conn || !conn->connected())
  {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

extern "C" void things_register_telemetry_generator(void (*generator)(void))
{
  std::lock_guard<std::mutex> lock(things_telemetry_mutex);
  things_telemetry_generators.push_back(generator);
}

extern "C" esp_err_t things_subscribe_attribute(const char *key, void (*callback)(const char *key, things_attribute_t *attr))
{
  ESP_RETURN_ON_FALSE(key != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG, "Key must not be NULL");
  ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG, "Callback must not be NULL");
  // This is a requirement to cache the attribute in NVS
  ESP_RETURN_ON_FALSE(strlen(key) <= 15, ESP_ERR_INVALID_ARG, RADIO_TAG, "Key must be less than 15 characters");

  std::lock_guard<std::mutex> lock(things_attribute_mutex);

  bool need_fetch = !things_attribute_subscribers.contains(key);
  things_attribute_subscribers.emplace(key, callback);

  if (need_fetch)
  {
    auto conn = tb.lock();
    if (conn && conn->connected())
    {
      std::vector<const char *> keys{key};
      Attribute_Request_Callback callback(things_attribute_callback, keys.cbegin(), keys.cend());
      conn->Shared_Attributes_Request(callback);
    }
  }

  // Now that we've issued the request, we can initialize the subscriber from
  // our local cache, since the request callback won't fire until we release the
  // lock
  things_attribute_t update = {
      .type = THINGS_ATTRIBUTE_TYPE_UNSET,
      .value = {},
  };
  auto const &it = things_attribute_cache.find(key);
  if (it != things_attribute_cache.end())
  {
    switch (it->second.index())
    {
    case 0:
      update.type = THINGS_ATTRIBUTE_TYPE_STRING;
      update.value.string = std::get<std::string>(it->second).c_str();
      break;
    case 1:
      update.type = THINGS_ATTRIBUTE_TYPE_FLOAT;
      update.value.f = std::get<float>(it->second);
      break;
    case 2:
      update.type = THINGS_ATTRIBUTE_TYPE_INT;
      update.value.i = std::get<long long>(it->second);
      break;
    case 3:
      update.type = THINGS_ATTRIBUTE_TYPE_BOOL;
      update.value.b = std::get<bool>(it->second);
      break;
    default:
      ESP_LOGE(RADIO_TAG, "Unsupported attribute type for %s", key);
      return ESP_ERR_NOT_SUPPORTED;
    }
  }
  callback(key, &update);

  return ESP_OK;
}
