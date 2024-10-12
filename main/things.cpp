#include "nvs_flash.h"

#include "../config.h"
#include "main.h"
#include "things.h"

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <set>
#include <string>
#include <variant>
#include <vector>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_core_dump.h"
#include "esp_crt_bundle.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_random.h"

#include "mbedtls/base64.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <Espressif_MQTT_Client.h>
#include <Espressif_Updater.h>
#include <ThingsBoard.h>

#ifndef RADIO_THINGSBOARD_SERVER
#error "RADIO_THINGSBOARD_SERVER is not defined"
#else
static_assert(strlen(RADIO_THINGSBOARD_SERVER) > 0,
              "RADIO_THINGSBOARD_SERVER is empty (make sure config.h exists "
              "and is populated)");
#endif

class ESPLogger {
public:
  template <typename... Args>
  static int printfln(char const *const format, Args const &...args) {
    int const size = Helper::detectSize(format, args...);
    char formatted[size] = {};
    int const written = snprintf(formatted, size, format, args...);
    ESP_LOGI("ThingsBoard", "%s", formatted);
    return written;
  }

  static int println(char const *const message) {
    return printfln("%s", message);
  }
};

using RadioThingsBoard = ThingsBoardSized<ESPLogger>;

static constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
static constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;
static constexpr uint16_t MAX_MESSAGE_SIZE = 256U;

static constexpr char THINGSBOARD_SERVER[] = RADIO_THINGSBOARD_SERVER;
static constexpr uint16_t THINGSBOARD_PORT = 8883U;

static std::weak_ptr<RadioThingsBoard> tb;

static constexpr char THINGS_NVS_NAMESPACE[] = "radio:things";
static constexpr char THINGS_NVS_TOKEN_KEY[] = "devicetoken";
static nvs_handle_t things_nvs_handle;

static std::string things_token;
static std::mutex things_telemetry_mutex;
static std::vector<things_telemetry_generator_t> things_telemetry_generators;
static EventGroupHandle_t things_force_telemetry_event_group = nullptr;
static constexpr size_t event_group_max_bits = 24;
static things_telemetry_generator_t
    things_force_telemetry_generators[event_group_max_bits] = {};
static size_t things_force_telemetry_next_bit = 0;

static constexpr TickType_t CONNECT_TIMEOUT = pdMS_TO_TICKS(5000);
static constexpr TickType_t HEALTHCHECK_TIMEOUT = pdMS_TO_TICKS(2000);

template <class... Ts> struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

static constexpr char THINGS_ATTR_NVS_NAMESPACE[] = "radio:tbattrs";

typedef std::variant<std::monostate, std::string, float, long long, bool>
    things_attribute_cache_entry_t;

static nvs_handle_t things_attr_nvs_handle;

static std::mutex things_attribute_mutex;
static std::map<std::string, things_attribute_cache_entry_t>
    things_attribute_cache;
static std::multimap<std::string, things_attribute_callback_t>
    things_attribute_subscribers;

// Must be called while holding things_attribute_mutex
static void update_attr_cache(std::string key,
                              things_attribute_cache_entry_t value) {
  things_attribute_cache[key] = value;
}

// Must be called while holding things_attribute_mutex
static void update_attr_nvs(std::string key,
                            things_attribute_cache_entry_t value) {
  esp_err_t err = std::visit(
      overloaded{
          [&](std::monostate &) {
            return nvs_erase_key(things_attr_nvs_handle, key.c_str());
          },
          [&](std::string &v) {
            return nvs_set_str(things_attr_nvs_handle, key.c_str(), v.c_str());
          },
          [&](float &v) {
            // NVS doesn't support floats
            return nvs_set_blob(things_attr_nvs_handle, key.c_str(), &v,
                                sizeof(v));
          },
          [&](long long &v) {
            return nvs_set_i64(things_attr_nvs_handle, key.c_str(), v);
          },
          [&](bool &v) {
            return nvs_set_u8(things_attr_nvs_handle, key.c_str(), v);
          },
      },
      value);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGE(RADIO_TAG, "Failed to update attribute %s in NVS: %d (%s)",
             key.c_str(), err, esp_err_to_name(err));
  }

  err = nvs_commit(things_attr_nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG,
             "Failed to commit NVS after updating attribute %s: %d (%s)",
             key.c_str(), err, esp_err_to_name(err));
  }
}

// Must be called while holding things_attribute_mutex
static void update_attr_subscribers(std::string key,
                                    things_attribute_cache_entry_t value) {
  things_attribute_t update = {};
  std::visit(
      overloaded{
          [&](std::monostate &) { update.type = THINGS_ATTRIBUTE_TYPE_UNSET; },
          [&](std::string &v) {
            update.type = THINGS_ATTRIBUTE_TYPE_STRING;
            update.value.string = v.c_str();
          },
          [&](float &v) {
            update.type = THINGS_ATTRIBUTE_TYPE_FLOAT;
            update.value.f = v;
          },
          [&](long long &v) {
            update.type = THINGS_ATTRIBUTE_TYPE_INT;
            update.value.i = v;
          },
          [&](bool &v) {
            update.type = THINGS_ATTRIBUTE_TYPE_BOOL;
            update.value.b = v;
          },
      },
      value);

  auto range = things_attribute_subscribers.equal_range(key);
  for (auto it = range.first; it != range.second; ++it) {
    it->second(key.c_str(), &update);
  }
}

static esp_err_t things_attribute_cache_init() {
  std::lock_guard<std::mutex> lock(things_attribute_mutex);

  esp_err_t err = nvs_open(THINGS_ATTR_NVS_NAMESPACE, NVS_READWRITE,
                           &things_attr_nvs_handle);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to open NVS handle for attributes: %s",
                      esp_err_to_name(err));

  nvs_iterator_t iter = NULL;
  err = nvs_entry_find_in_handle(things_attr_nvs_handle, NVS_TYPE_ANY, &iter);
  if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to find attributes in NVS");
  }
  while (iter) {
    nvs_entry_info_t info;
    ESP_RETURN_ON_ERROR(nvs_entry_info(iter, &info), RADIO_TAG,
                        "Failed to get attribute info from NVS");
    things_attribute_cache_entry_t value;
    switch (info.type) {
    case NVS_TYPE_I64: {
      int64_t tmp;
      ESP_RETURN_ON_ERROR(nvs_get_i64(things_attr_nvs_handle, info.key, &tmp),
                          RADIO_TAG, "Failed to get attribute %s from NVS",
                          info.key);
      value.emplace<long long>(tmp);
      break;
    }
    case NVS_TYPE_STR: {
      size_t length;
      ESP_RETURN_ON_ERROR(
          nvs_get_str(things_attr_nvs_handle, info.key, NULL, &length),
          RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      std::string tmp(length, '\0');
      ESP_RETURN_ON_ERROR(
          nvs_get_str(things_attr_nvs_handle, info.key, tmp.data(), &length),
          RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      value.emplace<std::string>(tmp);
      break;
    }
    case NVS_TYPE_BLOB: {
      float tmp;
      size_t length = sizeof(tmp);
      ESP_RETURN_ON_ERROR(
          nvs_get_blob(things_attr_nvs_handle, info.key, &tmp, &length),
          RADIO_TAG, "Failed to get attribute %s from NVS", info.key);
      value.emplace<float>(tmp);
      break;
    }
    case NVS_TYPE_U8: {
      uint8_t tmp;
      ESP_RETURN_ON_ERROR(nvs_get_u8(things_attr_nvs_handle, info.key, &tmp),
                          RADIO_TAG, "Failed to get attribute %s from NVS",
                          info.key);
      value.emplace<bool>(tmp);
      break;
    }
    default:
      ESP_LOGE(RADIO_TAG, "Unsupported attribute type (%d) for %s", info.type,
               info.key);
      break;
    }

    update_attr_cache(info.key, value);
    // Obviously don't update NVS with the value we just read from NVS
    update_attr_subscribers(info.key, value);

    err = nvs_entry_next(&iter);
    if (err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to find attributes in NVS");
    }
  }

  nvs_release_iterator(iter);
  return ESP_OK;
}

static struct {
  configRUN_TIME_COUNTER_TYPE run_time_counter;
  configRUN_TIME_COUNTER_TYPE cpu0_idle_counter;
  configRUN_TIME_COUNTER_TYPE cpu1_idle_counter;
} things_last_cpu_usage = {};

static void things_telemetry_generator() {
  // Report basic metrics
  things_send_telemetry_int("uptime", esp_timer_get_time() / (1000 * 1000));

  configRUN_TIME_COUNTER_TYPE run_time_counter = esp_timer_get_time();
  configRUN_TIME_COUNTER_TYPE cpu0_idle_counter =
      ulTaskGetIdleRunTimeCounterForCore(0);
  configRUN_TIME_COUNTER_TYPE cpu1_idle_counter =
      ulTaskGetIdleRunTimeCounterForCore(1);

  if (things_last_cpu_usage.run_time_counter != 0 &&
      run_time_counter != things_last_cpu_usage.run_time_counter) {
    configRUN_TIME_COUNTER_TYPE run_time_delta =
        run_time_counter - things_last_cpu_usage.run_time_counter;
    configRUN_TIME_COUNTER_TYPE cpu0_idle_delta =
        cpu0_idle_counter - things_last_cpu_usage.cpu0_idle_counter;
    configRUN_TIME_COUNTER_TYPE cpu1_idle_delta =
        cpu1_idle_counter - things_last_cpu_usage.cpu1_idle_counter;

    float cpu0_usage = std::min(
        100.0f, std::max(0.0f, 100.0f * (run_time_delta - cpu0_idle_delta) /
                                   run_time_delta));
    float cpu1_usage = std::min(
        100.0f, std::max(0.0f, 100.0f * (run_time_delta - cpu1_idle_delta) /
                                   run_time_delta));

    things_send_telemetry_float("cpu0_usage_percent", cpu0_usage);
    things_send_telemetry_float("cpu1_usage_percent", cpu1_usage);
  }

  things_last_cpu_usage.run_time_counter = run_time_counter;
  things_last_cpu_usage.cpu0_idle_counter = cpu0_idle_counter;
  things_last_cpu_usage.cpu1_idle_counter = cpu1_idle_counter;

  things_send_telemetry_int("task_count", uxTaskGetNumberOfTasks());

  things_send_telemetry_int("psram_free",
                            heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  things_send_telemetry_int("dram_free",
                            heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  things_send_telemetry_int("psram_total",
                            heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
  things_send_telemetry_int("dram_total",
                            heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
  things_send_telemetry_int("psram_lwm",
                            heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
  things_send_telemetry_int(
      "dram_lwm", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}

static void things_progress_callback(const size_t &currentChunk,
                                     const size_t &totalChuncks) {
  ESP_LOGV(RADIO_TAG, "Firmware update progress %d/%d chunks (%.2f%%)",
           currentChunk, totalChuncks,
           static_cast<float>(100 * currentChunk) / totalChuncks);
}

static void things_updated_callback(const bool &success) {
  if (!success) {
    ESP_LOGE(RADIO_TAG, "Failed to update firmware");
    abort();
  }

  // TODO: setup r/w lock to allow blocking updates
  ESP_LOGI(RADIO_TAG, "Successfully updated firmware; restarting now");
  esp_restart();
}

static void things_upload_coredump(RadioThingsBoard *conn, size_t core_size) {
  const esp_partition_t *partition;
  const void *core_dump_addr = NULL;
  esp_partition_mmap_handle_t handle;
  esp_err_t err = ESP_OK;
  size_t base64_size;
  char *base64_core_dump = NULL;
  int ret;

  partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
  if (partition == NULL) {
    ESP_LOGE(RADIO_TAG, "Failed to find coredump partition");
    return;
  }

  err = esp_partition_mmap(partition, 0, core_size, ESP_PARTITION_MMAP_DATA,
                           &core_dump_addr, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to map coredump partition: %d (%s)", err,
             esp_err_to_name(err));
    goto cleanup;
  }

  mbedtls_base64_encode(NULL, 0, &base64_size,
                        (const unsigned char *)core_dump_addr, core_size);
  base64_core_dump = (char *)malloc(base64_size);
  if (base64_core_dump == NULL) {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for base64 coredump");
    goto cleanup;
  }

  ret = mbedtls_base64_encode((unsigned char *)base64_core_dump, base64_size,
                              &base64_size,
                              (const unsigned char *)core_dump_addr, core_size);
  if (ret != 0) {
    ESP_LOGE(RADIO_TAG, "Failed to base64 encode coredump: %d", ret);
    goto cleanup;
  }

  conn->setBufferSize(UINT16_MAX);
  conn->sendTelemetryData("coredump", base64_core_dump);
  conn->setBufferSize(MAX_MESSAGE_SIZE);

  ESP_LOGI(RADIO_TAG, "Uploading coredump to ThingsBoard");

cleanup:
  if (base64_core_dump != NULL) {
    free(base64_core_dump);
  }

  if (core_dump_addr != NULL) {
    esp_partition_munmap(handle);
  }
}

static void things_attribute_callback(JsonObjectConst const &attrs,
                                      bool initial_fetch = false) {
  std::lock_guard<std::mutex> lock(things_attribute_mutex);

  // Copy the keys currently in the cache so we can delete the missing ones
  std::set<std::string> cached_keys;
  for (auto const &[key, _] : things_attribute_cache) {
    cached_keys.insert(key);
  }

  for (auto const &attr : attrs) {
    // If attributes have been deleted, their names show up as an array under
    // the "deleted" key
    if (attr.key() == "deleted") {
      if (!attr.value().is<JsonArrayConst>()) {
        ESP_LOGE(RADIO_TAG, "Expected array for deleted attributes, got %s",
                 attr.key().c_str());
        continue;
      }

      for (auto const &json_key : attr.value().as<JsonArrayConst>()) {
        std::string key = json_key.as<std::string>();
        update_attr_cache(key, std::monostate{});
        update_attr_nvs(key, std::monostate{});
        update_attr_subscribers(key, std::monostate{});

        cached_keys.erase(key);
      }

      continue;
    }

    std::string key;
    key.assign(attr.key().c_str(), attr.key().size());
    things_attribute_cache_entry_t value;
    if (attr.value().is<std::string>()) {
      value.emplace<std::string>(attr.value().as<std::string>());
    } else if (attr.value().is<float>()) {
      value.emplace<float>(attr.value().as<float>());
    } else if (attr.value().is<long long>()) {
      value.emplace<long long>(attr.value().as<long long>());
    } else if (attr.value().is<bool>()) {
      value.emplace<bool>(attr.value().as<bool>());
    } else {
      ESP_LOGE(RADIO_TAG, "Unsupported attribute type for %s", key.c_str());
      continue;
    }

    update_attr_cache(key, value);
    update_attr_nvs(key, value);
    update_attr_subscribers(key, value);

    cached_keys.erase(key);
  }

  if (initial_fetch) {
    for (auto const &key : cached_keys) {
      update_attr_cache(key, std::monostate{});
      update_attr_nvs(key, std::monostate{});
      update_attr_subscribers(key, std::monostate{});
    }
  }
}

static bool things_healthcheck(std::shared_ptr<RadioThingsBoard> conn) {
  if (!conn->connected()) {
    ESP_LOGE(RADIO_TAG, "Disconnected from ThingsBoard");
    return false;
  }

  // We don't want to wait forever for a response, so it's possible that this
  // function will return before the callback is invoked. Give the callback a
  // weak pointer to our task handle so it can notify us when it's done
  std::shared_ptr<TaskHandle_t> self =
      std::make_shared<TaskHandle_t>(xTaskGetCurrentTaskHandle());
  std::weak_ptr<TaskHandle_t> weak_self = self;

  StaticJsonDocument<JSON_OBJECT_SIZE(1)> doc;
  JsonArray array = doc.to<JsonArray>();
  uint64_t start = esp_timer_get_time() / 1000;
  array.add(start);
  auto callback = RPC_Request_Callback(
      "ping", &array, [&, weak_self](JsonVariantConst const &repl) {
        uint64_t server_time = repl["pong"].as<uint64_t>();
        uint64_t end = esp_timer_get_time() / 1000;
        ESP_LOGV(RADIO_TAG,
                 "ThingsBoard healthcheck successful: %" PRIu64
                 "ms (server time: %" PRIu64 "ms)",
                 end - start, server_time);

        if (auto self = weak_self.lock()) {
          xTaskNotify(*self, 0, eNoAction);
        }
      });
  bool success = conn->RPC_Request(callback);
  if (!success) {
    ESP_LOGE(RADIO_TAG, "Failed to send healthcheck request to ThingsBoard");
    return false;
  }

  // Wait for the callback to notify us
  return pdTRUE == xTaskNotifyWait(0, ULONG_MAX, NULL, HEALTHCHECK_TIMEOUT);
}

static void things_task(void *arg) {
  // First, try to pre-populate attributes from NVS - this doesn't require
  // provisioning
  esp_err_t err = things_attribute_cache_init();
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to preload attributes: %d (%s)", err,
             esp_err_to_name(err));
  }

  // Otherwise block until we're provisioned
  xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED,
                      pdFALSE, pdTRUE, portMAX_DELAY);

  // Main loop is around wifi connection. If we get disconnected from wifi, wait
  // until we reconnect and then reconnect to ThingsBoard too
  //
  // TODO: count how many times we loop without successfully connecting, and at
  // some point force a wifi reconnect
  while (true) {
  loop:
    int64_t wait = 4000 + esp_random() % 2000;
    xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    Espressif_MQTT_Client mqtt_client;
    mqtt_client.set_connect_callback([&self]() { xTaskNotifyGive(self); });
    mqtt_client.set_server_crt_bundle_attach(esp_crt_bundle_attach);
    std::shared_ptr<RadioThingsBoard> conn =
        std::make_shared<RadioThingsBoard>(mqtt_client, MAX_MESSAGE_SIZE);
    tb = conn;

    bool success = conn->connect(THINGSBOARD_SERVER, things_token.c_str(),
                                 THINGSBOARD_PORT);
    if (!success) {
      ESP_LOGE(RADIO_TAG,
               "Failed to connect to ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      conn->disconnect();
      continue;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t deadline = xTaskGetTickCount() + CONNECT_TIMEOUT;
    while (!conn->connected() && now < deadline) {
      xTaskNotifyWait(0, ULONG_MAX, NULL, deadline - now);
      now = xTaskGetTickCount();
    }

    if (!conn->connected()) {
      ESP_LOGE(RADIO_TAG, "Failed to connect to ThingsBoard within timeout. "
                          "Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    ESP_LOGI(RADIO_TAG, "Connected to ThingsBoard");

    size_t core_addr, core_size;
    esp_err_t err = esp_core_dump_image_get(&core_addr, &core_size);
    if (err == ESP_OK) {
      ESP_LOGW(RADIO_TAG, "Core dump detected at 0x%08x, size %d bytes",
               core_addr, core_size);
      things_upload_coredump(conn.get(), core_size);
      err = esp_core_dump_image_erase();
      if (err != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "Failed to erase core dump: %d (%s)", err,
                 esp_err_to_name(err));
      }
    } else if (err != ESP_ERR_NOT_FOUND && err != ESP_ERR_INVALID_SIZE) {
      ESP_LOGE(RADIO_TAG, "Failed to get core dump: %d (%s)", err,
               esp_err_to_name(err));
    }

    // Note that the project name is generated from the top-level CMakeLists.txt
    // file; the version is automatically generated, most likely from git
    // describe - see
    // https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/api-guides/build-system.html#build-project-variables
    // for details
    const esp_app_desc_t *app = esp_app_get_description();

    success = conn->Firmware_Send_Info(app->project_name, app->version);
    if (!success) {
      ESP_LOGE(RADIO_TAG, "Failed to send firmware info to ThingsBoard. "
                          "Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }
    success = conn->Firmware_Send_State("UPDATED");
    if (!success) {
      ESP_LOGE(RADIO_TAG, "Failed to send firmware state to ThingsBoard. "
                          "Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    Espressif_Updater updater;
    const OTA_Update_Callback ota_callback(
        things_progress_callback, things_updated_callback, app->project_name,
        app->version, &updater, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);

    // First subscribe to updates, in case there's not one already pending
    success = conn->Subscribe_Firmware_Update(ota_callback);
    if (!success) {
      ESP_LOGE(RADIO_TAG, "Failed to subscribe to firmware updates from "
                          "ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    // Then manually request an update, since Subscribe_Firmware_Update is
    // edge-triggered not level-triggered
    success = conn->Start_Firmware_Update(ota_callback);
    if (!success) {
      ESP_LOGE(RADIO_TAG, "Failed to request an immediate firmware updates "
                          "from ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    // Same with shared attributes - first subscribe, then request an update
    const Shared_Attribute_Callback attr_sub_callback(
        [](JsonObjectConst const &attrs) { things_attribute_callback(attrs); });
    success = conn->Shared_Attributes_Subscribe(attr_sub_callback);
    if (!success) {
      ESP_LOGE(RADIO_TAG, "Failed to subscribe to shared attributes from "
                          "ThingsBoard. Waiting and trying again...");
      vTaskDelay(pdMS_TO_TICKS(wait));
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(things_attribute_mutex);
      if (!things_attribute_subscribers.empty()) {
        auto attributes = std::views::keys(things_attribute_subscribers);
        std::vector<const char *> attribute_names;
        attribute_names.reserve(attributes.size());
        for (auto const &attr : attributes) {
          attribute_names.push_back(attr.c_str());
        }
        const Attribute_Request_Callback attr_req_callback(
            [](JsonObjectConst const &attrs) {
              things_attribute_callback(attrs, true);
            },
            attribute_names.cbegin(), attribute_names.cend());
        success = conn->Shared_Attributes_Request(attr_req_callback);
        if (!success) {
          ESP_LOGE(RADIO_TAG, "Failed to request shared attributes from "
                              "ThingsBoard. Waiting and trying again...");
          vTaskDelay(pdMS_TO_TICKS(wait));
          continue;
        }
      }
    }

    while (things_healthcheck(conn)) {
      {
        std::lock_guard<std::mutex> lock(things_telemetry_mutex);
        for (auto const &generator : things_telemetry_generators) {
          generator();
        }
      }

      // If the wifi disconnect bit remains unset after 30 seconds, send
      // telemetry again
      int64_t wait = 27500 + esp_random() % 5000;
      TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(wait);
      while (xTaskGetTickCount() < deadline) {
        EventBits_t bits = xEventGroupWaitBits(
            radio_event_group,
            RADIO_EVENT_GROUP_WIFI_DISCONNECTED |
                RADIO_EVENT_GROUP_THINGS_FORCE_TELEMETRY,
            pdFALSE, pdFALSE, deadline - xTaskGetTickCount());

        if (bits & RADIO_EVENT_GROUP_WIFI_DISCONNECTED) {
          goto loop;
        }

        if (bits & RADIO_EVENT_GROUP_THINGS_FORCE_TELEMETRY) {
          xEventGroupClearBits(radio_event_group,
                               RADIO_EVENT_GROUP_THINGS_FORCE_TELEMETRY);
          EventBits_t force_bits = xEventGroupWaitBits(
              things_force_telemetry_event_group,
              BIT(things_force_telemetry_next_bit) - 1, pdTRUE, pdFALSE, 0);
          if (force_bits == 0) {
            continue;
          }

          std::lock_guard<std::mutex> lock(things_telemetry_mutex);
          for (size_t i = 0; i < sizeof(EventBits_t) * 8; i++) {
            if (force_bits & BIT(i)) {
              things_force_telemetry_generators[i]();
            }
          }
        }
      }
    }
  }

  // (Unreachable)
  vTaskDelete(NULL);
}

esp_err_t things_init() {
  if (things_nvs_handle != 0) {
    ESP_LOGE(RADIO_TAG, "ThingsBoard already initialized");
    return ESP_ERR_INVALID_STATE;
  }

  things_force_telemetry_event_group = xEventGroupCreate();

  esp_err_t err =
      nvs_open(THINGS_NVS_NAMESPACE, NVS_READWRITE, &things_nvs_handle);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to open NVS handle: %s",
                      esp_err_to_name(err));

  // TODO: Can we get log streaming to perform well enough?

  xTaskCreate(things_task, "things_task", 4096, NULL, 10, NULL);

  size_t length;
  err = nvs_get_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, NULL, &length);
  if (err == ESP_OK) {
    things_token.resize(length);
    err = nvs_get_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY,
                      things_token.data(), &length);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to get device token: %s",
                        esp_err_to_name(err));
    xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED);
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGE(RADIO_TAG, "Failed to check device token: %d (%s)", err,
             esp_err_to_name(err));
    return err;
  }

  things_register_telemetry_generator(things_telemetry_generator, NULL);

  return ESP_OK;
}

esp_err_t things_provision(const char *token) {
  if (things_nvs_handle == 0) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = nvs_set_str(things_nvs_handle, THINGS_NVS_TOKEN_KEY, token);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to set device token: %s",
                      esp_err_to_name(err));
  things_token = token;
  xEventGroupSetBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED);

  return ESP_OK;
}

bool things_send_telemetry_string(char const *const key,
                                  char const *const value) {
  auto conn = tb.lock();
  if (!conn || !conn->connected()) {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

bool things_send_telemetry_float(char const *const key, float value) {
  auto conn = tb.lock();
  if (!conn || !conn->connected()) {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

bool things_send_telemetry_int(char const *const key, long long value) {
  auto conn = tb.lock();
  if (!conn || !conn->connected()) {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

bool things_send_telemetry_bool(char const *const key, bool value) {
  auto conn = tb.lock();
  if (!conn || !conn->connected()) {
    return false;
  }
  return conn->sendTelemetryData(key, value);
}

esp_err_t
things_register_telemetry_generator(things_telemetry_generator_t generator,
                                    size_t *index) {
  ESP_RETURN_ON_FALSE(generator != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Generator must not be NULL");
  ESP_RETURN_ON_FALSE(
      index == NULL || things_force_telemetry_next_bit < event_group_max_bits,
      ESP_ERR_NO_MEM, RADIO_TAG, "No more forceable telemetry generators");
  std::lock_guard<std::mutex> lock(things_telemetry_mutex);
  things_telemetry_generators.push_back(generator);

  if (index != NULL) {
    *index = things_force_telemetry_next_bit;
    things_force_telemetry_generators[things_force_telemetry_next_bit++] =
        generator;
  }

  return ESP_OK;
}

void things_force_telemetry(size_t index) {
  if (things_force_telemetry_event_group == nullptr) {
    return;
  }
  xEventGroupSetBits(things_force_telemetry_event_group, BIT(index));
  xEventGroupSetBits(radio_event_group,
                     RADIO_EVENT_GROUP_THINGS_FORCE_TELEMETRY);
}

esp_err_t things_subscribe_attribute(const char *key,
                                     things_attribute_callback_t callback) {
  ESP_RETURN_ON_FALSE(key != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Key must not be NULL");
  ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Callback must not be NULL");
  // This is a requirement to cache the attribute in NVS
  ESP_RETURN_ON_FALSE(strlen(key) <= 15, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Key must be less than 15 characters");

  std::lock_guard<std::mutex> lock(things_attribute_mutex);

  bool need_fetch = !things_attribute_subscribers.contains(key);
  things_attribute_subscribers.emplace(key, callback);

  if (need_fetch) {
    auto conn = tb.lock();
    if (conn && conn->connected()) {
      std::vector<const char *> keys{key};
      Attribute_Request_Callback callback(
          [](const JsonObjectConst &attrs) {
            things_attribute_callback(attrs);
          },
          keys.cbegin(), keys.cend());
      conn->Shared_Attributes_Request(callback);
    }
  }

  // Now that we've issued the request, we can initialize the subscriber from
  // our local cache, since the request callback won't fire until we release the
  // lock
  update_attr_subscribers(key, things_attribute_cache[key]);

  return ESP_OK;
}
