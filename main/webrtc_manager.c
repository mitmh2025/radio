#include "webrtc_manager.h"
#include "led.h"
#include "main.h"
#include "mixer.h"
#include "things.h"
#include "webrtc.h"
#include "wifi.h"

#include <stdatomic.h>
#include <string.h>
#include <sys/time.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static StaticTask_t webrtc_manager_task_buffer;
// 6KB stack size - kvswebrtc needs a lot of stack space
static EXT_RAM_BSS_ATTR StackType_t
    webrtc_manager_task_stack[6 * 1024 / sizeof(StackType_t)];
static TaskHandle_t webrtc_manager_task_handle = NULL;
static atomic_bool webrtc_entuned = false;
static atomic_int webrtc_latest_state = WEBRTC_CONNECTION_STATE_NONE;
static struct webrtc_rtp_stats webrtc_stats = {};

static SemaphoreHandle_t webrtc_manager_lock = NULL;
static char *webrtc_current_url = NULL;
static atomic_uint_least32_t webrtc_buffer_duration = 0;
static bool disconnect_failed = false;
static esp_timer_handle_t disconnect_watchdog = NULL;

static size_t telemetry_index = 0;

static void telemetry_generator() {
  things_send_telemetry_string(
      "webrtc_state",
      webrtc_connection_state_to_string(atomic_load(&webrtc_latest_state)));

  if (webrtc_stats.packets_received > 0) {
    things_send_telemetry_int("webrtc_packets_received",
                              webrtc_stats.packets_received);
    things_send_telemetry_int("webrtc_packets_failed_decryption",
                              webrtc_stats.packets_failed_decryption);
    things_send_telemetry_int("webrtc_packets_discarded",
                              webrtc_stats.packets_discarded);
    things_send_telemetry_int("webrtc_nack_count", webrtc_stats.nack_count);
    things_send_telemetry_int("webrtc_packets_lost", webrtc_stats.packets_lost);
    things_send_telemetry_float("webrtc_jitter_seconds",
                                webrtc_stats.jitter_seconds);
    things_send_telemetry_float("webrtc_jitter_buffer_delay_seconds",
                                webrtc_stats.jitter_buffer_delay_seconds);
    things_send_telemetry_int("webrtc_jitter_buffer_emitted_count",
                              webrtc_stats.jitter_buffer_emitted_count);
    things_send_telemetry_int("webrtc_bytes_received",
                              webrtc_stats.bytes_received);
  }
}

static void disconnect_watchdog_cb(void *arg) {
  disconnect_failed = true;
  if (atomic_load(&webrtc_entuned)) {
    esp_restart();
  }
}

#define NOTIFY_URL_CHANGED BIT(1)
#define NOTIFY_STATE_CHANGED BIT(2)
#define NOTIFY_BUFFER_DURATION_CHANGED BIT(3)
#define NOTIFY_ENTUNE_CHANGED BIT(4)

static void whep_url_callback(const char *key, const things_attribute_t *attr) {
  const char *url = NULL;
  switch (attr->type) {
  case THINGS_ATTRIBUTE_TYPE_UNSET:
    break;
  case THINGS_ATTRIBUTE_TYPE_STRING:
    url = attr->value.string;
    break;
  default:
    ESP_LOGE(RADIO_TAG, "Expected string for whep_url, got %d", attr->type);
    return;
  }

  xSemaphoreTake(webrtc_manager_lock, portMAX_DELAY);
  if (url == NULL && webrtc_current_url == NULL) {
    goto cleanup;
  } else if (url != NULL && webrtc_current_url != NULL &&
             strcmp(webrtc_current_url, url) == 0) {
    goto cleanup;
  }

  if (webrtc_current_url) {
    free(webrtc_current_url);
    webrtc_current_url = NULL;
  }

  if (url) {
    webrtc_current_url = strdup(url);
  }

  if (webrtc_manager_task_handle) {
    xTaskNotify(webrtc_manager_task_handle, NOTIFY_URL_CHANGED, eSetBits);
  }
cleanup:
  xSemaphoreGive(webrtc_manager_lock);
}

static void on_state_change(webrtc_connection_t conn, void *context,
                            webrtc_connection_state_t state) {
  // Notify the task that the state has changed. It can query for what the new
  // state is
  TaskHandle_t task = (TaskHandle_t)context;
  xTaskNotify(task, NOTIFY_STATE_CHANGED, eSetBits);

  atomic_store(&webrtc_latest_state, state);
  things_force_telemetry(telemetry_index);
}

static void on_buffer_duration(webrtc_connection_t conn, void *context,
                               uint32_t duration_samples) {
  TaskHandle_t task = (TaskHandle_t)context;
  atomic_store(&webrtc_buffer_duration, duration_samples);
  xTaskNotify(task, NOTIFY_BUFFER_DURATION_CHANGED, eSetBits);
}

static void set_led() {
  bool entuned = atomic_load(&webrtc_entuned);
  if (!entuned) {
    return;
  }

  if (!(xEventGroupGetBits(radio_event_group) &
        RADIO_EVENT_GROUP_WIFI_CONNECTED)) {
    led_set_pixel(1, 64, 0, 0);
    return;
  }

  webrtc_connection_state_t state = atomic_load(&webrtc_latest_state);
  switch (state) {
  case WEBRTC_CONNECTION_STATE_CONNECTED:
    led_set_pixel(1, 0, 64, 0);
    break;
  case WEBRTC_CONNECTION_STATE_NONE:
  case WEBRTC_CONNECTION_STATE_NEW:
  case WEBRTC_CONNECTION_STATE_CONNECTING:
    led_set_pixel(1, 64, 25, 0);
    break;
  case WEBRTC_CONNECTION_STATE_CLOSED:
  case WEBRTC_CONNECTION_STATE_DISCONNECTED:
  case WEBRTC_CONNECTION_STATE_FAILED:
    led_set_pixel(1, 64, 0, 64);
    break;
  default:
    led_set_pixel(1, 0, 0, 0);
    break;
  }
}

static bool webrtc_loop() {
  // Wait for wifi to connect
  xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED,
                      pdFALSE, pdTRUE, portMAX_DELAY);

  xSemaphoreTake(webrtc_manager_lock, portMAX_DELAY);
  if (!webrtc_current_url) {
    ESP_LOGW(RADIO_TAG, "No WebRTC WHEP URL has been configured. Set the "
                        "`whep_url` shared attribute in ThingsBoard.");
    xSemaphoreGive(webrtc_manager_lock);
    // Wait for the next notification before looping again
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return true;
  }

  char *url = strdup(webrtc_current_url);
  ulTaskNotifyValueClear(NULL, NOTIFY_URL_CHANGED);
  xSemaphoreGive(webrtc_manager_lock);

  // Allow 10 seconds to transition to connected
  int64_t connect_deadline = esp_timer_get_time() + 10000000ULL;
  int64_t connection_established_at = 0;
  ESP_LOGI(RADIO_TAG, "Connecting to WebRTC at %s", url);
  mixer_channel_t channel = NULL;
  webrtc_connection_t connection = NULL;
  webrtc_config_t cfg = {
      .whep_url = url,
      .state_change_callback = on_state_change,
      .buffer_duration_callback = on_buffer_duration,
      .user_data = xTaskGetCurrentTaskHandle(),
  };
  esp_err_t ret = webrtc_connect(&cfg, &connection);
  ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to connect to WebRTC: %d",
                    ret);

  uint32_t target_buffer_duration = 48000;

  // Main control loop
  while (true) {
    uint32_t notification =
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(900 + esp_random() % 200));

    int64_t now = esp_timer_get_time();

    set_led();
    if (notification & NOTIFY_URL_CHANGED) {
      // URL changed, reconnect
      ESP_LOGI(RADIO_TAG, "URL changed, reconnecting");
      goto cleanup;
    }

    if (notification & NOTIFY_STATE_CHANGED) {
      webrtc_connection_state_t state = webrtc_get_state(connection);
      ESP_LOGI(RADIO_TAG, "WebRTC state changed to %d (%s)", state,
               webrtc_connection_state_to_string(state));

      switch (state) {
      case WEBRTC_CONNECTION_STATE_CONNECTED: {
        if (connection_established_at == 0) {
          connection_established_at = now;
        }
        break;
      }
      case WEBRTC_CONNECTION_STATE_DISCONNECTED:
      case WEBRTC_CONNECTION_STATE_FAILED:
      case WEBRTC_CONNECTION_STATE_CLOSED:
        // Connection failed, reconnect
        ESP_LOGI(RADIO_TAG, "Connection failed, reconnecting");
        goto cleanup;
      default:
        break;
      }
    }

    uint32_t duration = atomic_load(&webrtc_buffer_duration);

    bool entuned = atomic_load(&webrtc_entuned);
    if (channel && !entuned) {
      ESP_LOGD(RADIO_TAG, "Detuning WebRTC");
      mixer_stop_audio(channel);
      channel = NULL;
    } else if (!channel && duration > target_buffer_duration && entuned) {
      int64_t end = now;
      ESP_LOGI(RADIO_TAG,
               "Starting webrtc playback (%" PRIu32
               " samples in buffer, %" PRIu64 "ms since boot)",
               duration, end / 1000);

      ret = mixer_play_audio(webrtc_read_audio_sample, connection, 48000, 16, 1,
                             false, &channel);
      ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG, "Failed to play audio: %d",
                        ret);
    }

    if (!channel) {
      // bleed down the buffer
      while (duration > target_buffer_duration) {
        webrtc_read_audio_sample(connection, NULL, 960, portMAX_DELAY);
        ESP_GOTO_ON_ERROR(webrtc_get_buffer_duration(connection, &duration),
                          cleanup, RADIO_TAG, "Failed to get buffer duration");
      }
    }

    if (connection_established_at == 0 && now > connect_deadline) {
      ESP_LOGW(RADIO_TAG,
               "Timed out waiting for WebRTC connection to establish");
      goto cleanup;
    }

    esp_err_t err = webrtc_get_rtp_stats(connection, &webrtc_stats);
    if (err != ESP_OK) {
      ESP_LOGW(RADIO_TAG, "Failed to get RTP stats: %d", err);
    }

    // Connection gets 10 seconds to get first packet, and then must not go more
    // than 5 seconds without a packet
    if (connection_established_at > 0 &&
        now - connection_established_at > 10000000) {
      // Check if we've received a packet recently
      struct timeval now_tv;
      int timeret = gettimeofday(&now_tv, NULL);
      if (timeret == 0) {
        int64_t now_ms = now / 1000;
        int64_t last_packet_ms = webrtc_stats.last_received_packet_timestamp_ms;
        int64_t last_packet_age = now_ms - last_packet_ms;
        if (last_packet_age > 5000) {
          ESP_LOGW(RADIO_TAG,
                   "No packets received in %" PRIu64 "ms, reconnecting",
                   last_packet_age);
          goto cleanup;
        }
      }
    }
  }

cleanup:
  if (channel) {
    mixer_stop_audio(channel);
  }

  if (connection) {
    esp_timer_start_once(disconnect_watchdog, pdMS_TO_TICKS(10000));
    webrtc_free_connection(connection);
    esp_timer_stop(disconnect_watchdog);
    atomic_store(&webrtc_buffer_duration, 0);
    atomic_store(&webrtc_latest_state, WEBRTC_CONNECTION_STATE_NONE);
  }
  free(url);

  memset(&webrtc_stats, 0, sizeof(webrtc_stats));

  if (ret != ESP_OK) {
    // Wait for a bit before trying again
    vTaskDelay(pdMS_TO_TICKS(2000 + esp_random() % 500));
  }

  return ret == ESP_OK && connection_established_at > 0;
}

static void webrtc_manager_task(void *ctx) {
  things_subscribe_attribute("whep_url", whep_url_callback);
  int attempt_count = 0;
  while (true) {
    attempt_count++;
    bool success = webrtc_loop();
    if (success) {
      attempt_count = 0;
    } else if (attempt_count > 3) {
      wifi_force_reconnect();
    }
  }
}

esp_err_t webrtc_manager_init() {
  webrtc_manager_lock = xSemaphoreCreateMutex();

  webrtc_manager_task_handle = xTaskCreateStaticPinnedToCore(
      webrtc_manager_task, "webrtc_manager",
      sizeof(webrtc_manager_task_stack) / sizeof(StackType_t), NULL, 10,
      webrtc_manager_task_stack, &webrtc_manager_task_buffer, 1);
  ESP_RETURN_ON_FALSE(webrtc_manager_task_handle != NULL, ESP_FAIL, RADIO_TAG,
                      "Failed to create WebRTC manager task");

  esp_err_t ret = things_register_telemetry_generator(
      telemetry_generator, "webrtc", &telemetry_index);
  ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to register telemetry generator");

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .callback = disconnect_watchdog_cb,
                              .name = "webrtc_disconnect_watchdog",
                          },
                          &disconnect_watchdog),
                      RADIO_TAG, "Failed to create disconnect watchdog");

  return ESP_OK;
}

void webrtc_manager_entune() {
  if (disconnect_failed) {
    // We can't easily recover from this because the webrtc task is locked up,
    // so just reboot
    esp_restart();
  }
  atomic_store(&webrtc_entuned, true);
  set_led();
  if (webrtc_manager_task_handle) {
    xTaskNotify(webrtc_manager_task_handle, NOTIFY_ENTUNE_CHANGED, eSetBits);
  }
}

void webrtc_manager_detune() {
  atomic_store(&webrtc_entuned, false);
  led_set_pixel(1, 0, 0, 0);
  if (webrtc_manager_task_handle) {
    xTaskNotify(webrtc_manager_task_handle, NOTIFY_ENTUNE_CHANGED, eSetBits);
  }
}
