#include "webrtc_manager.h"
#include "main.h"
#include "mixer.h"
#include "things.h"
#include "webrtc.h"

#include <stdatomic.h>
#include <string.h>

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

static SemaphoreHandle_t webrtc_manager_lock = NULL;
static char *webrtc_current_url = NULL;
static atomic_uint_least32_t webrtc_buffer_duration = 0;

#define NOTIFY_URL_CHANGED BIT(1)
#define NOTIFY_STATE_CHANGED BIT(2)
#define NOTIFY_BUFFER_DURATION_CHANGED BIT(3)

static void whep_url_callback(const char *key, things_attribute_t *attr) {
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
}

static void on_buffer_duration(webrtc_connection_t conn, void *context,
                               uint32_t duration_samples) {
  TaskHandle_t task = (TaskHandle_t)context;
  atomic_store(&webrtc_buffer_duration, duration_samples);
  xTaskNotify(task, NOTIFY_BUFFER_DURATION_CHANGED, eSetBits);
}

static void webrtc_loop() {
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
    return;
  }

  char *url = strdup(webrtc_current_url);
  ulTaskNotifyValueClear(NULL, NOTIFY_URL_CHANGED);
  xSemaphoreGive(webrtc_manager_lock);

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
  ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG,
                    "Failed to connect to WebRTC: %d (%s)", ret,
                    esp_err_to_name(ret));

  uint32_t target_buffer_duration = 48000;

  // Main control loop
  while (true) {
    uint32_t notification =
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(900 + esp_random() % 200));

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
        float rtt = webrtc_get_ice_rtt_ms(connection);
        target_buffer_duration = rtt * 4 * /* scale from ms to 48000 hz */ 48;
        ESP_LOGI(RADIO_TAG,
                 "Waiting to fill %" PRIu32 " samples (4 * %.2fms RTT)",
                 target_buffer_duration, rtt);

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

    if (notification & NOTIFY_BUFFER_DURATION_CHANGED) {
      uint32_t duration = atomic_load(&webrtc_buffer_duration);
      if (duration > target_buffer_duration && !channel) {
        int64_t end = esp_timer_get_time();
        ESP_LOGI(RADIO_TAG,
                 "Finished buffering audio (%" PRIu32 " samples, %" PRIu64
                 "ms since boot)",
                 duration, end / 1000);

        ret = mixer_play_audio(webrtc_read_audio_sample, connection, 48000, 16,
                               1, false, &channel);
        ESP_GOTO_ON_ERROR(ret, cleanup, RADIO_TAG,
                          "Failed to play audio: %d (%s)", ret,
                          esp_err_to_name(ret));
      }
    }

    // TODO: make sure connection is still healthy
  }

cleanup:
  if (channel) {
    mixer_stop_audio(channel);
  }

  if (connection) {
    webrtc_free_connection(connection);
  }
  free(url);

  if (ret != ESP_OK) {
    // Wait for a bit before trying again
    vTaskDelay(pdMS_TO_TICKS(2000 + esp_random() % 500));
  }
}

static void webrtc_manager_task(void *ctx) {
  things_subscribe_attribute("whep_url", whep_url_callback);
  while (true) {
    webrtc_loop();
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

  return ESP_OK;
}
