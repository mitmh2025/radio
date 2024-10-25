#include "station_pi_activation.h"
#include "accelerometer.h"
#include "main.h"
#include "playback.h"
#include "station_pi.h"
#include "things.h"
#include "tuner.h"

#include <stdatomic.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

// Configuration also used in station_pi; should probably be kept in sync
static const accelerometer_pulse_cfg_t pulse_cfg = {
    // 100Hz is fast enough to detect anything we care about and makes the math
    // easy
    .odr = ACCELEROMETER_DR_100HZ,
    .osm = ACCELEROMETER_OSM_NORMAL,
    // This threshold (1.375g, give or take) was chosen based on testing on a
    // development setup that is - notably - lacking the legs. We need to
    // re-test on a final device and decide how sensitive we want to be
    .threshold = 22,
    .timelimit = 6, // 30ms
    // The pulse latency setting doesn't apply if you're reading and clearing
    // the interrupt status. Once that's done, new pulses seem to come in, so we
    // need to debounce separately
    .latency = 1,
};

#define PULSE_DEBOUNCE_US (50 * 1000)
#define PULSE_MIN_SEPARATION_US (150 * 1000)
#define PULSE_MAX_SEPARATION_US (1500 * 1000)

static bool configured = false;
static bool attr_enabled = false;
static bool local_enabled = true;

static int64_t last_pulse = 0;

static int64_t rhythm_min_period = PULSE_MIN_SEPARATION_US;
static int64_t rhythm_max_period = PULSE_MAX_SEPARATION_US;
static int rhythm_count = 0;

static esp_timer_handle_t rhythm_timer = NULL;
static TaskHandle_t activate_task_handle = NULL;

static void activate_task(void *arg) {
  esp_err_t err = tuner_suspend();
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to suspend tuner: %d", err);
    goto cleanup;
  }

  playback_handle_t playback;
  err = playback_file(
      &(playback_cfg_t){
          .path = "practical-fighter/activation.opus",
          .tuned = true,
      },
      &playback);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to play activation file: %d", err);
    goto cleanup;
  }
  err = playback_wait_for_completion(playback);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to wait for completion: %d", err);
    goto cleanup;
  }

  playback_free(playback);

  ESP_LOGI(RADIO_TAG, "Activating station pi");
  err = station_pi_enable();
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to enable station pi: %d", err);
  }

  err = tuner_resume();
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to resume tuner: %d", err);
  }

cleanup:
  activate_task_handle = NULL;
  vTaskDelete(NULL);
}

static void timer_callback(void *arg) {
  ESP_LOGI(RADIO_TAG, "Knock rhythm triggered (period %" PRIu64 "us)",
           (rhythm_min_period + rhythm_max_period) / 2);

  if (!activate_task_handle) {
    if (pdPASS != xTaskCreate(activate_task, "pi_activate", 4096, NULL, 17,
                              &activate_task_handle)) {
      ESP_LOGE(RADIO_TAG, "Failed to create activation task");
    };
  }
}

static void pulse_callback(void *arg) {
  int64_t now = esp_timer_get_time();

  // Debounce the pulse
  if (now - last_pulse < PULSE_DEBOUNCE_US) {
    goto cleanup;
  }

  int64_t earliest_time = last_pulse + rhythm_min_period;
  int64_t latest_time = last_pulse + rhythm_max_period;
  if (latest_time < now || now < earliest_time) {
    // This is out of sync, but if it was supposed to be a 3rd pulse, it might
    // actually be a 2nd pulse establishing a rhythm
    if (rhythm_count == 2 && (now - last_pulse) < PULSE_MAX_SEPARATION_US) {
      rhythm_count = 1;
    } else {
      // Reset
      esp_timer_stop(rhythm_timer);
      rhythm_max_period = PULSE_MAX_SEPARATION_US;
      rhythm_min_period = PULSE_MIN_SEPARATION_US;
      rhythm_count = 0;
    }
  }

  rhythm_count++;
  switch (rhythm_count) {
  case 1:
    // Nothing to do on first pulse
    break;
  case 2: {
    // Use separation between two pulses to establish period
    int64_t period = now - last_pulse;
    rhythm_min_period = (period * 9) / 10;
    rhythm_max_period = (period * 11) / 10;
    break;
  }
  case 3:
    // Set a timer to make sure we don't get any more pulses
    esp_timer_start_once(rhythm_timer, rhythm_max_period * 2);
    break;
  default:
    // Too many pulses, cancel the timer (but let them keep pulsing)
    esp_timer_stop(rhythm_timer);
    break;
  }

cleanup:
  last_pulse = now;
}

static esp_err_t check_enable() {
  bool should_enable = attr_enabled && local_enabled;
  if (should_enable && !configured) {
    ESP_RETURN_ON_ERROR(
        accelerometer_subscribe_pulse(&pulse_cfg, pulse_callback, NULL),
        RADIO_TAG, "Failed to subscribe to pulse");
    configured = true;
  } else if (!should_enable && configured) {
    ESP_RETURN_ON_ERROR(accelerometer_unsubscribe_pulse(), RADIO_TAG,
                        "Failed to unsubscribe from pulse");
    configured = false;
  }

  return ESP_OK;
}

static void enable_attr_callback(const char *key, things_attribute_t *value) {
  bool enabled = false;
  switch (value->type) {
  case THINGS_ATTRIBUTE_TYPE_BOOL:
    enabled = value->value.b;
    break;
  case THINGS_ATTRIBUTE_TYPE_UNSET:
    enabled = false;
    break;
  default:
    ESP_LOGW(RADIO_TAG, "Unexpected attribute type %d for attribute %s",
             value->type, key);
    // Treat as false
    break;
  }

  attr_enabled = enabled;
  ESP_ERROR_CHECK_WITHOUT_ABORT(check_enable());
}

esp_err_t station_pi_activation_init() {
  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .callback = timer_callback,
                              .arg = NULL,
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "rhythm_timer",
                          },
                          &rhythm_timer),
                      RADIO_TAG, "Failed to create rhythm timer");

  ESP_RETURN_ON_ERROR(
      things_subscribe_attribute("en_knocks", enable_attr_callback), RADIO_TAG,
      "Failed to subscribe to en_knocks attribute");
  return ESP_OK;
}

esp_err_t station_pi_activation_enable(bool enable) {
  local_enabled = enable;
  return check_enable();
}
