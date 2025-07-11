#include "station_pi_activation.h"
#include "accelerometer.h"
#include "audio_output.h"
#include "audio_volume.h"
#include "main.h"
#include "playback.h"
#include "station_pi.h"
#include "tas2505.h"
#include "things.h"
#include "tuner.h"

#include <stdatomic.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const accelerometer_pulse_cfg_t pulse_cfg = {
    .threshold_x = 0,
    // This threshold (1g) was chosen to be fairly generous to avoid false
    // negatives. Only listen on the y axis because that gives us better
    // polarity detection for knock vs. setting down the radio
    .threshold_y = 16,
    .threshold_z = 0,
    .timelimit = 24, // 30ms
    // The pulse latency setting doesn't apply if you're reading and clearing
    // the interrupt status. Once that's done, new pulses seem to come in, so we
    // need to debounce separately
    .latency = 1,
};

#define PULSE_DEBOUNCE_US (50 * 1000)
#define PULSE_MIN_SEPARATION_US (300 * 1000)
#define PULSE_MAX_SEPARATION_US (800 * 1000)

static bool configured = false;
static bool attr_enabled = false;
static bool local_enabled = true;

static int64_t last_knocks[8] = {};
static int last_knock_index = 0;
static int64_t last_pulse = 0;

static int64_t rhythm_min_period = PULSE_MIN_SEPARATION_US;
static int64_t rhythm_max_period = PULSE_MAX_SEPARATION_US;
static int rhythm_count = 0;

static esp_timer_handle_t rhythm_timer = NULL;
static TaskHandle_t activate_task_handle = NULL;

static int64_t activate_last_trigger = 0;

static size_t telemetry_index;

static void telemetry_generator() {
  char knock_buffer[170];
  // Report the most recent pulse first
  snprintf(knock_buffer, sizeof(knock_buffer),
           "[%" PRIu64 ", %" PRIu64 ", %" PRIu64 ", %" PRIu64 ", %" PRIu64
           ", %" PRIu64 ", %" PRIu64 ", %" PRIu64 "]",
           last_knocks[(last_knock_index + 8) % 8],
           last_knocks[(last_knock_index + 7) % 8],
           last_knocks[(last_knock_index + 6) % 8],
           last_knocks[(last_knock_index + 5) % 8],
           last_knocks[(last_knock_index + 4) % 8],
           last_knocks[(last_knock_index + 3) % 8],
           last_knocks[(last_knock_index + 2) % 8],
           last_knocks[(last_knock_index + 1) % 8]);
  things_send_telemetry_string("pi_activation_knocks", knock_buffer);

  things_send_telemetry_int("pi_activation_last_trigger",
                            activate_last_trigger);
}

static void activate_task(void *arg) {
  activate_last_trigger = esp_timer_get_time();

  esp_err_t err = tuner_suspend();
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to suspend tuner: %d", err);
    goto cleanup;
  }

  audio_output_suspend();
  tas2505_set_output(TAS2505_OUTPUT_SPEAKER);
  audio_volume_set_floor(180);

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

  things_force_telemetry(telemetry_index);

cleanup:
  audio_volume_clear_floor();
  audio_output_resume();
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

static void pulse_callback(accelerometer_pulse_axis_t axis, void *arg) {

  int64_t now = esp_timer_get_time();

  // Debounce the pulse
  if (now - last_pulse < PULSE_DEBOUNCE_US) {
    ESP_LOGD(RADIO_TAG, "Pulse on axis %s (debounced)",
             accelerometer_pulse_axis_to_string(axis));
    last_pulse = now;
    return;
  }

  if (axis == ACCELEROMETER_PULSE_AXIS_Y_NEG) {
    ESP_LOGD(RADIO_TAG, "Pulse on axis %s (ignored)",
             accelerometer_pulse_axis_to_string(axis));
    last_pulse = now;
    return;
  }

  int64_t last_knock = last_knocks[last_knock_index];
  int64_t earliest_time = last_knock + rhythm_min_period;
  int64_t latest_time = last_knock + rhythm_max_period;
  if (latest_time < now || now < earliest_time) {
    // This is out of sync, but if it was supposed to be a 3rd pulse, it might
    // actually be a 2nd pulse establishing a rhythm
    esp_timer_stop(rhythm_timer);
    if (rhythm_count == 2 && (now - last_knock) > PULSE_MIN_SEPARATION_US &&
        (now - last_knock) < PULSE_MAX_SEPARATION_US) {
      rhythm_count = 1;
    } else {
      // Reset
      rhythm_max_period = PULSE_MAX_SEPARATION_US;
      rhythm_min_period = PULSE_MIN_SEPARATION_US;
      rhythm_count = 0;
    }
  }

  rhythm_count++;
  ESP_LOGD(RADIO_TAG, "Pulse on axis %s (count %d)",
           accelerometer_pulse_axis_to_string(axis), rhythm_count);

  switch (rhythm_count) {
  case 1:
    // Nothing to do on first pulse
    break;
  case 2: {
    // Use separation between two pulses to establish period
    int64_t period = now - last_knock;
    rhythm_min_period = (period * 4) / 5;
    rhythm_max_period = (period * 6) / 5;
    break;
  }
  case 3:
    // Set a timer to make sure we don't get any more pulses
    esp_timer_start_once(rhythm_timer, 1000000);
    break;
  default:
    // Too many pulses, cancel the timer (but let them keep pulsing)
    esp_timer_stop(rhythm_timer);
    break;
  }

  last_knock_index = (last_knock_index + 1) % 8;
  last_knocks[last_knock_index] = now;
  last_pulse = now;

  things_force_telemetry(telemetry_index);
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

static void enable_attr_callback(const char *key,
                                 const things_attribute_t *value) {
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

  ESP_RETURN_ON_ERROR(
      things_register_telemetry_generator(
          telemetry_generator, "station_pi_activation", &telemetry_index),
      RADIO_TAG, "Failed to register telemetry generator");

  return ESP_OK;
}

esp_err_t station_pi_activation_enable(bool enable) {
  local_enabled = enable;
  return check_enable();
}
