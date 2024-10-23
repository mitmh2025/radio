#include "nvs.h"

#include "accelerometer.h"
#include "adc.h"
#include "bounds.h"
#include "main.h"
#include "mixer.h"
#include "station_pi.h"
#include "station_pi_activation.h"
#include "tone_generator.h"
#include "tuner.h"

#include "board.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#define STATION_PI_NVS_NAMESPACE "radio:pi"
static nvs_handle_t pi_nvs_handle;

static frequency_handle_t freq_handle;
static bool entuned = false;

static bounds_handle_t light_bounds;
static const uint16_t light_threshold = 500;
static uint16_t light_smooth = 0xffff;
static bool light_triggered = false;
static tone_generator_t light_tone = NULL;

// Copied from station_pi_activation
static const accelerometer_pulse_cfg_t knock_cfg = {
    .odr = ACCELEROMETER_DR_100HZ,
    .osm = ACCELEROMETER_OSM_NORMAL,
    .threshold = 22,
    .timelimit = 6,
    .latency = 1,
};
#define PULSE_DEBOUNCE_US (50 * 1000)
static int64_t knock_last_time = 0;
static esp_timer_handle_t knock_timer = NULL;
static tone_generator_t knock_tone = NULL;

static void light_start_tone() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0xc000,
          .release_time = 100000,
          .frequency = FREQUENCY_C_5,
      },
      &light_tone));
}

static void light_stop_tone() {
  tone_generator_release(light_tone);
  light_tone = NULL;
}

static void knock_stop_tone(void *arg) {
  esp_timer_stop(knock_timer);
  if (knock_tone) {
    tone_generator_release(knock_tone);
    knock_tone = NULL;
  }
}

static void knock_start_tone(void *arg) {
  int64_t now = esp_timer_get_time();

  // Debounce the pulse
  if (now - knock_last_time < PULSE_DEBOUNCE_US) {
    goto cleanup;
  }

  if (esp_timer_is_active(knock_timer) || knock_tone) {
    knock_stop_tone(NULL);
  }

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(knock_timer, 600000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = true,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0xc000,
          .release_time = 100000,
          .frequency = FREQUENCY_D_5,
      },
      &knock_tone));

cleanup:
  knock_last_time = now;
}

static void update_state() {
  bool should_play_light = light_triggered;

  if (should_play_light && !light_tone) {
    light_start_tone();
    knock_stop_tone(NULL);
  } else if (!should_play_light && light_tone) {
    light_stop_tone();
  }

  if (light_tone) {
    tone_generator_set_entuned(light_tone, entuned);
  }
  if (knock_tone) {
    tone_generator_set_entuned(knock_tone, entuned);
  }
}

static void light_adc_cb(void *ctx, adc_digi_output_data_t *result) {
  uint16_t value = result->type2.data;

  if (light_smooth == 0xffff) {
    light_smooth = value;
    return;
  }
  light_smooth = light_smooth - (light_smooth >> 3) + (value >> 3);

  bounds_update(light_bounds, light_smooth);

  light_triggered =
      bounds_get_max(light_bounds) - light_threshold > light_smooth;

  update_state();
}

static void entune(void *ctx) {
  station_pi_activation_enable(false);
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      accelerometer_subscribe_pulse(&knock_cfg, knock_start_tone, NULL));

  entuned = true;
  update_state();
}

static void detune(void *ctx) {
  entuned = false;
  update_state();
  ESP_ERROR_CHECK_WITHOUT_ABORT(accelerometer_unsubscribe_pulse());
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
  station_pi_activation_enable(true);
}

esp_err_t station_pi_init() {
  ESP_RETURN_ON_ERROR(
      nvs_open(STATION_PI_NVS_NAMESPACE, NVS_READWRITE, &pi_nvs_handle),
      RADIO_TAG, "Failed to open NVS handle for station pi");

  ESP_RETURN_ON_ERROR(bounds_init(
                          &(bounds_config_t){
                              .buckets = 30,
                              .interval = 1000000,
                          },
                          &light_bounds),
                      RADIO_TAG, "Failed to initialize light bounds");
  ESP_RETURN_ON_ERROR(adc_subscribe(
                          &(adc_digi_pattern_config_t){
                              .atten = ADC_ATTEN_DB_12,
                              .channel = LIGHT_ADC_CHANNEL,
                              .unit = ADC_UNIT_1,
                              .bit_width = 12,
                          },
                          light_adc_cb, NULL),
                      RADIO_TAG, "Failed to subscribe to light ADC");

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "knock_tone",
                              .callback = knock_stop_tone,
                          },
                          &knock_timer),
                      RADIO_TAG, "Failed to initialize knock timer");

  uint8_t enabled = 0;
  esp_err_t ret = nvs_get_u8(pi_nvs_handle, "enabled", &enabled);
  if (ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get enabled from NVS");
  }

  frequency_config_t config = {
      .frequency = M_PI,
      .enabled = enabled != 0,
      .entune = entune,
      .detune = detune,
  };

  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register pi frequency");

  return ESP_OK;
}

esp_err_t station_pi_enable() {
  ESP_RETURN_ON_ERROR(nvs_set_u8(pi_nvs_handle, "enabled", 1), RADIO_TAG,
                      "Failed to set enabled in NVS");

  ESP_RETURN_ON_ERROR(tuner_enable_pm_frequency(freq_handle), RADIO_TAG,
                      "Failed to enable pi frequency");

  return ESP_OK;
}