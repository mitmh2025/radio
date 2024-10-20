#include "station_pi.h"
#include "adc.h"
#include "main.h"
#include "mixer.h"
#include "station_pi_activation.h"
#include "tone_generator.h"
#include "tuner.h"

#include "board.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"

static frequency_handle_t freq_handle;
static bool entuned = false;

static const uint16_t light_threshold = 1000;
static uint16_t light_smooth = 0xffff;
static bool light_triggered = false;
static tone_generator_t light_tone = NULL;

static void light_start_tone() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = 261.6256,
      },
      &light_tone));
}

static void light_stop_tone() {
  tone_generator_release(light_tone);
  light_tone = NULL;
}

static void update_state() {
  bool should_play_light = light_triggered;

  if (should_play_light && !light_tone) {
    light_start_tone();
  } else if (!should_play_light && light_tone) {
    light_stop_tone();
  }

  if (light_tone) {
    tone_generator_set_entuned(light_tone, entuned);
  }
}

static void light_adc_cb(void *ctx, adc_digi_output_data_t *result) {
  uint16_t value = result->type2.data;

  if (light_smooth == 0xffff) {
    light_smooth = value;
    return;
  }

  light_smooth = light_smooth - (light_smooth >> 3) + (value >> 3);
  light_triggered = light_smooth < light_threshold;

  update_state();
}

static void entune(void *ctx) {
  station_pi_activation_enable(false);
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
  entuned = true;
  update_state();
}

static void detune(void *ctx) {
  entuned = false;
  update_state();
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
  station_pi_activation_enable(true);
}

esp_err_t station_pi_init() {
  ESP_RETURN_ON_ERROR(adc_subscribe(
                          &(adc_digi_pattern_config_t){
                              .atten = ADC_ATTEN_DB_12,
                              .channel = LIGHT_ADC_CHANNEL,
                              .unit = ADC_UNIT_1,
                              .bit_width = 12,
                          },
                          light_adc_cb, NULL),
                      RADIO_TAG, "Failed to subscribe to light ADC");

  frequency_config_t config = {
      .frequency = M_PI,
      .enabled = false,
      .entune = entune,
      .detune = detune,
  };

  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register pi frequency");

  return ESP_OK;
}

esp_err_t station_pi_enable() {
  // TODO: this should persist to NVS
  ESP_RETURN_ON_ERROR(tuner_enable_pm_frequency(freq_handle), RADIO_TAG,
                      "Failed to enable pi frequency");

  return ESP_OK;
}