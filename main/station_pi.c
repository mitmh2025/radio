#include "station_pi.h"
#include "main.h"
#include "station_pi_activation.h"
#include "tuner.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"

static frequency_handle_t freq_handle;

void entune(void *ctx) { station_pi_activation_enable(false); }

void detune(void *ctx) { station_pi_activation_enable(true); }

esp_err_t station_pi_init() {
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