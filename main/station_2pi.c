#include "station_2pi.h"
#include "bluetooth.h"
#include "main.h"
#include "mixer.h"
#include "tuner.h"
#include "webrtc_manager.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"

static void entune_two_pi(void *ctx) {
  bluetooth_set_mode(BLUETOOTH_MODE_DISABLED);
  webrtc_manager_entune();
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_COMFORT));
}

static void detune_two_pi(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
  webrtc_manager_detune();
  bluetooth_set_mode(BLUETOOTH_MODE_DEFAULT);
}

esp_err_t station_2pi_init() {
  frequency_config_t config = {
      .frequency = 2.0f * M_PI,
      .enabled = true,
      .entune = entune_two_pi,
      .detune = detune_two_pi,
      .ctx = NULL,
  };

  // We don't need to make config changes so we don't need to save the handle
  frequency_handle_t handle;
  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &handle), RADIO_TAG,
                      "Failed to register 2pi frequency");

  return ESP_OK;
}