#include "station_pi.h"
#include "main.h"
#include "tuner.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"

esp_err_t station_pi_init() {
  frequency_config_t config = {
      .frequency = M_PI,
      .enabled = false,
  };

  // We don't need to make config changes so we don't need to save the handle
  frequency_handle_t handle;
  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &handle), RADIO_TAG,
                      "Failed to register pi frequency");

  return ESP_OK;
}