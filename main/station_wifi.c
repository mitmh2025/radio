#include "station_wifi.h"
#include "main.h"
#include "mixer.h"
#include "tuner.h"
#include "wifi.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t ap_mode_lock = NULL;
static esp_timer_handle_t ap_start_timer = NULL;

static void ap_start_timer_cb(void *arg) {
  xSemaphoreTake(ap_mode_lock, portMAX_DELAY);
  ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_enable_ap());
  xSemaphoreGive(ap_mode_lock);
}

static void entune_wifi(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
  esp_timer_start_once(ap_start_timer, 1000000);
  // TODO: audio cue
}

static void detune_wifi(void *ctx) {
  esp_timer_stop(ap_start_timer);

  xSemaphoreTake(ap_mode_lock, portMAX_DELAY);
  ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_disable_ap());
  xSemaphoreGive(ap_mode_lock);

  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
}

esp_err_t station_wifi_init() {
  ap_mode_lock = xSemaphoreCreateMutex();

  esp_err_t err = esp_timer_create(
      &(esp_timer_create_args_t){
          .callback = ap_start_timer_cb,
          .arg = NULL,
          .dispatch_method = ESP_TIMER_TASK,
          .name = "start_ap",
      },
      &ap_start_timer);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to create AP start timer: %d",
                      err);

  frequency_config_t config = {
      .frequency = 7.0f * M_PI / 5.0f,
      .enabled = true,
      .entune = entune_wifi,
      .detune = detune_wifi,
      .ctx = NULL,
  };

  // We don't need to make config changes so we don't need to save the handle
  frequency_handle_t handle;
  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &handle), RADIO_TAG,
                      "Failed to register wifi frequency");

  return ESP_OK;
}
