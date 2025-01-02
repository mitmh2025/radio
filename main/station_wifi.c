#include "station_wifi.h"
#include "debounce.h"
#include "led.h"
#include "main.h"
#include "mixer.h"
#include "playback_queue.h"
#include "tuner.h"
#include "wifi.h"

#include <math.h>

#include "board.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t ap_mode_lock = NULL;
static esp_timer_handle_t ap_start_timer = NULL;

static void trigger_audio(void) {
  uint8_t mac[6];
  ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_get_ap_network_mac(mac));

  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_add(&(playback_queue_entry_t){
      .path = "wifi-config-first.opus",
      .tuned = true,
  }));

  for (int i = 0; i < 6; i++) {
    uint8_t octet = mac[(i / 2) + 3];
    uint8_t digit = (i % 2 == 0) ? (octet >> 4) : (octet & 0x0F);
    playback_queue_entry_t cfg = {
        .tuned = true,
    };
    snprintf(cfg.path, sizeof(cfg.path), "hex-%1x.opus", digit);
    ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_add(&cfg));
  }

  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_add(&(playback_queue_entry_t){
      .path = "wifi-config-second.opus",
      .tuned = true,
  }));
}

static void button_intr(void *ctx, bool state) {
  xSemaphoreTake(ap_mode_lock, portMAX_DELAY);
  if (playback_queue_active()) {
    playback_queue_drain();
  } else {
    trigger_audio();
  }
  xSemaphoreGive(ap_mode_lock);
}

static void ap_start_timer_cb(void *arg) {
  xSemaphoreTake(ap_mode_lock, portMAX_DELAY);
  ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_enable_ap());
  trigger_audio();
  xSemaphoreGive(ap_mode_lock);

  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&(gpio_config_t){
      .pin_bit_mask = BIT64(BUTTON_CIRCLE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  }));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      debounce_handler_add(BUTTON_CIRCLE_PIN, GPIO_INTR_POSEDGE, button_intr,
                           NULL, pdMS_TO_TICKS(10)));
}

static void entune_wifi(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_COMFORT));
  esp_timer_start_once(ap_start_timer, 500000);
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 64, 0));
}

static void detune_wifi(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
  esp_timer_stop(ap_start_timer);

  xSemaphoreTake(ap_mode_lock, portMAX_DELAY);
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_drain());
  ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_disable_ap());
  xSemaphoreGive(ap_mode_lock);

  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
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
