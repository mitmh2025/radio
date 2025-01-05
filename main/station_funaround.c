#include "nvs.h"

#include "bluetooth.h"
#include "debounce.h"
#include "led.h"
#include "main.h"
#include "mixer.h"
#include "playback_queue.h"
#include "station_funaround.h"
#include "things.h"
#include "tuner.h"

#include <math.h>

#include "board.h"

#include "esp_check.h"
#include "esp_log.h"

#define STATION_FUNAROUND_NVS_NAMESPACE "radio:fun"
static nvs_handle_t funaround_nvs_handle;

static frequency_handle_t freq_handle;
static uint16_t ratchet = 0;
static bool tuned = false;

static const int8_t threshold = -75;

static size_t telemetry_index = 0;

static void telemetry_generator() {
  things_send_telemetry_int("funaround_ratchet", ratchet);
}

static void set_led(bool active) {
  if (!tuned) {
    led_set_pixel(1, 0, 0, 0);
  }

  if (active) {
    led_set_pixel(1, 0, 64, 0);
  } else {
    led_set_pixel(1, 64, 25, 0);
  }
}

static esp_err_t set_ratchet(uint16_t new_ratchet) {
  ESP_LOGI(RADIO_TAG, "Setting funaround ratchet to %d", new_ratchet);
  ratchet = new_ratchet;
  ESP_ERROR_CHECK(nvs_set_u16(funaround_nvs_handle, "ratchet", ratchet));
  ESP_ERROR_CHECK(nvs_commit(funaround_nvs_handle));
  things_force_telemetry(telemetry_index);
  return ESP_OK;
}

static void play_current_beacon() {
  playback_queue_entry_t cfg = {
      .tuned = true,
  };
  snprintf(cfg.path, sizeof(cfg.path), "dimpled-star/waypoint%" PRIu16 ".opus",
           ratchet);

  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_drain());
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_add(&cfg));
}

static void playback_cb(bool active) {
  set_led(active);
  if (!active && ratchet == 0) {
    play_current_beacon();
  }
}

static void triangle_button_intr(void *ctx, bool state) {
  if (ratchet != 0) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_pause_toggle());
  }
}

static void circle_button_intr(void *ctx, bool state) {
  if (ratchet != 0) {
    play_current_beacon();
  }
}

static void beacon_cb(bluetooth_beacon_t *newest, bluetooth_beacon_t *strongest,
                      void *arg) {
  if (!tuned) {
    return;
  }

  if (newest && newest->minor == ratchet + 1 && newest->rssi > threshold) {
    ESP_LOGD(RADIO_TAG, "Found funaround beacon %d", newest->minor);
    set_ratchet(ratchet + 1);
    play_current_beacon();
  }
}

static void entune_funaround(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_AGGRESSIVE));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_COMFORT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&(gpio_config_t){
      .pin_bit_mask = BIT64(BUTTON_TRIANGLE_PIN) | BIT64(BUTTON_CIRCLE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  }));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      debounce_handler_add(BUTTON_TRIANGLE_PIN, GPIO_INTR_POSEDGE,
                           triangle_button_intr, NULL, pdMS_TO_TICKS(10)));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      debounce_handler_add(BUTTON_CIRCLE_PIN, GPIO_INTR_POSEDGE,
                           circle_button_intr, NULL, pdMS_TO_TICKS(10)));
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_subscribe(playback_cb));
  tuned = true;
  play_current_beacon();
}

static void detune_funaround(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
  tuned = false;
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_unsubscribe(playback_cb));
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_CIRCLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_TRIANGLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_drain());
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_DEFAULT));
}

static void things_cb(const char *key, const things_attribute_t *value) {
  bool enabled = false;

  switch (value->type) {
  case THINGS_ATTRIBUTE_TYPE_BOOL:
    enabled = value->value.b;
    break;
  case THINGS_ATTRIBUTE_TYPE_UNSET:
    break;
  default:
    ESP_LOGE(RADIO_TAG, "Invalid type for funaround attribute");
    return;
  }

  if (enabled) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_enable_frequency(freq_handle));
  } else {
    ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_disable_frequency(freq_handle));
  }
}

esp_err_t rpc_set_ratchet(const things_attribute_t *param) {
  long long new_ratchet;
  switch (param->type) {
  case THINGS_ATTRIBUTE_TYPE_INT:
    new_ratchet = param->value.i;
    break;
  case THINGS_ATTRIBUTE_TYPE_FLOAT:
    new_ratchet = (uint16_t)param->value.f;
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }

  if (new_ratchet > 0xffff) {
    return ESP_ERR_INVALID_ARG;
  }

  return set_ratchet(new_ratchet);
}

esp_err_t rpc_dec_ratchet(const things_attribute_t *param) {
  return set_ratchet(ratchet - 1);
}

esp_err_t rpc_inc_ratchet(const things_attribute_t *param) {
  return set_ratchet(ratchet + 1);
}

esp_err_t station_funaround_init() {
  ESP_RETURN_ON_ERROR(nvs_open(STATION_FUNAROUND_NVS_NAMESPACE, NVS_READWRITE,
                               &funaround_nvs_handle),
                      RADIO_TAG,
                      "Failed to open NVS handle for station funaround");

  esp_err_t err = nvs_get_u16(funaround_nvs_handle, "ratchet", &ratchet);
  if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                        "Failed to get ratchet from NVS for funaround");
  }

  frequency_config_t config = {
      .frequency = 17.0f * M_PI / 10.0f,
      .enabled = false,
      .entune = entune_funaround,
      .detune = detune_funaround,
      .ctx = NULL,
  };

  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register funaround frequency");

  ESP_RETURN_ON_ERROR(things_subscribe_attribute("en_funaround", things_cb),
                      RADIO_TAG, "Failed to subscribe to funaround attribute");

  ESP_RETURN_ON_ERROR(
      bluetooth_subscribe_beacon(BLUETOOTH_MAJOR_FUNAROUND, beacon_cb, NULL),
      RADIO_TAG, "Failed to subscribe to funaround beacons");

  ESP_RETURN_ON_ERROR(things_register_telemetry_generator(
                          telemetry_generator, "funaround", &telemetry_index),
                      RADIO_TAG, "Failed to register funaround telemetry");

  ESP_RETURN_ON_ERROR(
      things_register_rpc("funaround:set_ratchet", rpc_set_ratchet), RADIO_TAG,
      "Failed to register funaround:set_ratchet RPC");
  ESP_RETURN_ON_ERROR(
      things_register_rpc("funaround:dec_ratchet", rpc_dec_ratchet), RADIO_TAG,
      "Failed to register funaround:dec_ratchet RPC");
  ESP_RETURN_ON_ERROR(
      things_register_rpc("funaround:inc_ratchet", rpc_inc_ratchet), RADIO_TAG,
      "Failed to register funaround:inc_ratchet RPC");

  return ESP_OK;
}

esp_err_t station_funaround_set_ratchet(uint16_t new_ratchet) {
  return set_ratchet(new_ratchet);
}