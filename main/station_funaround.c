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
static bool next_detected = false;
static bool tuned = false;

const int8_t threshold = -75;

static esp_err_t set_ratchet(uint16_t new_ratchet) {
  ESP_LOGI(RADIO_TAG, "Setting funaround ratchet to %d", new_ratchet);
  ratchet = new_ratchet;
  next_detected = false;
  ESP_ERROR_CHECK(nvs_set_u16(funaround_nvs_handle, "ratchet", ratchet));
  ESP_ERROR_CHECK(nvs_commit(funaround_nvs_handle));
  return ESP_OK;
}

static void play_current_beacon() {
  char path[sizeof("dimpled-star/waypoint00000.opus")];
  snprintf(path, sizeof(path), "dimpled-star/waypoint%" PRIu16 ".opus",
           ratchet);

  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_drain());
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_add(&(playback_cfg_t){
      .path = path,
      .tuned = true,
  }));
}

static void update(bool force_playback) {
  if (!tuned) {
    return;
  }

  bool play = force_playback;
  if (next_detected) {
    set_ratchet(ratchet + 1);
    play = true;
  }

  if (play) {
    play_current_beacon();
  }
}

static void IRAM_ATTR triangle_button_intr(void *ctx, bool state) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_pause_toggle());
}

static void IRAM_ATTR circle_button_intr(void *ctx, bool state) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_drain());
  play_current_beacon();
}

static void beacon_cb(bluetooth_beacon_t *newest, bluetooth_beacon_t *strongest,
                      void *arg) {
  if (newest && newest->minor == ratchet + 1) {
    bool new_detected = newest->rssi > threshold;
    if (new_detected != next_detected) {
      ESP_LOGD(RADIO_TAG, "%s funaround beacon %d",
               new_detected ? "Found" : "Lost", newest->minor);
      next_detected = new_detected;
    }
  }

  update(false);
}

static void entune_funaround(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_AGGRESSIVE));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
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
  tuned = true;
  update(true);
}

static void detune_funaround(void *ctx) {
  tuned = false;
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_CIRCLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_TRIANGLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_drain());
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
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
    ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_enable_pm_frequency(freq_handle));
  }
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

  return ESP_OK;
}

esp_err_t station_funaround_set_ratchet(uint16_t new_ratchet) {
  return set_ratchet(new_ratchet);
}