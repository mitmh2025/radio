#include "station_numbers.h"
#include "led.h"
#include "main.h"
#include "mixer.h"
#include "playback.h"
#include "playback_queue.h"
#include "things.h"
#include "tuner.h"

#include <math.h>
#include <string.h>
#include <sys/time.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"

static frequency_handle_t freq_handle;

static bool entuned = false;

static const char *numbers_file = "diligent-spy/numbers.opus";
static int64_t numbers_duration = 0;

static void playback_cb(bool active) {
  if (entuned && !active) {
    playback_queue_entry_t cfg = {
        .tuned = true,
    };
    strncpy(cfg.path, numbers_file, sizeof(cfg.path));
    playback_queue_add(&cfg);
  }
}

static void stop_playback() { playback_queue_drain(); }

static void start_playback() {
  if (playback_queue_active()) {
    return;
  }

  struct timeval now = {};
  gettimeofday(&now, NULL);
  uint64_t ts = now.tv_sec * 1000000 + now.tv_usec;
  if (ts < 1000000ULL * 60 * 60 * 24 * 365) {
    // clock is not accurate yet
    ts = ((uint64_t)esp_random() << 32) | esp_random();
  }

  int skip = 0;
  if (numbers_duration > 0) {
    skip = ((ts * 48000) / 1000000) % numbers_duration;
  }

  playback_queue_entry_t cfg = {
      .tuned = true,
      .skip_samples = skip,
  };
  strncpy(cfg.path, numbers_file, sizeof(cfg.path));
  playback_queue_add(&cfg);
}

static void entune(void *arg) {
  entuned = true;
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 64, 0));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_NONE));
  start_playback();
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_subscribe(playback_cb));
}

static void detune(void *arg) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_queue_unsubscribe(playback_cb));
  stop_playback();
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
  entuned = false;
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
    ESP_LOGE(RADIO_TAG, "Invalid attribute type %d for attribute %s",
             value->type, key);
    break;
  }

  if (enabled) {
    ESP_ERROR_CHECK(tuner_enable_frequency(freq_handle));
  } else {
    ESP_ERROR_CHECK(tuner_disable_frequency(freq_handle));
  }
}

esp_err_t station_numbers_init() {
  esp_err_t err = playback_duration(numbers_file, &numbers_duration);
  if (err != ESP_OK) {
    ESP_LOGW(
        RADIO_TAG,
        "Failed to get rickroll duration, will skip faking infinite loop: %d",
        err);
  }

  frequency_config_t config = {
      .frequency = 37.0f * M_PI / 20.0f,
      .enabled = false,
      .entune = entune,
      .detune = detune,
  };
  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register numbers frequency");

  ESP_RETURN_ON_ERROR(things_subscribe_attribute("en_numbers", things_cb),
                      RADIO_TAG, "Failed to subscribe to numbers attribute");

  return ESP_OK;
}
