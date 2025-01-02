#include "station_rickroll.h"
#include "bluetooth.h"
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
#include "esp_timer.h"

static frequency_handle_t freq_handle;

static bool entuned = false;
static uint16_t current_strongest_minor = 0;

static SemaphoreHandle_t interrupt_mutex = NULL;
static playback_handle_t interrupt_playback = NULL;
static esp_timer_handle_t interrupt_timer = NULL;

static const char *rickroll_file = "giant-switch/rickroll.opus";
static int64_t rickroll_duration = 0;

static void interrupt_task(void *arg) {
  playback_handle_t *handle = arg;

  xSemaphoreTake(interrupt_mutex, portMAX_DELAY);
  playback_handle_t current_handle = *handle;
  xSemaphoreGive(interrupt_mutex);

  esp_err_t err = playback_wait_for_completion(current_handle);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to wait for completion: %d", err);
  }

  xSemaphoreTake(interrupt_mutex, portMAX_DELAY);
  playback_free(*handle);
  *handle = NULL;
  xSemaphoreGive(interrupt_mutex);

  vTaskDelete(NULL);
}

static void timer_cb(void *arg) {
  if (!entuned) {
    return;
  }

  char path[sizeof("giant-switch/interrupt-00000.opus")] = {0};
  snprintf(path, sizeof(path), "giant-switch/interrupt-%d.opus",
           current_strongest_minor);

  xSemaphoreTake(interrupt_mutex, portMAX_DELAY);
  esp_err_t err = playback_file(
      &(playback_cfg_t){
          .path = path,
          .tuned = true,
          .duck_others = true,
      },
      &interrupt_playback);
  xSemaphoreGive(interrupt_mutex);

  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to play interrupt: %d", err);
    return;
  }

  BaseType_t result = xTaskCreate(interrupt_task, "interrupt_playback", 4096,
                                  &interrupt_playback, 11, NULL);
  if (result != pdPASS) {
    ESP_LOGE(RADIO_TAG, "Failed to create interrupt playback task: %d", result);
  }

  esp_timer_start_once(interrupt_timer, 30000000);
}

static void playback_empty_cb() {
  if (entuned && current_strongest_minor) {
    playback_queue_entry_t cfg = {
        .tuned = true,
    };
    strncpy(cfg.path, rickroll_file, sizeof(cfg.path));
    playback_queue_add(&cfg);
  }
}

static void stop_playback() {
  esp_timer_stop(interrupt_timer);

  led_set_pixel(1, 64, 25, 0);

  xSemaphoreTake(interrupt_mutex, portMAX_DELAY);
  if (interrupt_playback) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(playback_stop(interrupt_playback));
  }
  xSemaphoreGive(interrupt_mutex);

  playback_queue_drain();
}

static void start_playback() {
  if (playback_queue_active()) {
    return;
  }

  led_set_pixel(1, 0, 64, 0);

  struct timeval now = {};
  gettimeofday(&now, NULL);
  uint64_t ts = now.tv_sec * 1000000 + now.tv_usec;
  if (ts < 1000000ULL * 60 * 60 * 24 * 365) {
    // clock is not accurate yet
    ts = ((uint64_t)esp_random() << 32) | esp_random();
  }

  int skip = 0;
  if (rickroll_duration > 0) {
    skip = ((ts * 48000) / 1000000) % rickroll_duration;
  }

  playback_queue_entry_t cfg = {
      .tuned = true,
      .skip_samples = skip,
  };
  strncpy(cfg.path, rickroll_file, sizeof(cfg.path));
  playback_queue_add(&cfg);

  uint64_t delay = 30000000 - (ts % 30000000);
  esp_timer_start_once(interrupt_timer, delay);
}

static void bluetooth_cb(bluetooth_beacon_t *newest,
                         bluetooth_beacon_t *strongest, void *arg) {
  uint16_t new_strongest_minor = 0;
  if (strongest) {
    new_strongest_minor = strongest->minor;
  }

  if (new_strongest_minor == current_strongest_minor) {
    return;
  }

  if (current_strongest_minor != 0) {
    if (entuned) {
      stop_playback();
    }
  }

  current_strongest_minor = new_strongest_minor;

  if (new_strongest_minor != 0) {
    if (entuned) {
      start_playback();
    }
  }
}

static void entune(void *arg) {
  entuned = true;
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_COMFORT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_AGGRESSIVE));
  if (current_strongest_minor != 0) {
    start_playback();
  } else {
    led_set_pixel(1, 64, 25, 0);
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      playback_queue_subscribe_empty(playback_empty_cb));
}

static void detune(void *arg) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      playback_queue_unsubscribe_empty(playback_empty_cb));
  stop_playback();
  led_set_pixel(1, 0, 0, 0);
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_DEFAULT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
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

esp_err_t station_rickroll_init() {
  interrupt_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(interrupt_mutex != NULL, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create interrupt mutex");

  esp_err_t err = playback_duration(rickroll_file, &rickroll_duration);
  if (err != ESP_OK) {
    ESP_LOGW(
        RADIO_TAG,
        "Failed to get rickroll duration, will skip faking infinite loop: %d",
        err);
  }

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "rickroll_timer",
                              .callback = timer_cb,
                          },
                          &interrupt_timer),
                      RADIO_TAG, "Failed to initialize interrupt timer");

  frequency_config_t config = {
      .frequency = 23.0f * M_PI / 20.0f,
      .enabled = false,
      .entune = entune,
      .detune = detune,
  };
  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register rickroll frequency");

  ESP_RETURN_ON_ERROR(things_subscribe_attribute("en_rickroll", things_cb),
                      RADIO_TAG, "Failed to subscribe to rickroll attribute");

  ESP_RETURN_ON_ERROR(
      bluetooth_subscribe_beacon(BLUETOOTH_MAJOR_RICKROLL, bluetooth_cb, NULL),
      RADIO_TAG, "Failed to subscribe to rickroll beacons");

  return ESP_OK;
}
