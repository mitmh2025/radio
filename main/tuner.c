#include "tuner.h"
#include "main.h"

#include "adc.h"
#include "bluetooth.h"
#include "board.h"
#include "debounce.h"
#include "fm.h"
#include "mixer.h"
#include "tas2505.h"
#include "things.h"

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <sys/queue.h>

#define FREQUENCY_PM_MIN ((float)(M_PI))
#define FREQUENCY_PM_MAX ((float)(2 * M_PI))
// Channel is roughly 4% of the band
#define FREQUENCY_PM_WIDTH ((float)(M_PI) / 25.0f)

#define FREQUENCY_FM_MIN (88.0f)
#define FREQUENCY_FM_MAX (108.0f)
// Channel is roughly 1.5% of the band - this means channels will be partially
// overlapping, so we need to handle that in our hysteresis logic
#define FREQUENCY_FM_WIDTH (0.3f)

#define FREQUENCY_FM_CHAN_MIN (88.1f)
#define FREQUENCY_FM_SEPARATION (0.2f)

struct frequency_handle {
  TAILQ_ENTRY(frequency_handle) next;

  float frequency;
  bool enabled;
  void (*entune)(void *ctx);
  void (*detune)(void *ctx);
  void *ctx;

  // These fields are computed based on the frequency band ranges and ADC
  // calibration range
  uint16_t frequency_low;
  uint16_t frequency_center;
  uint16_t frequency_high;
};

typedef enum {
  TUNER_MODE_PM,
  TUNER_MODE_FM,
} tuner_mode_t;
static atomic_bool tuner_suspended = false;
static tuner_mode_t desired_radio_mode = TUNER_MODE_PM;
static atomic_uint_least16_t current_raw_frequency = UINT16_MAX;
static tuner_mode_t current_radio_mode = TUNER_MODE_PM;
static struct frequency_handle *current_frequency = NULL;
static TaskHandle_t tuner_task_handle = NULL;
static radio_calibration_t *tuner_calibration = NULL;
static size_t telemetry_index = 0;

static SemaphoreHandle_t giant_switch_mutex = NULL;
static uint16_t current_giant_switch_minor = 0;

static inline bool gpio_to_tuner_mode(bool state) {
  return state ? TUNER_MODE_PM : TUNER_MODE_FM;
}

static void telemetry_generator() {
  uint16_t raw_frequency = atomic_load(&current_raw_frequency);
  float position =
      ((float)raw_frequency - tuner_calibration->frequency_min) /
      (tuner_calibration->frequency_max - tuner_calibration->frequency_min);

  const struct frequency_handle *frequency = current_frequency;
  things_send_telemetry_string(
      "tuner_mode", current_radio_mode == TUNER_MODE_PM ? "PM" : "FM");
  things_send_telemetry_float("tuner_frequency",
                              frequency ? frequency->frequency : 0.0f);
  things_send_telemetry_int("tuner_min", tuner_calibration->frequency_min);
  things_send_telemetry_int("tuner_max", tuner_calibration->frequency_max);
  things_send_telemetry_int("tuner_raw", raw_frequency);
  things_send_telemetry_float("tuner_position", position);
  things_send_telemetry_bool("tuner_entuned", frequency != NULL);
  things_send_telemetry_bool("tuner_suspended", atomic_load(&tuner_suspended));
}

static void entune_pm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_input(TAS2505_INPUT_DAC));
}

static void detune_pm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_NONE));
}

static void entune_fm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_AGGRESSIVE));
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_input(TAS2505_INPUT_BOTH));
  ESP_ERROR_CHECK_WITHOUT_ABORT(fm_enable());
}

static void detune_fm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(fm_disable());
  bluetooth_set_mode(BLUETOOTH_MODE_DEFAULT);
}

static struct {
  void (*entune)();
  void (*detune)();
} mode_tuners[] = {
    [TUNER_MODE_PM] = {.entune = entune_pm, .detune = detune_pm},
    [TUNER_MODE_FM] = {.entune = entune_fm, .detune = detune_fm},
};

TAILQ_HEAD(frequency_list, frequency_handle);
static struct frequency_list pm_frequencies =
    TAILQ_HEAD_INITIALIZER(pm_frequencies);
static struct frequency_list fm_frequencies =
    TAILQ_HEAD_INITIALIZER(fm_frequencies);

static void entune_fm_channel(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(fm_tune((uint16_t)(uintptr_t)ctx));
}

static void modulation_callback(void *user_data, bool state) {
  desired_radio_mode = gpio_to_tuner_mode(state);
  if (tuner_task_handle) {
    xTaskNotifyFromISR(tuner_task_handle, 0, eNoAction, NULL);
  }
}

static void adc_callback(void *user_data, adc_digi_output_data_t *result) {
  uint16_t raw_frequency = atomic_load(&current_raw_frequency);
  if (raw_frequency == UINT16_MAX) {
    raw_frequency = result->type2.data;
  } else {
    raw_frequency =
        raw_frequency - (raw_frequency >> 3) + (result->type2.data >> 3);
  }

  atomic_store(&current_raw_frequency, raw_frequency);
  if (tuner_task_handle) {
    xTaskNotifyGive(tuner_task_handle);
  }
}

static void tuner_task(void *ctx) {
  bool suspended = false;
  while (true) {
    if (atomic_load(&tuner_suspended) != suspended) {
      if (suspended) {
        // unsuspend: re-enable the band, but let the standard tuning sequence
        // re-tune the frequency
        ESP_LOGI(RADIO_TAG, "Resuming tuner task");
        if (mode_tuners[current_radio_mode].entune) {
          mode_tuners[current_radio_mode].entune();
        }
      } else {
        // suspend: detune the frequency and the band
        ESP_LOGI(RADIO_TAG, "Suspending tuner task");
        if (current_frequency && current_frequency->detune) {
          current_frequency->detune(current_frequency->ctx);
        }
        current_frequency = NULL;
        if (mode_tuners[current_radio_mode].detune) {
          mode_tuners[current_radio_mode].detune();
        }
      }
      suspended = !suspended;
    }

    if (suspended) {
      goto next;
    }

    uint16_t frequency_raw = atomic_load(&current_raw_frequency);

    if (desired_radio_mode == current_radio_mode && current_frequency &&
        current_frequency->enabled &&
        frequency_raw >= current_frequency->frequency_low &&
        frequency_raw < current_frequency->frequency_high) {
      goto next;
    }

    // Moving to a new frequency, so need to detune the old one
    if (current_frequency && current_frequency->detune) {
      current_frequency->detune(current_frequency->ctx);
      things_force_telemetry(telemetry_index);
    }

    // If we're moving to a different mode, we need to detune the old one and
    // entune the new one
    if (desired_radio_mode != current_radio_mode) {
      ESP_LOGI(RADIO_TAG, "Switching radio mode from %s to %s",
               current_radio_mode == TUNER_MODE_PM ? "PM" : "FM",
               desired_radio_mode == TUNER_MODE_PM ? "PM" : "FM");
      if (mode_tuners[current_radio_mode].detune) {
        mode_tuners[current_radio_mode].detune();
      }
      if (mode_tuners[desired_radio_mode].entune) {
        mode_tuners[desired_radio_mode].entune();
      }
      current_radio_mode = desired_radio_mode;
    }

    // Find the frequency we're currently tuned to
    const struct frequency_list *frequencies =
        current_radio_mode == TUNER_MODE_PM ? &pm_frequencies : &fm_frequencies;
    struct frequency_handle *new_frequency = NULL;
    TAILQ_FOREACH(new_frequency, frequencies, next) {
      if (new_frequency && new_frequency->enabled &&
          frequency_raw >= new_frequency->frequency_low &&
          frequency_raw < new_frequency->frequency_high) {
        break;
      }
    }

    current_frequency = new_frequency;
    if (current_frequency && current_frequency->entune) {
      ESP_LOGD(RADIO_TAG, "Tuning to frequency %0.3f",
               current_frequency->frequency);
      current_frequency->entune(current_frequency->ctx);
      things_force_telemetry(telemetry_index);
    }

  next:
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
  }
}

static void beacon_callback(bluetooth_beacon_t *newest,
                            bluetooth_beacon_t *strongest, void *arg) {
  xSemaphoreTake(giant_switch_mutex, portMAX_DELAY);

  uint16_t new_minor = strongest ? strongest->minor : 0;
  if (new_minor == current_giant_switch_minor) {
    goto cleanup;
  }

  // Re-enable any disabled FM frequencies
  struct frequency_handle *handle;
  TAILQ_FOREACH(handle, &fm_frequencies, next) {
    if (!handle->enabled) {
      ESP_LOGD(RADIO_TAG, "Re-enabling FM frequency %0.1f", handle->frequency);
      ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_enable_frequency(handle));
    }
  }

  if (new_minor != 0) {
    float low_frequency = ((float)newest->minor / 10.0) - 12.0f;
    float high_frequency = ((float)newest->minor / 10.0) + 12.0f;

    TAILQ_FOREACH(handle, &fm_frequencies, next) {
      if (fabsf(handle->frequency - high_frequency) < 0.1f ||
          fabsf(handle->frequency - low_frequency) < 0.1f) {
        ESP_LOGD(RADIO_TAG, "Disabling FM frequency %0.1f due to nearby beacon",
                 handle->frequency);
        ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_disable_frequency(handle));
      }
    }
  }

  current_giant_switch_minor = new_minor;

  struct frequency_handle *new_frequency = NULL;
  if (strongest) {
    float frequency = ((float)strongest->minor) / 10.0f;
    TAILQ_FOREACH(new_frequency, &fm_frequencies, next) {
      if (fabsf(new_frequency->frequency - frequency) < 0.1f) {
        break;
      }
    }
  }

cleanup:
  xSemaphoreGive(giant_switch_mutex);
}

esp_err_t tuner_register_pm_frequency(frequency_config_t *config,
                                      frequency_handle_t *handle) {
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, RADIO_TAG, "Config is NULL");
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG, "Handle is NULL");
  ESP_RETURN_ON_FALSE(tuner_task_handle == NULL, ESP_ERR_INVALID_STATE,
                      RADIO_TAG, "Tuner has already been initialized");

  struct frequency_handle *new_handle =
      calloc(1, sizeof(struct frequency_handle));
  ESP_RETURN_ON_FALSE(new_handle, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to allocate memory for frequency handle");

  new_handle->frequency = config->frequency;
  new_handle->enabled = config->enabled;
  new_handle->entune = config->entune;
  new_handle->detune = config->detune;
  new_handle->ctx = config->ctx;

  struct frequency_handle *i;
  TAILQ_FOREACH(i, &pm_frequencies, next) {
    if (i->frequency > new_handle->frequency) {
      TAILQ_INSERT_BEFORE(i, new_handle, next);
      break;
    }
  }
  if (!i) {
    TAILQ_INSERT_TAIL(&pm_frequencies, new_handle, next);
  }

  *handle = new_handle;
  return ESP_OK;
}

esp_err_t tuner_init(radio_calibration_t *calibration) {
  tuner_calibration = calibration;

  giant_switch_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(giant_switch_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create giant switch mutex");

  // Populate FM frequencies
  for (float frequency = FREQUENCY_FM_CHAN_MIN; frequency < FREQUENCY_FM_MAX;
       frequency += FREQUENCY_FM_SEPARATION) {
    struct frequency_handle *handle =
        calloc(1, sizeof(struct frequency_handle));
    if (!handle) {
      ESP_LOGE(RADIO_TAG, "Failed to allocate memory for FM frequency");
      return ESP_ERR_NO_MEM;
    }
    handle->frequency = frequency;
    handle->enabled = true;
    handle->entune = entune_fm_channel;
    uint16_t channel;
    ESP_RETURN_ON_ERROR(
        fm_frequency_to_channel(lroundf(frequency * 1000), &channel), RADIO_TAG,
        "Error converting frequency to channel");
    handle->ctx = (void *)(uintptr_t)channel;
    TAILQ_INSERT_TAIL(&fm_frequencies, handle, next);
  }

  uint16_t adc_low = calibration->frequency_min,
           adc_high = calibration->frequency_max;

  struct frequency_handle *handle;
  TAILQ_FOREACH(handle, &pm_frequencies, next) {
    float low_freq = handle->frequency - FREQUENCY_PM_WIDTH / 2;
    float high_freq = handle->frequency + FREQUENCY_PM_WIDTH / 2;
    low_freq = low_freq < FREQUENCY_PM_MIN ? FREQUENCY_PM_MIN : low_freq;
    high_freq = high_freq > FREQUENCY_PM_MAX ? FREQUENCY_PM_MAX : high_freq;

    handle->frequency_low =
        (uint16_t)(adc_low + (adc_high - adc_low) *
                                 (low_freq - FREQUENCY_PM_MIN) /
                                 (FREQUENCY_PM_MAX - FREQUENCY_PM_MIN));
    handle->frequency_center =
        (uint16_t)(adc_low + (adc_high - adc_low) *
                                 (handle->frequency - FREQUENCY_PM_MIN) /
                                 (FREQUENCY_PM_MAX - FREQUENCY_PM_MIN));
    handle->frequency_high =
        (uint16_t)(adc_low + (adc_high - adc_low) *
                                 (high_freq - FREQUENCY_PM_MIN) /
                                 (FREQUENCY_PM_MAX - FREQUENCY_PM_MIN));
  }

  TAILQ_FOREACH(handle, &fm_frequencies, next) {
    float low_freq = handle->frequency - FREQUENCY_FM_WIDTH / 2;
    float high_freq = handle->frequency + FREQUENCY_FM_WIDTH / 2;
    low_freq = low_freq < FREQUENCY_FM_MIN ? FREQUENCY_FM_MIN : low_freq;
    high_freq = high_freq > FREQUENCY_FM_MAX ? FREQUENCY_FM_MAX : high_freq;

    handle->frequency_low =
        (uint16_t)(adc_low + (adc_high - adc_low) *
                                 (low_freq - FREQUENCY_FM_MIN) /
                                 (FREQUENCY_FM_MAX - FREQUENCY_FM_MIN));
    handle->frequency_center =
        (uint16_t)(adc_low + (adc_high - adc_low) *
                                 (handle->frequency - FREQUENCY_FM_MIN) /
                                 (FREQUENCY_FM_MAX - FREQUENCY_FM_MIN));
    handle->frequency_high =
        (uint16_t)(adc_low + (adc_high - adc_low) *
                                 (high_freq - FREQUENCY_FM_MIN) /
                                 (FREQUENCY_FM_MAX - FREQUENCY_FM_MIN));
  }

  // Make sure frequencies don't overlap (too much)
  TAILQ_FOREACH(handle, &fm_frequencies, next) {
    struct frequency_handle *next = TAILQ_NEXT(handle, next);
    if (!next) {
      break;
    }

    ESP_RETURN_ON_FALSE(handle->frequency_high < next->frequency_center &&
                            next->frequency_low > handle->frequency_center,
                        ESP_ERR_INVALID_ARG, RADIO_TAG,
                        "FM frequency %f overlaps with %f", handle->frequency,
                        next->frequency);
  }
  TAILQ_FOREACH(handle, &pm_frequencies, next) {
    struct frequency_handle *next = TAILQ_NEXT(handle, next);
    if (!next) {
      break;
    }

    ESP_RETURN_ON_FALSE(handle->frequency_high < next->frequency_center &&
                            next->frequency_low > handle->frequency_center,
                        ESP_ERR_INVALID_ARG, RADIO_TAG,
                        "FM frequency %f overlaps with %f", handle->frequency,
                        next->frequency);
  }

  // Finally, expand the bounds on the low and high end to cover the pot's full
  // range, just in case
  handle = TAILQ_FIRST(&pm_frequencies);
  handle->frequency_low = 0;
  handle = TAILQ_LAST(&pm_frequencies, frequency_list);
  handle->frequency_high = 0xfff;
  handle = TAILQ_FIRST(&fm_frequencies);
  handle->frequency_low = 0;
  handle = TAILQ_LAST(&fm_frequencies, frequency_list);
  handle->frequency_high = 0xfff;

  gpio_config_t config = {
      .pin_bit_mask = BIT64(TOGGLE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&config), RADIO_TAG,
                      "Failed to configure modulation pin");
  current_radio_mode = gpio_to_tuner_mode(gpio_get_level(TOGGLE_PIN));
  desired_radio_mode = current_radio_mode;

  if (mode_tuners[current_radio_mode].entune) {
    mode_tuners[current_radio_mode].entune();
  }

  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreatePinnedToCore(tuner_task, "tuner",
                                                        4096, NULL, 12,
                                                        &tuner_task_handle, 1),
                      ESP_ERR_NO_MEM, RADIO_TAG, "Failed to create tuner task");
  ESP_RETURN_ON_ERROR(debounce_handler_add(TOGGLE_PIN, GPIO_INTR_ANYEDGE,
                                           modulation_callback, NULL,
                                           pdMS_TO_TICKS(10)),
                      RADIO_TAG, "Failed to add modulation callback");
  ESP_RETURN_ON_ERROR(adc_subscribe(
                          &(adc_digi_pattern_config_t){
                              .atten = ADC_ATTEN_DB_12,
                              .channel = FREQUENCY_ADC_CHANNEL,
                              .unit = ADC_UNIT_1,
                              .bit_width = 12,
                          },
                          adc_callback, NULL),
                      RADIO_TAG, "Failed to subscribe to ADC channel");

  ESP_RETURN_ON_ERROR(bluetooth_subscribe_beacon(BLUETOOTH_MAJOR_RICKROLL,
                                                 beacon_callback, NULL),
                      RADIO_TAG, "Failed to subscribe to bluetooth beacon");

  ESP_RETURN_ON_ERROR(things_register_telemetry_generator(
                          telemetry_generator, "tuner", &telemetry_index),
                      RADIO_TAG, "Failed to register telemetry generator");

  return ESP_OK;
}

esp_err_t tuner_enable_frequency(frequency_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid frequency handle");
  handle->enabled = true;
  // Wake up the tuner just in case
  if (tuner_task_handle) {
    xTaskNotifyGive(tuner_task_handle);
  }
  return ESP_OK;
}

esp_err_t tuner_disable_frequency(frequency_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid frequency handle");
  handle->enabled = false;
  // Wake up the tuner just in case
  if (tuner_task_handle) {
    xTaskNotifyGive(tuner_task_handle);
  }
  return ESP_OK;
}

esp_err_t tuner_suspend() {
  atomic_store(&tuner_suspended, true);
  if (tuner_task_handle) {
    xTaskNotifyGive(tuner_task_handle);
  }
  return ESP_OK;
}

esp_err_t tuner_resume() {
  atomic_store(&tuner_suspended, false);
  if (tuner_task_handle) {
    xTaskNotifyGive(tuner_task_handle);
  }
  return ESP_OK;
}
