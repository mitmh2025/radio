#include "tuner.h"
#include "adc.h"
#include "board.h"
#include "calibration.h"
#include "debounce.h"
#include "fm.h"
#include "main.h"
#include "mixer.h"
#include "tas2505.h"
#include "webrtc_manager.h"

#include "esp_check.h"

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

// TODO: telemetry

#define FREQUENCY_PM_MIN ((float)(M_PI))
#define FREQUENCY_PM_MAX ((float)(2 * M_PI))
// Channel is roughly 4% of the band
#define FREQUENCY_PM_WIDTH ((float)(M_PI) / 25.0f)

#define FREQUENCY_FM_MIN (88.0f)
#define FREQUENCY_FM_MAX (108.0f)
#define FREQUENCY_FM_CHAN_MIN (88.1f)
// Channel is roughly 1.5% of the band - this means channels will be partially
// overlapping, so we need to handle that in our hysteresis logic
#define FREQUENCY_FM_WIDTH (0.3f)
#define FREQUENCY_FM_SEPARATION (0.2f)
#define FREQUENCY_COUNT (100)

typedef struct frequency_spec {
  float frequency;
  bool enabled;
  void (*entune)(void *ctx);
  void (*detune)(void *ctx);
  void *ctx;

  // These fields are computed based on the frequency band ranges and ADC
  // calibration range
  uint32_t frequency_low;
  uint32_t frequency_center;
  uint32_t frequency_high;
} frequency_spec_t;

static int frequency_spec_compare(const void *a, const void *b) {
  frequency_spec_t *spec_a = (frequency_spec_t *)a;
  frequency_spec_t *spec_b = (frequency_spec_t *)b;

  if (spec_a->frequency < spec_b->frequency) {
    return -1;
  }
  if (spec_a->frequency > spec_b->frequency) {
    return 1;
  }
  return 0;
}

typedef enum {
  TUNER_MODE_PM,
  TUNER_MODE_FM,
} tuner_mode_t;
static tuner_mode_t desired_radio_mode = TUNER_MODE_PM;
static atomic_uint_least32_t current_raw_frequency = 0;
static tuner_mode_t current_radio_mode = TUNER_MODE_PM;
static frequency_spec_t *current_frequency = NULL;
static TaskHandle_t tuner_task_handle = NULL;

static void entune_pm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_input(TAS2505_INPUT_DAC));
}

static void detune_pm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
}

static void entune_fm() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_input(TAS2505_INPUT_BOTH));
  ESP_ERROR_CHECK_WITHOUT_ABORT(fm_enable());
}

static void detune_fm() { ESP_ERROR_CHECK_WITHOUT_ABORT(fm_disable()); }

static struct {
  void (*entune)();
  void (*detune)();
} mode_tuners[] = {
    [TUNER_MODE_PM] = {.entune = entune_pm, .detune = detune_pm},
    [TUNER_MODE_FM] = {.entune = entune_fm, .detune = detune_fm},
};

frequency_spec_t pm_frequencies[] = {
    {
        .frequency = M_PI,
        .enabled = false,
    },
    {
        .frequency = (2 * M_PI),
        .enabled = true,
        .entune = webrtc_manager_entune,
        .detune = webrtc_manager_detune,
    },
};
frequency_spec_t fm_frequencies[FREQUENCY_COUNT] = {};

static void entune_fm_channel(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(fm_tune((uint16_t)(uintptr_t)ctx));
}

static void IRAM_ATTR modulation_callback(void *user_data, bool state) {
  desired_radio_mode = state ? TUNER_MODE_PM : TUNER_MODE_FM;
  if (tuner_task_handle) {
    xTaskNotifyFromISR(tuner_task_handle, 0, eNoAction, NULL);
  }
}

static void adc_callback(void *user_data, adc_digi_output_data_t *result) {
  uint32_t raw_frequency = atomic_load(&current_raw_frequency);
  raw_frequency =
      raw_frequency - (raw_frequency >> 3) + (result->type2.data >> 3);
  atomic_store(&current_raw_frequency, raw_frequency);
  if (tuner_task_handle) {
    xTaskNotifyGive(tuner_task_handle);
  }
}

static void tuner_task(void *ctx) {
  while (true) {
    uint32_t frequency_raw = atomic_load(&current_raw_frequency);

    if (desired_radio_mode == current_radio_mode && current_frequency &&
        frequency_raw > current_frequency->frequency_low &&
        frequency_raw < current_frequency->frequency_high) {
      goto next;
    }

    // Moving to a new frequency, so need to detune the old one
    if (current_frequency && current_frequency->detune) {
      current_frequency->detune(current_frequency->ctx);
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
    frequency_spec_t *frequencies =
        current_radio_mode == TUNER_MODE_PM ? pm_frequencies : fm_frequencies;
    size_t frequency_count =
        current_radio_mode == TUNER_MODE_PM
            ? sizeof(pm_frequencies) / sizeof(pm_frequencies[0])
            : sizeof(fm_frequencies) / sizeof(fm_frequencies[0]);
    frequency_spec_t *new_frequency = NULL;
    for (int i = 0; i < frequency_count; i++) {
      if (frequencies[i].enabled &&
          frequency_raw > frequencies[i].frequency_low &&
          frequency_raw < frequencies[i].frequency_high) {
        new_frequency = &frequencies[i];
        break;
      }
    }

    current_frequency = new_frequency;
    if (current_frequency && current_frequency->entune) {
      current_frequency->entune(current_frequency->ctx);
    }

  next:
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
  }
}

esp_err_t tuner_init(radio_calibration_t *calibration) {
  // Populate FM frequencies
  for (int i = 0; i < sizeof(fm_frequencies) / sizeof(fm_frequencies[0]); i++) {
    fm_frequencies[i].frequency =
        FREQUENCY_FM_CHAN_MIN + i * FREQUENCY_FM_SEPARATION;
    fm_frequencies[i].enabled = true;
    fm_frequencies[i].entune = entune_fm_channel;
    uint16_t channel;
    ESP_RETURN_ON_ERROR(
        fm_frequency_to_channel(lroundf(fm_frequencies[i].frequency * 1000),
                                &channel),
        RADIO_TAG, "Error converting frequency to channel");
    fm_frequencies[i].ctx = (void *)(uintptr_t)channel;
  }
  // Sort PM frequencies (FM frequencies are already sorted)
  qsort(pm_frequencies, sizeof(pm_frequencies) / sizeof(pm_frequencies[0]),
        sizeof(frequency_spec_t), frequency_spec_compare);

  // Validate frequency ranges
  for (int i = 0; i < sizeof(pm_frequencies) / sizeof(pm_frequencies[0]); i++) {
    frequency_spec_t *spec = &pm_frequencies[i];
    if (spec->frequency < FREQUENCY_PM_MIN ||
        spec->frequency > FREQUENCY_PM_MAX) {
      ESP_LOGE(RADIO_TAG, "Invalid frequency %f for PM band", spec->frequency);
      return ESP_ERR_INVALID_ARG;
    }
  }
  // This should be impossible, but just in case
  for (int i = 0; i < sizeof(fm_frequencies) / sizeof(fm_frequencies[0]); i++) {
    frequency_spec_t *spec = &fm_frequencies[i];
    if (spec->frequency < FREQUENCY_FM_MIN ||
        spec->frequency > FREQUENCY_FM_MAX) {
      ESP_LOGE(RADIO_TAG, "Invalid frequency %f for FM band", spec->frequency);
      return ESP_ERR_INVALID_ARG;
    }
  }

  uint32_t adc_low = calibration->frequency_min,
           adc_high = calibration->frequency_max;

  for (int i = 0; i < sizeof(pm_frequencies) / sizeof(pm_frequencies[0]); i++) {
    frequency_spec_t *spec = &pm_frequencies[i];
    float low_freq = spec->frequency - FREQUENCY_PM_WIDTH / 2;
    float high_freq = spec->frequency + FREQUENCY_PM_WIDTH / 2;
    low_freq = low_freq < FREQUENCY_PM_MIN ? FREQUENCY_PM_MIN : low_freq;
    high_freq = high_freq > FREQUENCY_PM_MAX ? FREQUENCY_PM_MAX : high_freq;

    spec->frequency_low =
        (uint32_t)(adc_low + (adc_high - adc_low) *
                                 (low_freq - FREQUENCY_PM_MIN) /
                                 (FREQUENCY_PM_MAX - FREQUENCY_PM_MIN));
    spec->frequency_center =
        (uint32_t)(adc_low + (adc_high - adc_low) *
                                 (spec->frequency - FREQUENCY_PM_MIN) /
                                 (FREQUENCY_PM_MAX - FREQUENCY_PM_MIN));
    spec->frequency_high =
        (uint32_t)(adc_low + (adc_high - adc_low) *
                                 (high_freq - FREQUENCY_PM_MIN) /
                                 (FREQUENCY_PM_MAX - FREQUENCY_PM_MIN));
  }

  for (int i = 0; i < sizeof(fm_frequencies) / sizeof(fm_frequencies[0]); i++) {
    frequency_spec_t *spec = &fm_frequencies[i];
    float low_freq = spec->frequency - FREQUENCY_FM_WIDTH / 2;
    float high_freq = spec->frequency + FREQUENCY_FM_WIDTH / 2;
    low_freq = low_freq < FREQUENCY_FM_MIN ? FREQUENCY_FM_MIN : low_freq;
    high_freq = high_freq > FREQUENCY_FM_MAX ? FREQUENCY_FM_MAX : high_freq;

    spec->frequency_low =
        (uint32_t)(adc_low + (adc_high - adc_low) *
                                 (low_freq - FREQUENCY_FM_MIN) /
                                 (FREQUENCY_FM_MAX - FREQUENCY_FM_MIN));
    spec->frequency_center =
        (uint32_t)(adc_low + (adc_high - adc_low) *
                                 (spec->frequency - FREQUENCY_FM_MIN) /
                                 (FREQUENCY_FM_MAX - FREQUENCY_FM_MIN));
    spec->frequency_high =
        (uint32_t)(adc_low + (adc_high - adc_low) *
                                 (high_freq - FREQUENCY_FM_MIN) /
                                 (FREQUENCY_FM_MAX - FREQUENCY_FM_MIN));
  }

  // Make sure frequencies don't overlap (too much)
  for (int i = 0; i < sizeof(fm_frequencies) / sizeof(fm_frequencies[0]) - 1;
       i++) {
    frequency_spec_t *spec = &fm_frequencies[i];
    frequency_spec_t *next_spec = &fm_frequencies[i + 1];
    ESP_RETURN_ON_FALSE(spec->frequency_high < next_spec->frequency_center &&
                            next_spec->frequency_low > spec->frequency_center,
                        ESP_ERR_INVALID_ARG, RADIO_TAG,
                        "FM frequency %f overlaps with %f", spec->frequency,
                        next_spec->frequency);
  }
  for (int i = 0; i < sizeof(pm_frequencies) / sizeof(pm_frequencies[0]) - 1;
       i++) {
    frequency_spec_t *spec = &pm_frequencies[i];
    frequency_spec_t *next_spec = &pm_frequencies[i + 1];
    ESP_RETURN_ON_FALSE(spec->frequency_high < next_spec->frequency_center &&
                            next_spec->frequency_low > spec->frequency_center,
                        ESP_ERR_INVALID_ARG, RADIO_TAG,
                        "PM frequency %f overlaps with %f", spec->frequency,
                        next_spec->frequency);
  }

  // Finally, expand the bounds on the low and high end to cover the pot's full
  // range, just in case
  pm_frequencies[0].frequency_low = 0;
  pm_frequencies[sizeof(pm_frequencies) / sizeof(pm_frequencies[0]) - 1]
      .frequency_high = 0xfff;
  fm_frequencies[0].frequency_low = 0;
  fm_frequencies[sizeof(fm_frequencies) / sizeof(fm_frequencies[0]) - 1]
      .frequency_high = 0xfff;

  gpio_config_t config = {
      .pin_bit_mask = 1ULL << TOGGLE_PIN,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&config), RADIO_TAG,
                      "Failed to configure modulation pin");
  bool toggle_level = gpio_get_level(TOGGLE_PIN);
  current_radio_mode = toggle_level ? TUNER_MODE_PM : TUNER_MODE_FM;
  modulation_callback(NULL, toggle_level);

  if (mode_tuners[current_radio_mode].entune) {
    mode_tuners[current_radio_mode].entune();
  }

  ESP_RETURN_ON_ERROR(debounce_handler_add(TOGGLE_PIN, GPIO_INTR_ANYEDGE,
                                           modulation_callback, NULL, 50000),
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
  xTaskCreatePinnedToCore(tuner_task, "tuner", 4096, NULL, 12,
                          &tuner_task_handle, 1);

  return ESP_OK;
}
