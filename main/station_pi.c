#include "nvs.h"

#include "accelerometer.h"
#include "adc.h"
#include "audio_output.h"
#include "bounds.h"
#include "debounce.h"
#include "main.h"
#include "mixer.h"
#include "station_pi.h"
#include "station_pi_activation.h"
#include "tas2505.h"
#include "tone_generator.h"
#include "tuner.h"

#include "board.h"

#include <math.h>

#include "driver/touch_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#define STATION_PI_NVS_NAMESPACE "radio:pi"
static nvs_handle_t pi_nvs_handle;

static frequency_handle_t freq_handle;
static bool entuned = false;

static bounds_handle_t light_bounds;
static const uint16_t light_threshold = 500;
static uint16_t light_smooth = 0xffff;
static tone_generator_t light_tone = NULL;

// Copied from station_pi_activation
static const accelerometer_pulse_cfg_t knock_cfg = {
    .odr = ACCELEROMETER_DR_100HZ,
    .osm = ACCELEROMETER_OSM_NORMAL,
    .threshold = 22,
    .timelimit = 6,
    .latency = 1,
};
#define PULSE_DEBOUNCE_US (50 * 1000)
static int64_t knock_last_time = 0;
static esp_timer_handle_t knock_timer = NULL;
static tone_generator_t knock_tone = NULL;

static TaskHandle_t touch_task_handle = NULL;
static uint32_t touch_threshold = 0x8000;
static tone_generator_t touch_tone = NULL;

static tone_generator_t button_tone = NULL;

// TODO: telemetry

static void light_start_tone() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = FREQUENCY_G_4,
      },
      &light_tone));
}

static void light_stop_tone() {
  tone_generator_release(light_tone);
  light_tone = NULL;
}

static void knock_stop_tone(void *arg) {
  esp_timer_stop(knock_timer);
  if (knock_tone) {
    tone_generator_release(knock_tone);
    knock_tone = NULL;
  }
}

static void knock_start_tone(void *arg) {
  int64_t now = esp_timer_get_time();

  // Debounce the pulse
  if (now - knock_last_time < PULSE_DEBOUNCE_US) {
    goto cleanup;
  }

  if (esp_timer_is_active(knock_timer) || knock_tone) {
    knock_stop_tone(NULL);
  }

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(knock_timer, 400000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = FREQUENCY_C_5,
      },
      &knock_tone));

cleanup:
  knock_last_time = now;
}

static void touch_start_tone() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = FREQUENCY_A_4,
      },
      &touch_tone));
}

static void touch_stop_tone() {
  tone_generator_release(touch_tone);
  touch_tone = NULL;
}

static void button_start_tone() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = FREQUENCY_B_4,
      },
      &button_tone));
}

static void button_stop_tone() {
  tone_generator_release(button_tone);
  button_tone = NULL;
}

static void update_state() {
  if (light_tone) {
    tone_generator_set_entuned(light_tone, entuned);
  }
  if (knock_tone) {
    tone_generator_set_entuned(knock_tone, entuned);
  }
  if (touch_tone) {
    tone_generator_set_entuned(touch_tone, entuned);
  }
  if (button_tone) {
    tone_generator_set_entuned(button_tone, entuned);
  }
}

static void light_adc_cb(void *ctx, adc_digi_output_data_t *result) {
  uint16_t value = result->type2.data;

  if (light_smooth == 0xffff) {
    light_smooth = value;
    return;
  }
  light_smooth = light_smooth - (light_smooth >> 3) + (value >> 3);

  bounds_update(light_bounds, light_smooth);

  bool triggered =
      bounds_get_max(light_bounds) - light_threshold > light_smooth;
  if (triggered && !light_tone) {
    light_start_tone();
  } else if (!triggered && light_tone) {
    light_stop_tone();
  }
}

#define NOTIFY_TOUCH_ACTIVE BIT(0)
#define NOTIFY_TOUCH_INACTIVE BIT(1)
#define NOTIFY_BUTTON_ACTIVE BIT(2)
#define NOTIFY_BUTTON_INACTIVE BIT(3)

static void touch_task(void *ctx) {
  while (true) {
    uint32_t notification = 0;
    xTaskNotifyWait(0, ULONG_MAX, &notification, pdMS_TO_TICKS(100));

    if (notification & NOTIFY_TOUCH_ACTIVE && !touch_tone) {
      touch_start_tone();
    }
    if (notification & NOTIFY_TOUCH_INACTIVE && touch_tone) {
      touch_stop_tone();
    }
    if (notification & NOTIFY_BUTTON_ACTIVE && !button_tone) {
      button_start_tone();
    }
    if (notification & NOTIFY_BUTTON_INACTIVE && button_tone) {
      button_stop_tone();
    }
  }
}

static void IRAM_ATTR touch_intr(void *ctx) {
  TaskHandle_t task_handle = (TaskHandle_t)ctx;
  uint32_t touch_status = touch_pad_read_intr_status_mask();
  uint32_t notification = 0;
  if (touch_status & TOUCH_PAD_INTR_MASK_ACTIVE) {
    notification |= NOTIFY_TOUCH_ACTIVE;
  }
  if (touch_status & TOUCH_PAD_INTR_MASK_INACTIVE) {
    notification |= NOTIFY_TOUCH_INACTIVE;
  }
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, notification, eSetBits,
                     &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void IRAM_ATTR button_intr(void *ctx, bool state) {
  TaskHandle_t task_handle = (TaskHandle_t)ctx;
  uint32_t notification = state ? NOTIFY_BUTTON_ACTIVE : NOTIFY_BUTTON_INACTIVE;
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, notification, eSetBits,
                     &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void entune(void *ctx) {
  station_pi_activation_enable(false);
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_output_suspend());
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_output(TAS2505_OUTPUT_SPEAKER));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      accelerometer_subscribe_pulse(&knock_cfg, knock_start_tone, NULL));
  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&(gpio_config_t){
      .pin_bit_mask = 1ULL << BUTTON_TRIANGLE_PIN,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  }));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      debounce_handler_add(BUTTON_TRIANGLE_PIN, GPIO_INTR_ANYEDGE, button_intr,
                           touch_task_handle, 50000));

  entuned = true;
  update_state();
}

static void detune(void *ctx) {
  entuned = false;
  update_state();
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_TRIANGLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(accelerometer_unsubscribe_pulse());
  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_output_resume());
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
  station_pi_activation_enable(true);
}

esp_err_t station_pi_init() {
  ESP_RETURN_ON_ERROR(
      nvs_open(STATION_PI_NVS_NAMESPACE, NVS_READWRITE, &pi_nvs_handle),
      RADIO_TAG, "Failed to open NVS handle for station pi");

  ESP_RETURN_ON_ERROR(bounds_init(
                          &(bounds_config_t){
                              .buckets = 30,
                              .interval = 1000000,
                          },
                          &light_bounds),
                      RADIO_TAG, "Failed to initialize light bounds");
  ESP_RETURN_ON_ERROR(adc_subscribe(
                          &(adc_digi_pattern_config_t){
                              .atten = ADC_ATTEN_DB_12,
                              .channel = LIGHT_ADC_CHANNEL,
                              .unit = ADC_UNIT_1,
                              .bit_width = 12,
                          },
                          light_adc_cb, NULL),
                      RADIO_TAG, "Failed to subscribe to light ADC");

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "knock_tone",
                              .callback = knock_stop_tone,
                          },
                          &knock_timer),
                      RADIO_TAG, "Failed to initialize knock timer");

  ESP_RETURN_ON_FALSE(
      pdPASS == xTaskCreatePinnedToCore(touch_task, "touch_task", 4096, NULL,
                                        11, &touch_task_handle, 0),
      ESP_FAIL, RADIO_TAG, "Failed to create touch task");
  ESP_RETURN_ON_ERROR(touch_pad_init(), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_config(TOUCH_PAD_CHANNEL), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_filter_set_config(&(touch_filter_config_t){
                          .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
                          .mode = TOUCH_PAD_FILTER_IIR_256,
                      }),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_filter_enable(), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_denoise_set_config(&(touch_pad_denoise_t){
                          .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
                          .grade = TOUCH_PAD_DENOISE_BIT4,
                      }),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_denoise_enable(), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_set_thresh(TOUCH_PAD_CHANNEL, touch_threshold),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_isr_register(touch_intr, touch_task_handle,
                                             TOUCH_PAD_INTR_MASK_ACTIVE |
                                                 TOUCH_PAD_INTR_MASK_INACTIVE),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE |
                                            TOUCH_PAD_INTR_MASK_INACTIVE),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_fsm_start(), RADIO_TAG,
                      "Failed to initialize touch");

  uint8_t enabled = 0;
  esp_err_t ret = nvs_get_u8(pi_nvs_handle, "enabled", &enabled);
  if (ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get enabled from NVS");
  }

  frequency_config_t config = {
      .frequency = M_PI,
      .enabled = enabled != 0,
      .entune = entune,
      .detune = detune,
  };

  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register pi frequency");

  return ESP_OK;
}

esp_err_t station_pi_enable() {
  ESP_RETURN_ON_ERROR(nvs_set_u8(pi_nvs_handle, "enabled", 1), RADIO_TAG,
                      "Failed to set enabled in NVS");

  ESP_RETURN_ON_ERROR(tuner_enable_pm_frequency(freq_handle), RADIO_TAG,
                      "Failed to enable pi frequency");

  return ESP_OK;
}