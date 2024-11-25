#include "calibration.h"
#include "accelerometer.h"
#include "adc.h"
#include "board.h"
#include "debounce.h"
#include "led.h"
#include "magnet.h"
#include "main.h"
#include "mixer.h"
#include "tas2505.h"
#include "tone_generator.h"
#include "touch.h"

#include <math.h>

#include "esp_check.h"
#include "esp_err.h"

#include "nvs_flash.h"

static const char *CALIBRATION_NVS_NAMESPACE = "radio:cali";

esp_err_t calibration_load(radio_calibration_t *calibration) {
  ESP_RETURN_ON_FALSE(calibration != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Calibration is NULL");

  nvs_handle_t handle;
  ESP_RETURN_ON_ERROR(
      nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READONLY, &handle), RADIO_TAG,
      "Failed to open NVS handle for calibration");

  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(nvs_get_u32(handle, "vol_min", &calibration->volume_min),
                    cleanup, RADIO_TAG, "Failed to get volume min from NVS: %d",
                    err_rc_);
  ESP_GOTO_ON_ERROR(nvs_get_u32(handle, "vol_max", &calibration->volume_max),
                    cleanup, RADIO_TAG, "Failed to get volume max from NVS: %d",
                    err_rc_);
  ESP_GOTO_ON_ERROR(
      nvs_get_u32(handle, "freq_min", &calibration->frequency_min), cleanup,
      RADIO_TAG, "Failed to get frequency min from NVS: %d", err_rc_);
  ESP_GOTO_ON_ERROR(
      nvs_get_u32(handle, "freq_max", &calibration->frequency_max), cleanup,
      RADIO_TAG, "Failed to get frequency max from NVS: %d", err_rc_);

cleanup:
  nvs_close(handle);
  return ret;
}

#define BUTTON_NOTIFY_INDEX 0
#define ADC_NOTIFY_INDEX 1

static uint32_t adc_value = UINT32_MAX;

static void IRAM_ATTR button_callback(void *user_data, bool state) {
  TaskHandle_t task = (TaskHandle_t)user_data;
  vTaskNotifyGiveIndexedFromISR(task, BUTTON_NOTIFY_INDEX, NULL);
}

static void adc_callback(void *user_data, adc_digi_output_data_t *result) {
  if (adc_value == UINT32_MAX) {
    adc_value = result->type2.data;
  } else {
    adc_value = adc_value - (adc_value >> 3) + (result->type2.data >> 3);
  }
  TaskHandle_t task = (TaskHandle_t)user_data;
  xTaskNotifyIndexed(task, ADC_NOTIFY_INDEX, adc_value, eSetValueWithOverwrite);
}

static void IRAM_ATTR touch_callback(void *user_data) {
  uint32_t touch_status = touch_pad_read_intr_status_mask();
  if (!(touch_status & TOUCH_PAD_INTR_MASK_ACTIVE)) {
    return;
  }

  TaskHandle_t task = (TaskHandle_t)user_data;
  vTaskNotifyGiveIndexedFromISR(task, BUTTON_NOTIFY_INDEX, NULL);
}

esp_err_t calibration_calibrate(radio_calibration_t *calibration) {
  ESP_RETURN_ON_FALSE(calibration != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Calibration is NULL");

  bool debounce_registered = false, volume_registered = false,
       frequency_registered = false, touch_registered = false;
  tone_generator_t tone = NULL;
  nvs_handle_t handle;
  ESP_RETURN_ON_ERROR(
      nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READWRITE, &handle), RADIO_TAG,
      "Failed to open NVS handle for calibration");

  esp_err_t ret = ESP_OK;

  gpio_config_t button_config = {
      .pin_bit_mask = BIT64(BUTTON_CIRCLE_PIN) | BIT64(BUTTON_TRIANGLE_PIN) |
                      BIT64(TOGGLE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_GOTO_ON_ERROR(gpio_config(&button_config), cleanup, RADIO_TAG,
                    "Failed to configure button");
  ESP_GOTO_ON_ERROR(debounce_handler_add(
                        BUTTON_CIRCLE_PIN, GPIO_INTR_NEGEDGE, button_callback,
                        xTaskGetCurrentTaskHandle(), pdMS_TO_TICKS(10)),
                    cleanup, RADIO_TAG, "Failed to add debounce handler");
  debounce_registered = true;

  // Volume calibration
  ESP_GOTO_ON_ERROR(adc_subscribe(
                        &(adc_digi_pattern_config_t){
                            .atten = ADC_ATTEN_DB_12,
                            .channel = VOLUME_ADC_CHANNEL,
                            .unit = ADC_UNIT_1,
                            .bit_width = 12,
                        },
                        adc_callback, xTaskGetCurrentTaskHandle()),
                    cleanup, RADIO_TAG, "Failed to subscribe to volume ADC");
  volume_registered = true;

  ESP_LOGW(RADIO_TAG, "Set volume to minimum and press the circle button");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  // Store the next ADC reading
  xTaskNotifyWaitIndexed(ADC_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX,
                         &calibration->volume_min, portMAX_DELAY);
  ESP_LOGW(RADIO_TAG, "Volume min: %" PRIu32 " (0x%" PRIx32 ")",
           calibration->volume_min, calibration->volume_min);

  ESP_LOGW(RADIO_TAG, "Set volume to maximum and press the circle button");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  // Store the next ADC reading
  xTaskNotifyWaitIndexed(ADC_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX,
                         &calibration->volume_max, portMAX_DELAY);
  ESP_LOGW(RADIO_TAG, "Volume max: %" PRIu32 " (0x%" PRIx32 ")",
           calibration->volume_max, calibration->volume_max);

  // We don't need this reading, it's just to prevent the radio from blasting
  // them
  ESP_LOGW(RADIO_TAG, "Reset volume to minimum and press the circle button");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);

  ESP_GOTO_ON_ERROR(adc_unsubscribe(VOLUME_ADC_CHANNEL), cleanup, RADIO_TAG,
                    "Failed to unsubscribe from volume ADC");
  volume_registered = false;

  // Frequency calibration
  ESP_GOTO_ON_ERROR(adc_subscribe(
                        &(adc_digi_pattern_config_t){
                            .atten = ADC_ATTEN_DB_12,
                            .channel = FREQUENCY_ADC_CHANNEL,
                            .unit = ADC_UNIT_1,
                            .bit_width = 12,
                        },
                        adc_callback, xTaskGetCurrentTaskHandle()),
                    cleanup, RADIO_TAG, "Failed to subscribe to frequency ADC");
  frequency_registered = true;

  ESP_LOGW(RADIO_TAG,
           "Set frequency to the maximum position and press the circle button");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  // Store the next ADC reading
  uint32_t frequency_physical_max;
  xTaskNotifyWaitIndexed(ADC_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX,
                         &frequency_physical_max, portMAX_DELAY);
  // We need to assume that we have roughly 190º of rotation and need to be able
  // to measure 180º of difference, so we want to move the dial until the value
  // decreases by 5º, or 2.5% (plus some margin of error)
  uint32_t frequency_max_target = 0.96 * frequency_physical_max;
  ESP_LOGW(RADIO_TAG,
           "Frequency physical max: %" PRIu32 " (0x%" PRIx32 "), "
           "target: %" PRIu32 " (0x%" PRIx32 ")",
           frequency_physical_max, frequency_physical_max, frequency_max_target,
           frequency_max_target);

  ESP_LOGW(RADIO_TAG,
           "Turn frequency dial down until green LED turns on, then attach "
           "knob pointing at 108MHz, then press the circle button");
  xTaskNotifyStateClearIndexed(NULL, BUTTON_NOTIFY_INDEX);
  uint32_t frequency;
  while (xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, 0, ULONG_MAX, NULL, 0) ==
         pdFALSE) {
    // Take a reading, adjust the LED
    xTaskNotifyWaitIndexed(ADC_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, &frequency,
                           portMAX_DELAY);
    uint32_t error = imaxabs((int32_t)(frequency - frequency_max_target));
    // Our tolerance for error is 1/2 of the difference between the target and
    // the physical max
    uint32_t tolerance = (frequency_physical_max - frequency_max_target) >> 1;
    if (error < tolerance) {
      led_set_pixel(1, 0, 255, 0);
    } else {
      led_set_pixel(1, 255, 0, 0);
    }
  }
  // Take one last reading
  xTaskNotifyWaitIndexed(ADC_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, &frequency,
                         portMAX_DELAY);
  calibration->frequency_max = frequency;
  ESP_LOGW(RADIO_TAG,
           "Frequency max: %" PRIu32 " (0x%" PRIx32 "), target: %" PRIu32
           " (0x%" PRIx32 ")",
           calibration->frequency_max, calibration->frequency_max,
           frequency_max_target, frequency_max_target);
  led_set_pixel(1, 0, 0, 0);
  ESP_GOTO_ON_FALSE(
      frequency_physical_max > calibration->frequency_max &&
          frequency_physical_max - calibration->frequency_max > 8,
      ESP_FAIL, cleanup, RADIO_TAG,
      "Maximum frequency is too close to physical limit, calibration failed");

  ESP_LOGW(RADIO_TAG,
           "Set frequency dial to 88MHz and press the circle button");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  // Store the next ADC reading as our frequency min
  xTaskNotifyWaitIndexed(ADC_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX,
                         &calibration->frequency_min, portMAX_DELAY);
  ESP_LOGW(RADIO_TAG, "Frequency min: %" PRIu32 " (0x%" PRIx32 ")",
           calibration->frequency_min, calibration->frequency_min);
  ESP_GOTO_ON_FALSE(calibration->frequency_min > 0, ESP_FAIL, cleanup,
                    RADIO_TAG,
                    "Minimum frequency reads as 0, calibration failed");
  ESP_GOTO_ON_ERROR(adc_unsubscribe(FREQUENCY_ADC_CHANNEL), cleanup, RADIO_TAG,
                    "Failed to unsubscribe from frequency ADC");
  frequency_registered = false;

  // We are done with calibration; time for verification

  ESP_GOTO_ON_ERROR(tas2505_set_output(TAS2505_OUTPUT_SPEAKER), cleanup,
                    RADIO_TAG, "Failed to set TAS2505 output to speaker");
  ESP_GOTO_ON_ERROR(tas2505_set_volume(255), cleanup, RADIO_TAG,
                    "Failed to set TAS2505 volume");
  ESP_GOTO_ON_ERROR(tone_generator_init(
                        &(tone_generator_config_t){
                            .waveform = TONE_GENERATOR_SINE,
                            .frequency = 440,
                            .volume = 0x4000,
                            .attack_time = 0,
                            .decay_time = 0,
                            .sustain_level = 0xffff,
                            .release_time = 0,
                            .entuned = true,
                        },
                        &tone),
                    cleanup, RADIO_TAG, "Failed to initialize tone generator");
  ESP_LOGW(RADIO_TAG,
           "Press the circle button if you can hear a tone from the speaker");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);

  ESP_GOTO_ON_ERROR(tas2505_set_output(TAS2505_OUTPUT_HEADPHONE), cleanup,
                    RADIO_TAG, "Failed to set TAS2505 output to headphone");
  ESP_LOGW(RADIO_TAG, "Press the circle button if you can hear a tone through "
                      "the headphone jack");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);

  tone_generator_release(tone);
  tone = NULL;

  debounce_handler_remove(BUTTON_CIRCLE_PIN);
  debounce_registered = false;

  ESP_GOTO_ON_ERROR(debounce_handler_add(
                        BUTTON_TRIANGLE_PIN, GPIO_INTR_NEGEDGE, button_callback,
                        xTaskGetCurrentTaskHandle(), pdMS_TO_TICKS(10)),
                    cleanup, RADIO_TAG, "Failed to add debounce handler");
  ESP_LOGW(RADIO_TAG, "Press the triangle button");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  debounce_handler_remove(BUTTON_TRIANGLE_PIN);

  ESP_GOTO_ON_ERROR(
      touch_register_isr(touch_callback, xTaskGetCurrentTaskHandle()), cleanup,
      RADIO_TAG, "Failed to register touch ISR");
  touch_registered = true;
  ESP_LOGW(RADIO_TAG, "Touch each foot once");
  for (int i = 0; i < 4; i++) {
    xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                           portMAX_DELAY);
    ESP_LOGW(RADIO_TAG, "Touch detected");
  }
  ESP_GOTO_ON_ERROR(
      touch_deregister_isr(touch_callback, xTaskGetCurrentTaskHandle()),
      cleanup, RADIO_TAG, "Failed to deregister touch ISR");
  touch_registered = false;

  ESP_GOTO_ON_ERROR(
      debounce_handler_add(TOGGLE_PIN, GPIO_INTR_NEGEDGE, button_callback,
                           xTaskGetCurrentTaskHandle(), pdMS_TO_TICKS(10)),
      cleanup, RADIO_TAG, "Failed to add debounce handler");
  ESP_LOGW(RADIO_TAG, "Set modulation switch to FM");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  debounce_handler_remove(TOGGLE_PIN);
  ESP_GOTO_ON_ERROR(
      debounce_handler_add(TOGGLE_PIN, GPIO_INTR_POSEDGE, button_callback,
                           xTaskGetCurrentTaskHandle(), pdMS_TO_TICKS(10)),
      cleanup, RADIO_TAG, "Failed to add debounce handler");
  ESP_LOGW(RADIO_TAG, "Set modulation switch to PM");
  xTaskNotifyWaitIndexed(BUTTON_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, NULL,
                         portMAX_DELAY);
  debounce_handler_remove(TOGGLE_PIN);

  ESP_LOGW(RADIO_TAG, "Hold magnet to the ground symbol");
  while (true) {
    int16_t x, y, z;
    ESP_GOTO_ON_ERROR(magnet_read(&x, &y, &z), cleanup, RADIO_TAG,
                      "Failed to read accelerometer");
    uint32_t magnitude = sqrtf(x * x + y * y + z * z);
    bool sensed = magnitude > 2400;
    led_set_pixel(1, 0, sensed ? 255 : 0, sensed ? 0 : magnitude >> 4);
    if (sensed) {
      break;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
  led_set_pixel(1, 0, 0, 0);

  // TODO: verify accelerometer, light

  // Finally write calibration to NVS
  ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "vol_min", calibration->volume_min),
                    cleanup, RADIO_TAG, "Failed to set volume min in NVS: %d",
                    err_rc_);
  ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "vol_max", calibration->volume_max),
                    cleanup, RADIO_TAG, "Failed to set volume max in NVS: %d",
                    err_rc_);
  ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "freq_min", calibration->frequency_min),
                    cleanup, RADIO_TAG,
                    "Failed to set frequency min in NVS: %d", err_rc_);
  ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "freq_max", calibration->frequency_max),
                    cleanup, RADIO_TAG,
                    "Failed to set frequency max in NVS: %d", err_rc_);
  ESP_GOTO_ON_ERROR(nvs_commit(handle), cleanup, RADIO_TAG,
                    "Failed to commit calibration to NVS");

cleanup:
  if (tone) {
    tone_generator_release(tone);
  }

  if (touch_registered) {
    touch_deregister_isr(touch_callback, xTaskGetCurrentTaskHandle());
  }

  if (debounce_registered) {
    debounce_handler_remove(BUTTON_CIRCLE_PIN);
  }

  if (volume_registered) {
    adc_unsubscribe(VOLUME_ADC_CHANNEL);
  }

  if (frequency_registered) {
    adc_unsubscribe(FREQUENCY_ADC_CHANNEL);
  }

  nvs_close(handle);
  return ret;
}

esp_err_t calibration_erase() {
  nvs_handle_t handle;
  ESP_RETURN_ON_ERROR(
      nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READWRITE, &handle), RADIO_TAG,
      "Failed to open NVS handle for calibration");

  esp_err_t ret = ESP_OK;
  ESP_GOTO_ON_ERROR(nvs_erase_all(handle), cleanup, RADIO_TAG,
                    "Failed to erase calibration NVS");
cleanup:
  nvs_close(handle);
  return ret;
}