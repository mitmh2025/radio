#include "audio_volume.h"
#include "adc.h"
#include "main.h"
#include "tas2505.h"

#include "board.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t volume_mutex = NULL;
static uint8_t volume_floor = 0;

static bool volume_increasing = true;
// TAS2505 defaults to max volume, so start at max volume
static uint16_t average_volume_raw = 0xfff;
static uint8_t last_volume_setting = 0xff;
static void volume_callback(void *user_data, adc_digi_output_data_t *result) {
  xSemaphoreTake(volume_mutex, portMAX_DELAY);

  radio_calibration_t *calibration = (radio_calibration_t *)user_data;

  average_volume_raw = average_volume_raw - (average_volume_raw >> 3) +
                       (result->type2.data >> 3);

  // Scale volume from calibration minimum to maximum (with clamping) to 0-512
  // (one more bit than actual volume setting, to allow for hysteresis)
  if (average_volume_raw < calibration->volume_min) {
    average_volume_raw = calibration->volume_min;
  } else if (average_volume_raw > calibration->volume_max) {
    average_volume_raw = calibration->volume_max;
  }
  uint16_t new_volume_setting =
      (uint64_t)((average_volume_raw - calibration->volume_min) * 512) /
      (calibration->volume_max - calibration->volume_min);
  if (new_volume_setting & 0x1) {
    if (volume_increasing) {
      new_volume_setting++;
    } else {
      new_volume_setting--;
    }
  }
  new_volume_setting >>= 1;

  // Clamp one more time
  if (new_volume_setting > 0xff) {
    new_volume_setting = 0xff;
  }

  if (new_volume_setting < volume_floor) {
    new_volume_setting = volume_floor;
  }

  if (new_volume_setting > last_volume_setting) {
    volume_increasing = true;
  } else if (new_volume_setting < last_volume_setting) {
    volume_increasing = false;
  }

  if (new_volume_setting != last_volume_setting) {
    tas2505_set_volume(new_volume_setting);
    last_volume_setting = new_volume_setting;
  }

  xSemaphoreGive(volume_mutex);
}

esp_err_t audio_volume_init(radio_calibration_t *calibration) {
  volume_mutex = xSemaphoreCreateMutex();

  ESP_RETURN_ON_ERROR(adc_subscribe(
                          &(adc_digi_pattern_config_t){
                              .atten = ADC_ATTEN_DB_12,
                              .channel = VOLUME_ADC_CHANNEL,
                              .unit = ADC_UNIT_1,
                              .bit_width = 12,
                          },
                          volume_callback, calibration),
                      RADIO_TAG, "Failed to subscribe to volume ADC");

  return ESP_OK;
}

void audio_volume_set_floor(uint8_t floor) {
  xSemaphoreTake(volume_mutex, portMAX_DELAY);
  volume_floor = floor;
  xSemaphoreGive(volume_mutex);
}

void audio_volume_clear_floor() {
  xSemaphoreTake(volume_mutex, portMAX_DELAY);
  volume_floor = 0;
  xSemaphoreGive(volume_mutex);
}
