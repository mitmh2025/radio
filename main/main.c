#include "main.h"
#include "../config.h"
#include "accelerometer.h"
#include "adc.h"
#include "audio_output.h"
#include "battery.h"
#include "board.h"
#include "calibration.h"
#include "console.h"
#include "debounce.h"
#include "file_cache.h"
#include "fm.h"
#include "led.h"
#include "magnet.h"
#include "mixer.h"
#include "station_2pi.h"
#include "station_pi.h"
#include "station_pi_activation.h"
#include "storage.h"
#include "tas2505.h"
#include "things.h"
#include "tuner.h"
#include "webrtc.h"
#include "webrtc_manager.h"
#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/usb_serial_jtag.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"

const char *RADIO_TAG = "radio";

EventGroupHandle_t radio_event_group;

static bool volume_increasing = true;
// TAS2505 defaults to max volume, so start at max volume
static uint16_t average_volume_raw = 0xfff;
static uint8_t last_volume_setting = 0xff;
static void volume_callback(void *user_data, adc_digi_output_data_t *result) {
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

  if (new_volume_setting > last_volume_setting) {
    volume_increasing = true;
  } else if (new_volume_setting < last_volume_setting) {
    volume_increasing = false;
  }

  if (new_volume_setting != last_volume_setting) {
    tas2505_set_volume(new_volume_setting);
    last_volume_setting = new_volume_setting;
  }
}

void app_main(void) {
  esp_log_set_level_master(ESP_LOG_DEBUG);
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("AUDIO_PIPELINE", ESP_LOG_ERROR);
  esp_log_level_set("AUDIO_ELEMENT", ESP_LOG_ERROR);
  esp_log_level_set(RADIO_TAG, ESP_LOG_DEBUG);
  esp_log_level_set("kvswebrtc", ESP_LOG_WARN);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  radio_event_group = xEventGroupCreate();
  if (radio_event_group == NULL) {
    ESP_LOGE(RADIO_TAG, "Failed to create event group");
    abort();
    return;
  }

  ESP_ERROR_CHECK(debounce_init());
  ESP_ERROR_CHECK(adc_init());
  ESP_ERROR_CHECK(led_init());
  ESP_ERROR_CHECK(wifi_init());
  ESP_ERROR_CHECK(board_i2c_init());
  ESP_ERROR_CHECK(battery_init());
  ESP_ERROR_CHECK(tas2505_init());
  ESP_ERROR_CHECK(storage_init());
  ESP_ERROR_CHECK(fm_init());
  ESP_ERROR_CHECK(accelerometer_init());
  ESP_ERROR_CHECK(magnet_init());
  ESP_ERROR_CHECK(mixer_init());
  ESP_ERROR_CHECK(console_init());

  // We want to mark the firmware as good fast enough that we're not likely to
  // run into spurious reboots, but not so fast that we haven't validated
  // anything. At this point, we've brought up most of the hardware subsystems,
  // so we're in a good place to mark the firmware as good.
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());

  radio_calibration_t calibration;
  err = calibration_load(&calibration);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    err = calibration_calibrate(&calibration);
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK_WITHOUT_ABORT(adc_subscribe(
      &(adc_digi_pattern_config_t){
          .atten = ADC_ATTEN_DB_12,
          .channel = VOLUME_ADC_CHANNEL,
          .unit = ADC_UNIT_1,
          .bit_width = 12,
      },
      volume_callback, &calibration));

  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_output_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(webrtc_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(things_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(file_cache_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(webrtc_manager_init());

  ESP_ERROR_CHECK_WITHOUT_ABORT(station_pi_activation_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_pi_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_2pi_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_init(&calibration));

  if (!(xEventGroupGetBits(radio_event_group) &
        RADIO_EVENT_GROUP_THINGS_PROVISIONED)) {
    uint8_t mac[6];
    ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_get_mac(mac));
    char macstr[18];
    snprintf(macstr, sizeof(macstr), MACSTR, MAC2STR(mac));

    ESP_LOGE(RADIO_TAG,
             "Device not provisioned. Provision at https://%s/entities/devices "
             "with MAC address %s and use `provision` console command to store",
             RADIO_THINGSBOARD_SERVER, macstr);
  }

  // Prevent the main task from exiting because we have stack-allocated
  // variables that we want to keep around
  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}
