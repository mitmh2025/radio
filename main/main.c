#include "main.h"
#include "../config.h"
#include "accelerometer.h"
#include "adc.h"
#include "audio_output.h"
#include "audio_volume.h"
#include "battery.h"
#include "bluetooth.h"
#include "board.h"
#include "calibration.h"
#include "captive_http_server.h"
#include "console.h"
#include "debounce.h"
#include "file_cache.h"
#include "fm.h"
#include "led.h"
#include "magnet.h"
#include "mixer.h"
#include "playback.h"
#include "playback_queue.h"
#include "static.h"
#include "station_2pi.h"
#include "station_funaround.h"
#include "station_numbers.h"
#include "station_pi.h"
#include "station_pi_activation.h"
#include "station_rickroll.h"
#include "station_wifi.h"
#include "storage.h"
#include "tas2505.h"
#include "things.h"
#include "touch.h"
#include "tuner.h"
#include "webrtc.h"
#include "webrtc_manager.h"
#include "wifi.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"

const char *RADIO_TAG = "radio";

EventGroupHandle_t radio_event_group;

#ifdef CONFIG_RADIO_GIANT_SWITCH

static SemaphoreHandle_t giant_switch_file_mutex = NULL;
static TaskHandle_t giant_switch_task = NULL;
static char *giant_switch_file = NULL;

static void giant_switch_audio_file_attr_cb(const char *key,
                                            const things_attribute_t *value) {
  xSemaphoreTake(giant_switch_file_mutex, portMAX_DELAY);
  if (giant_switch_file) {
    free(giant_switch_file);
  }
  if (value->type != THINGS_ATTRIBUTE_TYPE_STRING) {
    giant_switch_file = NULL;
  } else {
    giant_switch_file = strdup(value->value.string);
  }
  xSemaphoreGive(giant_switch_file_mutex);
  xTaskNotifyGive(giant_switch_task);
}

static void giant_switch_frequency_attr_cb(const char *key,
                                           const things_attribute_t *value) {
  long long frequency;
  switch (value->type) {
  case THINGS_ATTRIBUTE_TYPE_FLOAT:
    frequency = lroundf(value->value.f * 1000);
    break;
  case THINGS_ATTRIBUTE_TYPE_INT:
    frequency = value->value.i * 1000;
    break;
  default:
    frequency = 0;
    break;
  }

  if (frequency == 0) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(fm_disable());
  } else {
    uint16_t channel;
    ESP_ERROR_CHECK_WITHOUT_ABORT(fm_enable());
    ESP_ERROR_CHECK_WITHOUT_ABORT(fm_frequency_to_channel(frequency, &channel));
    ESP_ERROR_CHECK_WITHOUT_ABORT(fm_tune(channel));
  }
}

static void giant_switch_main() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_NONE));
  ESP_ERROR_CHECK_WITHOUT_ABORT(bluetooth_set_mode(BLUETOOTH_MODE_AGGRESSIVE));

  giant_switch_file_mutex = xSemaphoreCreateMutex();
  giant_switch_task = xTaskGetCurrentTaskHandle();

  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_volume(255));
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_output(TAS2505_OUTPUT_HEADPHONE));

  ESP_ERROR_CHECK_WITHOUT_ABORT(things_subscribe_attribute(
      "audio_file", giant_switch_audio_file_attr_cb));
  ESP_ERROR_CHECK_WITHOUT_ABORT(things_subscribe_attribute(
      "fm_frequency", giant_switch_frequency_attr_cb));

  while (true) {
    xSemaphoreTake(giant_switch_file_mutex, portMAX_DELAY);
    char *file = giant_switch_file;
    if (!file) {
      xSemaphoreGive(giant_switch_file_mutex);
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
      continue;
    }

    playback_handle_t playback = NULL;
    esp_err_t ret = playback_file(
        &(playback_cfg_t){
            .path = file,
            .tuned = true,
        },
        &playback);
    xSemaphoreGive(giant_switch_file_mutex);
    if (ret != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to play file: %d", ret);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ret = playback_wait_for_completion(playback);
    playback_free(playback);
    if (ret != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to wait for completion: %d", ret);
    }
  }
}

#else // CONFIG_RADIO_GIANT_SWITCH

static void radio_main() {
  radio_calibration_t calibration;
  esp_err_t err = calibration_load(&calibration);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    err = calibration_calibrate(&calibration);
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK_WITHOUT_ABORT(captive_http_server_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_volume_init(&calibration));
  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_output_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(webrtc_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(webrtc_manager_init());

  ESP_ERROR_CHECK_WITHOUT_ABORT(station_2pi_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_funaround_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_numbers_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_pi_activation_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_pi_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_rickroll_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(station_wifi_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(tuner_init(&calibration));

  // Prevent the main task from exiting because we have stack-allocated
  // variables that we want to keep around
  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}

#endif

#define RADIO_HARDWARE_ERROR_CHECK(err)                                        \
  do {                                                                         \
    esp_err_t rc = (err);                                                      \
    if (unlikely((rc) != ESP_OK)) {                                            \
      ESP_LOGE(RADIO_TAG, "Hardware error: %d", (rc));                         \
      led_set_pixel(1, 255, 0, 0);                                             \
      /* 4000 ms + 1000 ms of panic delay */                                   \
      vTaskDelay(pdMS_TO_TICKS(4000));                                         \
      ESP_ERROR_CHECK(rc);                                                     \
    }                                                                          \
  } while (0)

void app_main(void) {
  esp_log_set_level_master(ESP_LOG_WARN);
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("AUDIO_PIPELINE", ESP_LOG_ERROR);
  esp_log_level_set("AUDIO_ELEMENT", ESP_LOG_ERROR);
  esp_log_level_set(RADIO_TAG, ESP_LOG_DEBUG);

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

  ESP_ERROR_CHECK(led_init());

  RADIO_HARDWARE_ERROR_CHECK(debounce_init());
  RADIO_HARDWARE_ERROR_CHECK(adc_init());
  RADIO_HARDWARE_ERROR_CHECK(touch_init());
  RADIO_HARDWARE_ERROR_CHECK(wifi_init());
  RADIO_HARDWARE_ERROR_CHECK(bluetooth_init());
  RADIO_HARDWARE_ERROR_CHECK(board_i2c_init());
  RADIO_HARDWARE_ERROR_CHECK(battery_init());
  RADIO_HARDWARE_ERROR_CHECK(tas2505_init());
  RADIO_HARDWARE_ERROR_CHECK(storage_init());
  RADIO_HARDWARE_ERROR_CHECK(mixer_init());
  RADIO_HARDWARE_ERROR_CHECK(console_init());
  RADIO_HARDWARE_ERROR_CHECK(playback_queue_init());

  RADIO_HARDWARE_ERROR_CHECK(fm_init());
#ifndef CONFIG_RADIO_GIANT_SWITCH
  RADIO_HARDWARE_ERROR_CHECK(accelerometer_init());
  RADIO_HARDWARE_ERROR_CHECK(magnet_init());
#endif

  // We want to mark the firmware as good fast enough that we're not likely to
  // run into spurious reboots, but not so fast that we haven't validated
  // anything. At this point, we've brought up most of the hardware subsystems,
  // so we're in a good place to mark the firmware as good.
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());

  ESP_ERROR_CHECK_WITHOUT_ABORT(things_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(file_cache_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(static_init());

  if (!(xEventGroupGetBits(radio_event_group) &
        RADIO_EVENT_GROUP_THINGS_PROVISIONED)) {
    uint8_t mac[6];
    ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_get_mac(mac));
    char macstr[18];
    snprintf(macstr, sizeof(macstr), MACSTR, MAC2STR(mac));

    ESP_LOGE(RADIO_TAG,
             "Device not provisioned. Provision with MAC address %s and use "
             "`provision` console command to store",
             macstr);
  }

#ifdef CONFIG_RADIO_GIANT_SWITCH
  giant_switch_main();
#else
  radio_main();
#endif
}
