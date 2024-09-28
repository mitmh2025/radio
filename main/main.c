#include "../config.h"
#include "main.h"
#include "adc.h"
#include "battery.h"
#include "board.h"
#include "console.h"
#include "file_cache.h"
#include "led.h"
#include "mixer.h"
#include "storage.h"
#include "tas2505.h"
#include "things.h"
#include "webrtc.h"
#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "audio_pipeline.h"
#include "http_stream.h"
#include "mp3_decoder.h"
#include "i2s_stream.h"
#include "esp_crt_bundle.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag.h"
#include "esp_ota_ops.h"

const char *RADIO_TAG = "radio";

EventGroupHandle_t radio_event_group;

static bool volume_increasing = true;
// TAS2505 defaults to max volume, so start at max volume
static uint8_t last_volume_setting = 0xff;
static uint32_t average_volume = 0xfff;
void dac_volume_callback(adc_digi_output_data_t *result)
{
  uint32_t new_volume = average_volume - (average_volume >> 3) + (result->type2.data >> 3);

  // Take volume reading down from 12 to 8 bits
  uint8_t new_volume_setting = new_volume >> 4;

  // If volume was already increasing, pass through any increase. If not,
  // require 2 steps to introduce some hysteresis. Same for decreasing.
  if (new_volume_setting - last_volume_setting > (volume_increasing ? 1 : 2))
  {
    volume_increasing = true;
    last_volume_setting = new_volume_setting;
    tas2505_set_volume(new_volume_setting);
  }
  else if (last_volume_setting - new_volume_setting > (volume_increasing ? 2 : 1))
  {
    volume_increasing = false;
    last_volume_setting = new_volume_setting;
    tas2505_set_volume(new_volume_setting);
  }

  average_volume = new_volume;
}

void dac_output_task(void *arg)
{
  while (1)
  {
loop:
  uint32_t wait = 100 + esp_random() % 100;
  vTaskDelay(pdMS_TO_TICKS(wait));

  bool gpio;
  esp_err_t err = tas2505_read_gpio(&gpio);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to read GPIO: %d (%s)", err, esp_err_to_name(err));
    goto loop;
  }

    if (gpio)
    {
      tas2505_set_output(TAS2505_OUTPUT_SPEAKER);
    }
    else
    {
      tas2505_set_output(TAS2505_OUTPUT_HEADPHONE);
    }
  }
}

void webrtc_pipeline_start(void *context)
{
  webrtc_connection_t connection = (webrtc_connection_t)context;

  int64_t start = esp_timer_get_time();
  esp_err_t err = webrtc_wait_buffer_duration(connection, 48000, 3000);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to wait for buffer duration: %d (%s)", err, esp_err_to_name(err));
    vTaskDelete(NULL);
  }
  int64_t end = esp_timer_get_time();
  ESP_LOGI(RADIO_TAG, "Spent %lldms buffering audio", (end - start) / 1000);

  mixer_channel_t channel;
  err = mixer_play_audio(webrtc_read_audio_sample, connection, &channel);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to play audio: %d (%s)", err, esp_err_to_name(err));
    vTaskDelete(NULL);
  }

  TaskHandle_t killer = NULL;
  xTaskNotifyWait(0, ULONG_MAX, (uint32_t *)&killer, portMAX_DELAY);

  mixer_stop_audio(channel);
  webrtc_free_connection(connection);

  if (killer != NULL)
  {
    xTaskNotifyGive(killer);
  }

  vTaskDelete(NULL);
}

TaskHandle_t webrtc_pipeline_task = NULL;

static void on_webrtc_state_change(webrtc_connection_t conn, void *context, WEBRTC_CONNECTION_STATE state)
{
  if (state == WEBRTC_CONNECTION_STATE_CONNECTED)
  {
    xTaskCreatePinnedToCore(webrtc_pipeline_start, "webrtc_pipeline", 4096, conn, 10, &webrtc_pipeline_task, 1);
  }
}

static char *webrtc_current_url = NULL;
static webrtc_connection_t webrtc_connection = NULL;

void things_whep_url_callback(const char *key, things_attribute_t *attr)
{
  const char *url = NULL;
  switch (attr->type)
  {
  case THINGS_ATTRIBUTE_TYPE_UNSET:
    break;
  case THINGS_ATTRIBUTE_TYPE_STRING:
    url = attr->value.string;
    break;
  default:
    ESP_LOGE(RADIO_TAG, "Expected string for whep_url, got %d", attr->type);
    return;
  }

  if (webrtc_current_url != NULL && url != NULL && strcmp(webrtc_current_url, url) == 0)
  {
    ESP_LOGI(RADIO_TAG, "WebRTC URL unchanged");
    return;
  }

  if (webrtc_pipeline_task != NULL)
  {
    xTaskNotify(webrtc_pipeline_task, (uint32_t)xTaskGetCurrentTaskHandle(), eSetValueWithOverwrite);
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    webrtc_pipeline_task = NULL;
  }

  if (webrtc_current_url != NULL)
  {
    free(webrtc_current_url);
    webrtc_current_url = NULL;
  }

  if (url == NULL)
  {
    ESP_LOGE(RADIO_TAG, "No WebRTC WHEP URL has been configured. Set the `whep_url` shared attribute in ThingsBoard.");
    return;
  }

  ESP_LOGI(RADIO_TAG, "Connecting to WebRTC endpoint %s", url);

  webrtc_current_url = strdup(url);
  webrtc_config_t cfg = {
      .whep_url = webrtc_current_url,
      .state_change_callback = on_webrtc_state_change,
  };
  webrtc_connect(&cfg, &webrtc_connection);
}

void app_main(void)
{
  esp_log_set_level_master(ESP_LOG_DEBUG);
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set(RADIO_TAG, ESP_LOG_DEBUG);
  esp_log_level_set("webrtc", ESP_LOG_DEBUG);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  radio_event_group = xEventGroupCreate();
  if (radio_event_group == NULL)
  {
    ESP_LOGE(RADIO_TAG, "Failed to create event group");
    abort();
    return;
  }

  ESP_ERROR_CHECK(adc_init());
  ESP_ERROR_CHECK(led_init());
  ESP_ERROR_CHECK(wifi_init());
  ESP_ERROR_CHECK(board_i2c_init());
  ESP_ERROR_CHECK(battery_init());
  ESP_ERROR_CHECK(tas2505_init());
  ESP_ERROR_CHECK(mixer_init());

  ESP_ERROR_CHECK(adc_subscribe(&(adc_digi_pattern_config_t){
                                    .atten = ADC_ATTEN_DB_12,
                                    .channel = VOLUME_ADC_CHANNEL,
                                    .unit = ADC_UNIT_1,
                                    .bit_width = 12,
                                },
                                dac_volume_callback));
  xTaskCreatePinnedToCore(dac_output_task, "dac_output", 4096, NULL, 5, NULL, 0);

  ESP_ERROR_CHECK(things_init());
  ESP_ERROR_CHECK(storage_init());
  ESP_ERROR_CHECK(webrtc_init());
  ESP_ERROR_CHECK(file_cache_init());

  if (!(xEventGroupGetBits(radio_event_group) & RADIO_EVENT_GROUP_THINGS_PROVISIONED))
  {
    uint8_t mac[6];
    ESP_ERROR_CHECK(wifi_get_mac(mac));
    char macstr[18];
    snprintf(macstr, sizeof(macstr), MACSTR, MAC2STR(mac));

    ESP_LOGE(RADIO_TAG, "Device not provisioned. Provision at https://%s/entities/devices with MAC address %s and use `provision` console command to store", RADIO_THINGSBOARD_SERVER, macstr);
  }

  err = storage_mount();
  if (err == ESP_FAIL)
  {
    ESP_LOGE(RADIO_TAG, "Flash storage not formatted. Use `format` console command to format");
  }
  else
  {
    ESP_ERROR_CHECK(err);
  }

  ESP_ERROR_CHECK(console_init());

  // If we make it this far, mark the firmware as good
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());

  xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

  things_subscribe_attribute("whep_url", things_whep_url_callback);
}