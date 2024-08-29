#include "../config.h"
#include "main.h"
#include "wifi.h"
#include "things.h"
#include "console.h"
#include "webrtc.h"
#include "battery.h"
#include "board.h"
#include "tas2505.h"
#include "adc.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
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
#include "esp_adc/adc_oneshot.h"

const char *RADIO_TAG = "radio";

EventGroupHandle_t radio_event_group;

void dac_volume_output_task(void *arg)
{
  adc_oneshot_unit_handle_t adc_unit = adc_get_handle();

  adc_channel_t channel = VOLUME_ADC_CHANNEL;
  adc_oneshot_chan_cfg_t chan_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
  };
  esp_err_t err = adc_oneshot_config_channel(adc_unit, channel, &chan_cfg);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to configure ADC channel: %d", err);
    vTaskDelete(NULL);
  }

  while (1)
  {
loop:
    vTaskDelay(pdMS_TO_TICKS(100));

    bool gpio;
    err = tas2505_read_gpio(&gpio);
    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to read GPIO: %d", err);
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

    int volume_total = 0;
    for (int i = 0; i < 16; i++)
    {
      int volume;
      err = adc_oneshot_read(adc_unit, channel, &volume);
      if (err != ESP_OK)
      {
        ESP_LOGE(RADIO_TAG, "Failed to read ADC: %d", err);
        goto loop;
      }
      volume_total += volume;
    }
    // Shift down by 8 bits: 4 bits for the average and 4 bits to get from
    // 12-bit ADC precision to 8-bit volume
    err = tas2505_set_volume(volume_total >> 8);
    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to set volume: %d", err);
      goto loop;
    }
  }
}

void webrtc_pipeline_start(void *context)
{
  vTaskDelay(pdMS_TO_TICKS(100));

  webrtc_connection_t connection = (webrtc_connection_t)context;

  // Create pipeline
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
  if (!pipeline)
  {
    ESP_LOGE(RADIO_TAG, "Failed to create audio pipeline");
    vTaskDelete(NULL);
  }

  // Create I2S output
  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(I2S_NUM_0, 48000, 16, AUDIO_STREAM_WRITER);
  // Opus decoding will happen in the I2S task so it needs a lot of stack and a
  // big enough buffer
  i2s_cfg.task_core = 1;
  i2s_cfg.task_stack = 30 * 1024;
  i2s_cfg.stack_in_ext = true;
  // Need space for 20ms of audio at 48kHz, 16-bit, stereo
  // TODO: adapt to packet sizes?
  i2s_cfg.buffer_len = 960 * 2 * 2;
  audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);
  if (!i2s_stream_writer)
  {
    ESP_LOGE(RADIO_TAG, "Failed to create I2S stream");
    vTaskDelete(NULL);
  }
  esp_err_t err = audio_element_set_read_cb(i2s_stream_writer, webrtc_read_audio_sample, connection);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to set read callback for I2S stream: %d", err);
    vTaskDelete(NULL);
  }

  // Register audio elements to pipeline and link
  err = audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to register I2S stream: %d", err);
    vTaskDelete(NULL);
  }
  const char *link_tag[] = {"i2s"};
  err = audio_pipeline_link(pipeline, &link_tag[0], sizeof(link_tag) / sizeof(link_tag[0]));
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to link I2S stream: %d", err);
    vTaskDelete(NULL);
  }

  err = audio_pipeline_run(pipeline);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to run pipeline: %d", err);
    vTaskDelete(NULL);
  }

  vTaskDelete(NULL);
}

static void on_webrtc_state_change(webrtc_connection_t conn, void *context, WEBRTC_CONNECTION_STATE state)
{
  if (state == WEBRTC_CONNECTION_STATE_CONNECTED)
  {
    xTaskCreatePinnedToCore(webrtc_pipeline_start, "webrtc_pipeline", 4096, conn, 5, NULL, 1);
  }
}

void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  esp_log_level_set(RADIO_TAG, ESP_LOG_INFO);

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
  ESP_ERROR_CHECK(things_init());
  ESP_ERROR_CHECK(board_i2c_init());
  ESP_ERROR_CHECK(battery_init());
  ESP_ERROR_CHECK(tas2505_init());
  ESP_ERROR_CHECK(wifi_init());
  ESP_ERROR_CHECK(webrtc_init());

  xTaskCreatePinnedToCore(dac_volume_output_task, "dac_volume_output", 4096, NULL, 15, NULL, 1);

  if (!(xEventGroupGetBits(radio_event_group) & RADIO_EVENT_GROUP_THINGS_PROVISIONED))
  {
    uint8_t mac[6];
    ESP_ERROR_CHECK(wifi_get_mac(mac));
    char macstr[18];
    snprintf(macstr, sizeof(macstr), MACSTR, MAC2STR(mac));

    ESP_LOGE(RADIO_TAG, "Device not provisioned. Provision at https://%s/entities/devices with MAC address %s and paste device token", RADIO_THINGSBOARD_SERVER, macstr);
  }

  ESP_ERROR_CHECK(console_init());

  // If we make it this far, mark the firmware as good
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());

  // xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_THINGS_PROVISIONED, pdFALSE, pdTRUE, portMAX_DELAY);
  xEventGroupWaitBits(radio_event_group, RADIO_EVENT_GROUP_WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

  webrtc_config_t webrtc_cfg = {
      .whep_url = "http://192.168.21.207:8889/music/whep",
      .state_change_callback = on_webrtc_state_change,
  };
  webrtc_connection_t connection;
  webrtc_connect(&webrtc_cfg, &connection);
}