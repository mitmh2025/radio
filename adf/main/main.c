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
#include "board.h"
#include "tas2505.h"

#include "wifi.h"

static const char *TAG = "radio";

extern const uint8_t music_start[] asm("_binary_song_mp3_start");
extern const uint8_t music_end[] asm("_binary_song_mp3_end");

static size_t music_pos = 0;

int mp3_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
  int read_size = music_end - music_start - music_pos;
  if (read_size == 0)
  {
    return AEL_IO_DONE;
  }
  else if (len < read_size)
  {
    read_size = len;
  }
  memcpy(buf, music_start + music_pos, read_size);
  music_pos += read_size;
  return read_size;
}

void app_main(void)
{
  // esp_log_level_set("*", ESP_LOG_INFO);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // wifi_setup();
  // xEventGroupWaitBits(wifi_event_group, WIFI_EVENT_GROUP_STA_GOT_IP, pdFALSE, pdTRUE, portMAX_DELAY);

  // Initialize hardware
  ESP_ERROR_CHECK(board_i2c_init());
  ESP_ERROR_CHECK(tas2505_init());

  // Create pipeline
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
  mem_assert(pipeline);

  // Create HTTP fetcher
  // http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
  // http_cfg.crt_bundle_attach = esp_crt_bundle_attach;
  // audio_element_handle_t http_stream_reader = http_stream_init(&http_cfg);
  // mem_assert(http_stream_reader);

  // Create mp3 decoder
  mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
  audio_element_handle_t mp3_decoder = mp3_decoder_init(&mp3_cfg);
  mem_assert(mp3_decoder);
  ESP_ERROR_CHECK(audio_element_set_read_cb(mp3_decoder, mp3_read_cb, NULL));

  // Create I2S output
  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
  i2s_cfg.type = AUDIO_STREAM_WRITER;
  audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);
  mem_assert(i2s_stream_writer);

  // Register audio elements to pipeline and link
  // ESP_ERROR_CHECK(audio_pipeline_register(pipeline, http_stream_reader, "http"));
  ESP_ERROR_CHECK(audio_pipeline_register(pipeline, mp3_decoder, "mp3"));
  ESP_ERROR_CHECK(audio_pipeline_register(pipeline, i2s_stream_writer, "i2s"));
  const char *link_tag[] = {"mp3", "i2s"};
  ESP_ERROR_CHECK(audio_pipeline_link(pipeline, &link_tag[0], sizeof(link_tag) / sizeof(link_tag[0])));

  audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
  audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
  audio_pipeline_set_listener(pipeline, evt);

  // audio_element_set_uri(http_stream_reader, "https://ebroder.net/assets/take5.mp3");

  ESP_ERROR_CHECK(audio_pipeline_run(pipeline));

  while (1)
  {
    audio_event_iface_msg_t msg;
    esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
    if (ret != ESP_OK)
    {
      continue;
    }

    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)mp3_decoder && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO)
    {
      audio_element_info_t music_info = {0};
      audio_element_getinfo(mp3_decoder, &music_info);
      ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
               music_info.sample_rates, music_info.bits, music_info.channels);
      i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
      continue;
    }
  }
}