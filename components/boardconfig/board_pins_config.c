#include "board.h"

#include "esp_log.h"
#include "esp_check.h"
#include "audio_error.h"
#include "board_pins_config.h"
#include "driver/i2s_common.h"

static const char *TAG = "radio_boardconfig";

int8_t get_sdcard_open_file_num_max(void)
{
  return SDCARD_OPEN_FILE_NUM_MAX;
}

static i2c_master_bus_handle_t i2c_handle = 0;
SemaphoreHandle_t i2c_mutex;
UBaseType_t i2c_mutex_holder_priority;

esp_err_t
board_i2c_init()
{
  if (i2c_handle != NULL)
  {
    ESP_LOGW(TAG, "I2C bus already initialized");
    return ESP_OK;
  }

  i2c_master_bus_config_t i2c_cfg = {
      .i2c_port = I2C_NUM_0,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .scl_io_num = GPIO_NUM_40,
      .sda_io_num = GPIO_NUM_41,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  esp_err_t ret = i2c_new_master_bus(&i2c_cfg, &i2c_handle);
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_new_master_bus failed");

  i2c_mutex = xSemaphoreCreateMutex();
  if (i2c_mutex == NULL)
  {
    ESP_LOGE(TAG, "Failed to create I2C mutex");
    return ESP_FAIL;
  }

  return ESP_OK;
}

i2c_master_bus_handle_t board_i2c_get_handle()
{
  return i2c_handle;
}

esp_err_t get_i2s_pins(int port, board_i2s_pin_t *i2s_config)
{
  AUDIO_NULL_CHECK(TAG, i2s_config, return ESP_FAIL);
  if (port == 0)
  {
    i2s_config->bck_io_num = GPIO_NUM_16;
    i2s_config->ws_io_num = GPIO_NUM_17;
    i2s_config->data_out_num = GPIO_NUM_18;
    i2s_config->mck_io_num = I2S_GPIO_UNUSED;
    i2s_config->data_in_num = I2S_GPIO_UNUSED;
  }
  else
  {
    i2s_config->bck_io_num = I2S_GPIO_UNUSED;
    i2s_config->data_in_num = I2S_GPIO_UNUSED;
    i2s_config->data_out_num = I2S_GPIO_UNUSED;
    i2s_config->mck_io_num = I2S_GPIO_UNUSED;
    i2s_config->ws_io_num = I2S_GPIO_UNUSED;
    ESP_LOGE(TAG, "i2s port %d is not supported", port);
    return ESP_FAIL;
  }
  return ESP_OK;
}
