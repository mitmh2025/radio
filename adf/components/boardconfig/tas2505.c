#include "board.h"
#include "tas2505.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"

#define TAS2505_RST_GPIO PA_ENABLE_GPIO

const char *TAG = "radio:tas2505";

static i2c_master_dev_handle_t i2c_device;

#define TAS2505_CFG_META_DELAY (255)

typedef struct
{
  uint8_t offset;
  uint8_t value;
} tas2505_cfg_reg_t;

// These initialization values were calculated assuming 44.1kHz or 48kHz sample
// rate, stereo, 16-bit I2S (so byte clock of either 1.4112MHz or 1.536MHz), but
// in practice they seem to work fine with lower sample/clock rates too
//
// Specific values are DOSR=128, MDAC=2, NDAC=8, PLL_P=1, PLL_D=0, PLL_J=32, and
// PLL_R=2
static tas2505_cfg_reg_t tas2505_init_registers[] = {
    {0x0, 0x0},
    {0x1, 0x1},
    {0x0, 0x1},
    {0x2, 0x0},
    {0x0, 0x0},
    {0x4, 0x7},
    {0x5, 0x92},
    {0x6, 32},
    {0x7, 0x0},
    {0x8, 0x0},
    {TAS2505_CFG_META_DELAY, 15},
    {0xb, 0x88},
    {0xc, 0x82},
    {0xd, 0x0},
    {0xe, 0x80},
    {0x3c, 0x2},
    {0x3f, 0xb0},
    {0x41, 0x0},
    {0x40, 0x4},
    {0x0, 0x1},
    {0x1, 0x10},
    {0x2e, 0x0},
    {0x30, 0x30},
    {0x2d, 0x2},
};

static esp_err_t tas2505_write_registers(tas2505_cfg_reg_t *registers, size_t len)
{
  esp_err_t ret = ESP_OK;
  uint8_t data[2];

  for (size_t i = 0; i < len; i++)
  {
    switch (registers[i].offset)
    {
    case TAS2505_CFG_META_DELAY:
      vTaskDelay(registers[i].value / portTICK_PERIOD_MS);
      break;
    default:
      data[0] = registers[i].offset;
      data[1] = registers[i].value;
      BOARD_I2C_MUTEX_LOCK();
      ret = i2c_master_transmit(i2c_device, data, sizeof(data) / sizeof(data[0]), -1);
      BOARD_I2C_MUTEX_UNLOCK();
      ESP_RETURN_ON_ERROR(
          ret,
          TAG,
          "i2c_bus_write_bytes: reg=%d val=%d (idx=%d)",
          registers[i].offset,
          registers[i].value,
          i);
    }
  }

  return ret;
}

esp_err_t tas2505_init()
{
  esp_err_t ret = ESP_OK;

  gpio_config_t io_conf = {
      .pin_bit_mask = BIT64(TAS2505_RST_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ret = gpio_config(&io_conf);
  ESP_RETURN_ON_ERROR(ret, TAG, "gpio_config failed");

  gpio_set_level(TAS2505_RST_GPIO, 0);
  vTaskDelay(1);
  gpio_set_level(TAS2505_RST_GPIO, 1);
  vTaskDelay(1);

  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL)
  {
    ESP_LOGE(TAG, "I2C bus is not initialized");
    return ESP_FAIL;
  }

  BOARD_I2C_MUTEX_LOCK();
  ret = i2c_master_probe(i2c_bus, TAS2505_ADDR, 100);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_probe failed");

  i2c_device_config_t i2c_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = TAS2505_ADDR,
      .scl_speed_hz = 400000,
  };

  BOARD_I2C_MUTEX_LOCK();
  ret = i2c_master_bus_add_device(i2c_bus, &i2c_cfg, &i2c_device);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_bus_add_device failed");

  tas2505_enable_pa(false);
  tas2505_enable_pa(true);

  return ret;
}

esp_err_t tas2505_deinit(void)
{
  BOARD_I2C_MUTEX_LOCK();
  esp_err_t ret = i2c_master_bus_rm_device(i2c_device);
  BOARD_I2C_MUTEX_UNLOCK();
  return ret;
}

esp_err_t tas2505_enable_pa(bool enable)
{
  // RST pin only needs to be low for 10ns or high for 1ms; either way that's
  // less than a single tick
  if (enable)
  {
    gpio_set_level(TAS2505_RST_GPIO, 1);
    vTaskDelay(1);

    esp_err_t ret = tas2505_write_registers(tas2505_init_registers, sizeof(tas2505_init_registers) / sizeof(tas2505_init_registers[0]));
    return ret;
  }
  else
  {
    gpio_set_level(TAS2505_RST_GPIO, 0);
    vTaskDelay(1);
  }
  return ESP_OK;
}
