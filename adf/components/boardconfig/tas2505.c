#include "board.h"
#include "tas2505.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"

#define TAS2505_RST_GPIO PA_ENABLE_GPIO

const char *TAG = "radio:tas2505";

static i2c_master_dev_handle_t i2c_device;

#define TAS2505_CFG_OP_SET_REG 0
#define TAS2505_CFG_OP_SET_BITS 1
#define TAS2505_CFG_OP_CLEAR_BITS 2
#define TAS2505_CFG_OP_DELAY 3

#define TAS2505_CFG_REG_SOFTWARE_RESET 0x001
#define TAS2505_CFG_REG_CLOCK1 0x004
#define TAS2505_CFG_REG_CLOCK2 0x005
#define TAS2505_CFG_REG_CLOCK3 0x006
#define TAS2505_CFG_REG_CLOCK4 0x007
#define TAS2505_CFG_REG_CLOCK5 0x008
#define TAS2505_CFG_REG_CLOCK6 0x00b
#define TAS2505_CFG_REG_CLOCK7 0x00c
#define TAS2505_CFG_REG_DOSR1 0x00d
#define TAS2505_CFG_REG_DOSR2 0x00e
#define TAS2505_CFG_REG_GPIO_CONTROL 0x034
#define TAS2505_CFG_REG_DAC_INSTRUCTION_SET 0x03c
#define TAS2505_CFG_REG_DAC_SETUP1 0x03f
#define TAS2505_CFG_REG_DAC_SETUP2 0x040
#define TAS2505_CFG_REG_DAC_VOLUME 0x041
#define TAS2505_CFG_REG_POWER_CONTROL 0x101
#define TAS2505_CFG_REG_LDO_CONTROL 0x102
#define TAS2505_CFG_REG_OUTPUT_CONTROL 0x109
#define TAS2505_CFG_REG_OUTPUT_ROUTING 0x10c
#define TAS2505_CFG_REG_HP_GAIN 0x110
#define TAS2505_CFG_REG_HP_VOLUME 0x116
#define TAS2505_CFG_REG_AINL_VOLUME 0x118
#define TAS2505_CFG_REG_SPEAKER_CONTROL 0x12d
#define TAS2505_CFG_REG_SPEAKER_VOLUME 0x12e
#define TAS2505_CFG_REG_SPEAKER_VOLUME_RANGE 0x130

typedef struct
{
  uint8_t op;
  uint16_t reg;
  uint8_t value;
} tas2505_cfg_reg_t;

static tas2505_cfg_reg_t tas2505_init_registers[] = {
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_SOFTWARE_RESET, 0x1},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_LDO_CONTROL, 0x0},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_GPIO_CONTROL, 0x8},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK1, 0x7},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK2, 0x92},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK3, 32},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK4, 0x0},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK5, 0x0},
    {TAS2505_CFG_OP_DELAY, 0, 15},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK6, 0x88},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_CLOCK7, 0x82},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_DOSR1, 0x0},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_DOSR2, 0x80},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_DAC_INSTRUCTION_SET, 0x2},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_DAC_VOLUME, 0x0},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_DAC_SETUP2, 0x4},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_POWER_CONTROL, 0x10},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_SPEAKER_VOLUME, 0x0},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_SPEAKER_VOLUME_RANGE, 0x30},
    {TAS2505_CFG_OP_SET_REG, TAS2505_CFG_REG_HP_GAIN, 0x0},
};

static tas2505_output_t current_output = TAS2505_OUTPUT_SPEAKER;

// TODO: Fix this to set register 0x1/0x9 to disable the HP driver
static tas2505_cfg_reg_t tas2505_speaker_output_registers[] = {
    {TAS2505_CFG_REG_OUTPUT_ROUTING, 0xc0},
    {TAS2505_CFG_REG_AINL_VOLUME, 0x80},
    {TAS2505_CFG_REG_HP_VOLUME, 0x75 /* mute */},
    {TAS2505_CFG_REG_SPEAKER_CONTROL, 0x2},
};

static tas2505_cfg_reg_t tas2505_headphone_output_registers[] = {
    {TAS2505_CFG_REG_OUTPUT_ROUTING, 0xb},
    {TAS2505_CFG_REG_AINL_VOLUME, 0x0},
    {TAS2505_CFG_REG_HP_VOLUME, 0x0},
    {TAS2505_CFG_REG_SPEAKER_CONTROL, 0x0},
};

static tas2505_cfg_reg_t tas2505_both_output_registers[] = {
    {TAS2505_CFG_REG_OUTPUT_ROUTING, 0xc4},
    {TAS2505_CFG_REG_AINL_VOLUME, 0x80},
    {TAS2505_CFG_REG_HP_VOLUME, 0x0},
    {TAS2505_CFG_REG_SPEAKER_CONTROL, 0x2},
};

static tas2505_input_t current_input = TAS2505_INPUT_DAC;

// These initialization values were calculated assuming 44.1kHz or 48kHz sample
// rate, stereo, 16-bit I2S (so byte clock of either 1.4112MHz or 1.536MHz), but
// in practice they seem to work fine with lower sample/clock rates too
//
// Specific values are DOSR=128, MDAC=2, NDAC=8, PLL_P=1, PLL_D=0, PLL_J=32, and
// PLL_R=2
static tas2505_cfg_reg_t tas2505_dac_on_registers[] = {
    {TAS2505_CFG_REG_DAC_SETUP1, 0xb0},
};

static tas2505_cfg_reg_t tas2505_dac_off_registers[] = {
    {TAS2505_CFG_REG_DAC_SETUP1, 0x00},
};

static tas2505_cfg_reg_t tas2505_line_on_registers[] = {
    {TAS2505_CFG_REG_OUTPUT_CONTROL, 0x23},
};

static tas2505_cfg_reg_t tas2505_line_off_registers[] = {
    {TAS2505_CFG_REG_OUTPUT_CONTROL, 0x20},
};

static esp_err_t tas2505_write_register(uint8_t offset, uint8_t value)
{
  uint8_t data[2] = {offset, value};
  esp_err_t ret = i2c_master_transmit(i2c_device, data, sizeof(data) / sizeof(data[0]), -1);
  return ret;
}

static esp_err_t tas2505_read_register(uint8_t offset, uint8_t *value)
{
  esp_err_t ret = i2c_master_transmit_receive(i2c_device, &offset, 1, value, 1, -1);
  return ret;
}

static esp_err_t tas2505_write_registers(tas2505_cfg_reg_t *registers, size_t len)
{
  esp_err_t ret = ESP_OK;
  uint8_t current_page = 0xff;

  BOARD_I2C_MUTEX_LOCK();

  for (size_t i = 0; i < len; i++)
  {
    switch (registers[i].op)
    {
    case TAS2505_CFG_OP_DELAY:
      vTaskDelay(registers[i].value / portTICK_PERIOD_MS);
      break;
    default: {
      uint8_t page = registers[i].reg >> 8;
      uint8_t offset = registers[i].reg & 0xff;
      if (current_page != page) {
        ret = tas2505_write_register(0x0, page);
        ESP_GOTO_ON_ERROR(ret, end, TAG, "i2c_bus_write_bytes: page=%d (idx=%d)", page, i);
        current_page = page;
      }

      uint8_t val = registers[i].value;
      if (registers[i].op == TAS2505_CFG_OP_SET_BITS || registers[i].op == TAS2505_CFG_OP_CLEAR_BITS) {
        ret = tas2505_read_register(offset, &val);
        ESP_GOTO_ON_ERROR(ret, end, TAG, "i2c_bus_read_bytes: reg=%d (idx=%d)", offset, i);

        if (registers[i].op == TAS2505_CFG_OP_SET_BITS) {
          val |= registers[i].value;
        } else {
          val &= ~registers[i].value;
        }
      }

      ret = tas2505_write_register(offset, registers[i].value);
      ESP_GOTO_ON_ERROR(
          ret,
          end,
          TAG,
          "i2c_bus_write_bytes: reg=%d val=%d (idx=%d)",
          offset,
          registers[i].value,
          i);
    }
    }
  }

end:
  BOARD_I2C_MUTEX_UNLOCK();
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
    ESP_RETURN_ON_ERROR(ret, TAG, "Writing initial registers failed");
    ret = tas2505_set_output(current_output);
    ESP_RETURN_ON_ERROR(ret, TAG, "Setting output failed");
    ret = tas2505_set_input(current_input);
    ESP_RETURN_ON_ERROR(ret, TAG, "Setting input failed");
    return ret;
  }
  else
  {
    gpio_set_level(TAS2505_RST_GPIO, 0);
    vTaskDelay(1);
  }
  return ESP_OK;
}

esp_err_t tas2505_set_output(tas2505_output_t output)
{
  esp_err_t ret = ESP_OK;

  switch (output)
  {
  case TAS2505_OUTPUT_SPEAKER:
    ESP_LOGI(TAG, "Setting output to speaker");
    ret = tas2505_write_registers(tas2505_speaker_output_registers, sizeof(tas2505_speaker_output_registers) / sizeof(tas2505_speaker_output_registers[0]));
    break;
  case TAS2505_OUTPUT_HEADPHONE:
    ESP_LOGI(TAG, "Setting output to headphone");
    ret = tas2505_write_registers(tas2505_headphone_output_registers, sizeof(tas2505_headphone_output_registers) / sizeof(tas2505_headphone_output_registers[0]));
    break;
  case TAS2505_OUTPUT_BOTH:
    ESP_LOGI(TAG, "Setting output to both");
    ret = tas2505_write_registers(tas2505_both_output_registers, sizeof(tas2505_both_output_registers) / sizeof(tas2505_both_output_registers[0]));
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }

  if (ret == ESP_OK)
  {
    current_output = output;
  }

  return ret;
}

esp_err_t tas2505_set_input(tas2505_input_t input)
{
  esp_err_t ret = ESP_OK;

  switch (input)
  {
    case TAS2505_INPUT_DAC:
      ESP_LOGI(TAG, "Setting input to DAC");
      ret = tas2505_write_registers(tas2505_line_off_registers, sizeof(tas2505_line_off_registers) / sizeof(tas2505_line_off_registers[0]));
      ESP_RETURN_ON_ERROR(ret, TAG, "Setting input to DAC failed");
      ret = tas2505_write_registers(tas2505_dac_on_registers, sizeof(tas2505_dac_on_registers) / sizeof(tas2505_dac_on_registers[0]));
      break;
    case TAS2505_INPUT_LINE:
      ESP_LOGI(TAG, "Setting input to line");
      ret = tas2505_write_registers(tas2505_dac_off_registers, sizeof(tas2505_dac_off_registers) / sizeof(tas2505_dac_off_registers[0]));
      ESP_RETURN_ON_ERROR(ret, TAG, "Setting input to line failed");
      ret = tas2505_write_registers(tas2505_line_on_registers, sizeof(tas2505_line_on_registers) / sizeof(tas2505_line_on_registers[0]));
      break;
    case TAS2505_INPUT_BOTH:
      ESP_LOGI(TAG, "Setting input to both");
      ret = tas2505_write_registers(tas2505_dac_on_registers, sizeof(tas2505_dac_on_registers) / sizeof(tas2505_dac_on_registers[0]));
      ESP_RETURN_ON_ERROR(ret, TAG, "Setting input to both failed");
      ret = tas2505_write_registers(tas2505_line_on_registers, sizeof(tas2505_line_on_registers) / sizeof(tas2505_line_on_registers[0]));
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  if (ret == ESP_OK)
  {
    current_input = input;
  }

  return ret;
}

esp_err_t tas2505_read_gpio(bool *val)
{
  esp_err_t ret = 0;
  uint8_t page = TAS2505_CFG_REG_GPIO_CONTROL >> 8;
  uint8_t offset = TAS2505_CFG_REG_GPIO_CONTROL & 0xff;
  uint8_t register_value = 0;

  BOARD_I2C_MUTEX_LOCK();
  ret = tas2505_write_register(0x0, page);
  ESP_GOTO_ON_ERROR(ret, end, TAG, "i2c_bus_write_bytes: switch to page=%d", page);
  ret = tas2505_read_register(offset, &register_value);
  ESP_GOTO_ON_ERROR(ret, end, TAG, "i2c_bus_read_bytes: reg=%d", offset);
  *val = !!(register_value & 0x2);

end:
  BOARD_I2C_MUTEX_UNLOCK();
  return ret;
}
