#include "fm.h"
#include "board.h"
#include "main.h"
#include "things.h"

#include "esp_check.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <string.h>

#define SI4702_ADDR 0x10

// Register definitions taken from https://github.com/pu2clr/SI470X by PU2CLR
// (Ricardo Lima Caratti) and are distributed under the MIT license

typedef union {
  struct {
    uint16_t MFGID : 12;
    uint16_t PN : 4;
  } refined;
  uint16_t raw;
} si4702_reg00;
typedef union {
  struct {
    uint16_t FIRMWARE : 6;
    uint16_t DEV : 4;
    uint16_t REV : 6;
  } refined;
  uint16_t raw;
} si4702_reg01;
typedef union {
  struct {
    uint8_t ENABLE : 1;
    uint8_t RESERVED1 : 5;
    uint8_t DISABLE : 1;
    uint8_t RESERVED2 : 1;
    uint8_t SEEK : 1;
    uint8_t SEEKUP : 1;
    uint8_t SKMODE : 1;
    uint8_t RDSM : 1;
    uint8_t RESERVED3 : 1;
    uint8_t MONO : 1;
    uint8_t DMUTE : 1;
    uint8_t DSMUTE : 1;
  } refined;
  uint16_t raw;
} si4702_reg02;
typedef union {
  struct {
    uint16_t CHAN : 10;
    uint16_t RESERVED : 5;
    uint16_t TUNE : 1;
  } refined;
  uint16_t raw;
} si4702_reg03;
typedef union {
  struct {
    uint8_t GPIO1 : 2;
    uint8_t GPIO2 : 2;
    uint8_t GPIO3 : 2;
    uint8_t BLNDADJ : 2;
    uint8_t RESERVED1 : 2;
    uint8_t AGCD : 1;
    uint8_t DE : 1;
    uint8_t RDS : 1;
    uint8_t RESERVED2 : 1;
    uint8_t STCIEN : 1;
    uint8_t RDSIEN : 1;
  } refined;
  uint16_t raw;
} si4702_reg04;
typedef union {
  struct {
    uint8_t VOLUME : 4;
    uint8_t SPACE : 2;
    uint8_t BAND : 2;
    uint8_t SEEKTH;
  } refined;
  uint16_t raw;
} si4702_reg05;
typedef union {
  struct {
    uint8_t SKCNT : 4;
    uint8_t SKSNR : 4;
    uint8_t VOLEXT : 1;
    uint8_t RESERVED : 3;
    uint8_t SMUTEA : 2;
    uint8_t SMUTER : 2;
  } refined;
  uint16_t raw;
} si4702_reg06;
typedef union {
  struct {
    uint16_t RESERVED : 14;
    uint16_t AHIZEN : 1;
    uint16_t XOSCEN : 1;
  } refined;
  uint16_t raw;
} si4702_reg07;
typedef union {
  struct {
    uint8_t lowByte;
    uint8_t highByte;
  } refined;
  uint16_t raw;
} si4702_reg08;
typedef union {
  struct {
    uint8_t lowByte;
    uint8_t highByte;
  } refined;
  uint16_t raw;
} si4702_reg09;
typedef union {
  struct {
    uint8_t RSSI;
    uint8_t ST : 1;
    uint8_t BLERA : 2;
    uint8_t RDSS : 1;
    uint8_t AFCRL : 1;
    uint8_t SF_BL : 1;
    uint8_t STC : 1;
    uint8_t RDSR : 1;
  } refined;
  uint16_t raw;
} si4702_reg0a;
typedef union {
  struct {
    uint16_t READCHAN : 10;
    uint16_t BLERD : 2;
    uint16_t BLERC : 2;
    uint16_t BLERB : 2;
  } refined;
  uint16_t raw;
} si4702_reg0b;
typedef uint16_t si4702_reg0c;
typedef uint16_t si4702_reg0d;
typedef uint16_t si4702_reg0e;
typedef uint16_t si4702_reg0f;

static long long fm_frequency_base[] = {
    87500,
    76000,
    76000,
    0,
};

static long long fm_frequency_spacing[] = {
    200,
    100,
    50,
    0,
};

static SemaphoreHandle_t si4702_mutex = NULL;
static union {
  struct {
    si4702_reg00 reg00;
    si4702_reg01 reg01;
    si4702_reg02 reg02;
    si4702_reg03 reg03;
    si4702_reg04 reg04;
    si4702_reg05 reg05;
    si4702_reg06 reg06;
    si4702_reg07 reg07;
    si4702_reg08 reg08;
    si4702_reg09 reg09;
    si4702_reg0a reg0a;
    si4702_reg0b reg0b;
    si4702_reg0c reg0c;
    si4702_reg0d reg0d;
    si4702_reg0e reg0e;
    si4702_reg0f reg0f;
  };
  uint16_t raw[16];
} si4702_shadow_registers = {};

static i2c_master_dev_handle_t i2c_device;
static const char *TAG = "radio:si4702";

// Frequency is units of kHz
esp_err_t fm_channel_to_frequency(uint16_t channel, long long *frequency) {
  ESP_RETURN_ON_FALSE(frequency != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "frequency must not be NULL");
  long long frequency_base =
      fm_frequency_base[si4702_shadow_registers.reg05.refined.BAND];
  long long frequency_spacing =
      fm_frequency_spacing[si4702_shadow_registers.reg05.refined.SPACE];
  *frequency = frequency_base + (frequency_spacing * channel);
  return ESP_OK;
}

esp_err_t fm_frequency_to_channel(long long frequency, uint16_t *channel) {
  ESP_RETURN_ON_FALSE(channel != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "channel must not be NULL");
  long long frequency_base =
      fm_frequency_base[si4702_shadow_registers.reg05.refined.BAND];
  long long frequency_spacing =
      fm_frequency_spacing[si4702_shadow_registers.reg05.refined.SPACE];

  ESP_RETURN_ON_FALSE(frequency >= frequency_base, ESP_ERR_INVALID_ARG, TAG,
                      "frequency must be greater than or equal to base");
  long long frequency_offset = frequency - frequency_base;

  ESP_RETURN_ON_FALSE(frequency_offset % frequency_spacing == 0,
                      ESP_ERR_INVALID_ARG, TAG,
                      "frequency must be a multiple of spacing");
  *channel = frequency_offset / frequency_spacing;

  return ESP_OK;
}

// Note that read operations begin at register 0x0a and then wrap around. Pass
// -1 to read all
static esp_err_t fm_read_registers(int8_t end_reg) {
  size_t register_read_len;
  if (end_reg == -1) {
    register_read_len = 16;
  } else if (end_reg >= 0x0a) {
    register_read_len = end_reg - 0x0a + 1;
  } else {
    register_read_len = (0x10 - 0xa) + end_reg + 1;
  }

  // Registers are 16-bit
  register_read_len <<= 1;

  uint8_t buffer[register_read_len];
  memset(buffer, 0, register_read_len);

  BOARD_I2C_MUTEX_LOCK();
  esp_err_t ret = i2c_master_receive(i2c_device, buffer, register_read_len, -1);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_receive failed");

  for (int i = 0; i < register_read_len; i += 2) {
    size_t addr = (0x0a + (i >> 1)) & 0xf;
    si4702_shadow_registers.raw[addr] = (buffer[i] << 8) | buffer[i + 1];
  }

  return ESP_OK;
}

// Weirdly, *write* operations begin at register 0x02. Pass -1 to write all
static esp_err_t fm_flush_registers(int8_t end_reg) {
  size_t register_write_len;
  if (end_reg == -1) {
    register_write_len = 16;
  } else if (end_reg >= 0x02) {
    register_write_len = end_reg - 0x02 + 1;
  } else {
    register_write_len = (0x10 - 0x02) + end_reg + 1;
  }

  // Registers are 16-bit
  register_write_len <<= 1;

  uint8_t buffer[register_write_len];
  for (int i = 0; i < register_write_len; i += 2) {
    size_t addr = (0x02 + (i >> 1)) & 0xf;
    buffer[i] = si4702_shadow_registers.raw[addr] >> 8;
    buffer[i + 1] = si4702_shadow_registers.raw[addr] & 0xff;
  }

  BOARD_I2C_MUTEX_LOCK();
  esp_err_t ret =
      i2c_master_transmit(i2c_device, buffer, register_write_len, -1);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_transmit failed");

  return ESP_OK;
}

static size_t telemetry_index = 0;
static void fm_telemetry_generator() {
  xSemaphoreTake(si4702_mutex, portMAX_DELAY);

  esp_err_t ret = fm_read_registers(0xb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Unable to fetch status for telemetry: %d", ret);
    // Report what we have anyway
  }

  long long frequency;
  ret = fm_channel_to_frequency(si4702_shadow_registers.reg0b.refined.READCHAN,
                                &frequency);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Unable to convert channel to frequency: %d", ret);
    frequency = 0;
  }

  bool enabled = !si4702_shadow_registers.reg02.refined.DISABLE;
  uint8_t rssi = si4702_shadow_registers.reg0a.refined.RSSI;

  xSemaphoreGive(si4702_mutex);

  things_send_telemetry_bool("fm_enabled", enabled);
  if (frequency != 0) {
    things_send_telemetry_int("fm_frequency", frequency);
  }
  things_send_telemetry_int("fm_rssi", rssi);
}

esp_err_t fm_init(void) {
  si4702_mutex = xSemaphoreCreateMutex();
  if (si4702_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return ESP_FAIL;
  }

  gpio_config_t io_conf = {
      .pin_bit_mask = BIT64(SI4702_RST_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&io_conf);
  ESP_RETURN_ON_ERROR(ret, TAG, "gpio_config failed");

  // We need to make sure that no I2C operations happen in the 300ns before we
  // pulse the RST pin, and pulse must be at least 30ns
  BOARD_I2C_MUTEX_LOCK();
  ets_delay_us(1);
  gpio_set_level(SI4702_RST_GPIO, 0);
  ets_delay_us(1);
  gpio_set_level(SI4702_RST_GPIO, 1);
  BOARD_I2C_MUTEX_UNLOCK();

  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(TAG, "I2C bus is not initialized");
    return ESP_FAIL;
  }

  BOARD_I2C_MUTEX_LOCK();
  ret = i2c_master_probe(i2c_bus, SI4702_ADDR, 100);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_probe failed");

  i2c_device_config_t i2c_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = SI4702_ADDR,
      .scl_speed_hz = 400000,
  };

  BOARD_I2C_MUTEX_LOCK();
  ret = i2c_master_bus_add_device(i2c_bus, &i2c_cfg, &i2c_device);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_bus_add_device failed");

  ESP_RETURN_ON_ERROR(fm_read_registers(-1), TAG, "Failed to read registers");

  xSemaphoreTake(si4702_mutex, portMAX_DELAY);
  si4702_shadow_registers.reg02.refined.DSMUTE = 1;
  si4702_shadow_registers.reg02.refined.DMUTE = 1;
  si4702_shadow_registers.reg02.refined.MONO = 1;
  si4702_shadow_registers.reg05.refined.VOLUME = 0xf;
  ret = fm_flush_registers(0x05);
  xSemaphoreGive(si4702_mutex);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set configuration");

  ESP_RETURN_ON_ERROR(fm_read_registers(-1), TAG, "Failed to read registers");
  for (int i = 0; i < 16; i++) {
    ESP_LOGI(TAG, "Register 0x%02x: 0x%04x", i, si4702_shadow_registers.raw[i]);
  }

  things_register_telemetry_generator(fm_telemetry_generator, "fm",
                                      &telemetry_index);

  return ESP_OK;
}

esp_err_t fm_enable(void) {
  xSemaphoreTake(si4702_mutex, portMAX_DELAY);

  // Enable the radio
  si4702_shadow_registers.reg02.refined.ENABLE = 1;
  si4702_shadow_registers.reg02.refined.DISABLE = 0;
  esp_err_t ret = fm_flush_registers(0x02);
  xSemaphoreGive(si4702_mutex);
  things_force_telemetry(telemetry_index);

  // Wait 110ms for power up to complete
  vTaskDelay(pdMS_TO_TICKS(110));

  return ret;
}

esp_err_t fm_disable(void) {
  xSemaphoreTake(si4702_mutex, portMAX_DELAY);

  // Disable the radio
  si4702_shadow_registers.reg02.refined.ENABLE = 1;
  si4702_shadow_registers.reg02.refined.DISABLE = 1;
  esp_err_t ret = fm_flush_registers(0x02);
  xSemaphoreGive(si4702_mutex);
  things_force_telemetry(telemetry_index);
  return ret;
}

esp_err_t fm_tune(uint16_t channel) {
  xSemaphoreTake(si4702_mutex, portMAX_DELAY);

  esp_err_t ret = ESP_OK;

  si4702_shadow_registers.reg03.refined.TUNE = 1;
  si4702_shadow_registers.reg03.refined.CHAN = channel;
  ret = fm_flush_registers(0x3);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to set channel");

  // Wait for STC to go high. Theoretically this can be a maximum of 60ms per
  // channel (which could be way more than 1s), but we'll make some optimistic
  // assumptions
  uint64_t deadline = esp_timer_get_time() + 1000000;
  do {
    ret = fm_read_registers(0xa);
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to fetch status register");
    vTaskDelay(1);
  } while (!si4702_shadow_registers.reg0a.refined.STC &&
           esp_timer_get_time() < deadline);

  ESP_GOTO_ON_FALSE(si4702_shadow_registers.reg0a.refined.STC, ESP_FAIL,
                    cleanup, TAG,
                    "Failed to tune to channel %d (status reg: %" PRIx16 ")",
                    channel, si4702_shadow_registers.reg0a.raw);

  si4702_shadow_registers.reg03.refined.TUNE = 0;
  ret = fm_flush_registers(0x3);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to clear TUNE register");

cleanup:
  xSemaphoreGive(si4702_mutex);
  things_force_telemetry(telemetry_index);
  return ret;
}
