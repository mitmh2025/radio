#include "magnet.h"

#include "board.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "radio:magnet";

#define TMAG5273_I2C_ADDR (0x35)

#define TMAG5273_REG_DEVICE_CONFIG_1 (0x0)
#define TMAG5273_REG_DEVICE_CONFIG_2 (0x1)
#define TMAG5273_REG_SENSOR_CONFIG_1 (0x2)
#define TMAG5273_REG_T_CONFIG (0x7)
#define TMAG5273_REG_DEVICE_ID (0xd)
#define TMAG5273_REG_MANUFACTURER_ID_LSB (0xe)
#define TMAG5273_REG_MANUFACTURER_ID_MSB (0xf)
#define TMAG5273_REG_DATA_X_MSB (0x12)
#define TMAG5273_REG_DATA_X_LSB (0x13)
#define TMAG5273_REG_DATA_Y_MSB (0x14)
#define TMAG5273_REG_DATA_Y_LSB (0x15)
#define TMAG5273_REG_DATA_Z_MSB (0x16)
#define TMAG5273_REG_DATA_Z_LSB (0x17)
#define TMAG5273_REG_CONV_STATUS (0x18)
#define TMAG5273_REG_DEVICE_STATUS (0x1c)

typedef enum {
  TMAG5273_CONV_AVG_1X = 0,
  TMAG5273_CONV_AVG_2X = 1,
  TMAG5273_CONV_AVG_4X = 2,
  TMAG5273_CONV_AVG_8X = 3,
  TMAG5273_CONV_AVG_16X = 4,
  TMAG5273_CONV_AVG_32X = 5,
} tmag5273_conv_avg_t;

typedef enum {
  TMAG5273_I2C_READ_MODE_STD = 0,
  TMAG5273_I2C_READ_MODE_1BYTE_16BIT = 1,
  TMAG5273_I2C_READ_MODE_1BYTE_8BIT = 2,
} tmag5273_i2c_read_mode_t;

typedef enum {
  TMAG5273_OPERATING_MODE_STANDBY = 0,
  TMAG5273_OPERATING_MODE_SLEEP = 1,
  TMAG5273_OPERATING_MODE_CONTINUOUS = 2,
  TMAG5273_OPERATING_MODE_WAKE_UP_AND_SLEEP = 3,
} tmag5273_operating_mode_t;

typedef enum {
  TMAG5723_MAG_CHAN_OFF = 0,
  TMAG5723_MAG_CHAN_X = 1,
  TMAG5723_MAG_CHAN_Y = 2,
  TMAG5723_MAG_CHAN_XY = 3,
  TMAG5723_MAG_CHAN_Z = 4,
  TMAG5723_MAG_CHAN_ZX = 5,
  TMAG5723_MAG_CHAN_YZ = 6,
  TMAG5723_MAG_CHAN_XYZ = 7,
  TMAG5723_MAG_CHAN_XYX = 8,
  TMAG5723_MAG_CHAN_YXY = 9,
  TMAG5723_MAG_CHAN_YZY = 10,
  TMAG5723_MAG_CHAN_XZX = 11,
} tmag5723_mag_chan_t;

typedef enum {
  TMAG5273_SLEEPTIME_1MS = 0,
  TMAG5273_SLEEPTIME_5MS = 1,
  TMAG5273_SLEEPTIME_10MS = 2,
  TMAG5273_SLEEPTIME_15MS = 3,
  TMAG5273_SLEEPTIME_20MS = 4,
  TMAG5273_SLEEPTIME_30MS = 5,
  TMAG5273_SLEEPTIME_50MS = 6,
  TMAG5273_SLEEPTIME_100MS = 7,
  TMAG5273_SLEEPTIME_500MS = 8,
  TMAG5273_SLEEPTIME_1000MS = 9,
  TMAG5273_SLEEPTIME_2000MS = 10,
  TMAG5273_SLEEPTIME_5000MS = 11,
  TMAG5273_SLEEPTIME_20000MS = 12,
} tmag5723_sleeptime_t;

typedef union {
  struct {
    tmag5273_i2c_read_mode_t i2c_rd : 2;
    tmag5273_conv_avg_t conv_avg : 3;
    uint8_t mag_tempco : 2;
    uint8_t crc_en : 1;
  };
  uint8_t raw;
} tmag5723_device_config_1_t;

typedef union {
  struct {
    tmag5273_operating_mode_t operating_mode : 2;
    uint8_t trigger_mode : 1;
    uint8_t i2c_glitch_filter : 1;
    uint8_t lp_ln : 1;
    uint8_t thr_hyst : 3;
  };
  uint8_t raw;
} tmag5723_device_config_2_t;

typedef union {
  struct {
    tmag5723_sleeptime_t sleeptime : 4;
    tmag5723_mag_chan_t mag_chan_en : 4;
  };
  uint8_t raw;
} tmag5723_sensor_config_1_t;

typedef union {
  struct {
    uint8_t result_status : 1;
    uint8_t diag_status : 1;
    uint8_t reserved : 2;
    uint8_t por : 1;
    uint8_t set_count : 3;
  };
  uint8_t raw;
} tmag5273_conv_status_t;

static i2c_master_dev_handle_t magnet_i2c_device;

esp_err_t magnet_init(void) {
  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(TAG, "I2C bus is not initialized");
    return ESP_FAIL;
  }

  BOARD_I2C_MUTEX_LOCK();
  esp_err_t ret = i2c_master_probe(i2c_bus, TMAG5273_I2C_ADDR, 100);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "i2c_master_probe failed");

  i2c_device_config_t i2c_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = TMAG5273_I2C_ADDR,
      .scl_speed_hz = 400000,
  };

  ret = i2c_master_bus_add_device(i2c_bus, &i2c_cfg, &magnet_i2c_device);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "i2c_master_bus_add_device failed");

  // Make sure we have the right device
  uint16_t id = 0;
  uint8_t scratch;
  ret = i2c_master_transmit_receive(
      magnet_i2c_device, (uint8_t[]){TMAG5273_REG_MANUFACTURER_ID_MSB}, 1,
      &scratch, 1, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to read ID register");
  id = scratch << 8;
  ret = i2c_master_transmit_receive(
      magnet_i2c_device, (uint8_t[]){TMAG5273_REG_MANUFACTURER_ID_LSB}, 1,
      (uint8_t *)&scratch, 1, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to read ID register");
  id |= scratch;

  ESP_GOTO_ON_FALSE(id == 0x5449, ESP_FAIL, cleanup, TAG,
                    "Unexpected manufacturer ID: 0x%04x", id);

  ret = i2c_master_transmit_receive(magnet_i2c_device,
                                    (uint8_t[]){TMAG5273_REG_DEVICE_ID}, 1,
                                    &scratch, 1, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to read ID register");

  ESP_GOTO_ON_FALSE((scratch & 0x3) == 1, ESP_FAIL, cleanup, TAG,
                    "Unexpected device ID: 0x%02x", scratch);

  // Configure the magnetometer
  tmag5723_sensor_config_1_t sensor_config_1 = {
      .mag_chan_en = TMAG5723_MAG_CHAN_XYZ,
  };
  ret = i2c_master_transmit(
      magnet_i2c_device,
      (uint8_t[]){TMAG5273_REG_SENSOR_CONFIG_1, sensor_config_1.raw}, 2, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to write sensor config");

  ret = i2c_master_transmit(magnet_i2c_device,
                            (uint8_t[]){TMAG5273_REG_T_CONFIG, 1}, 2, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to write temperature config");

  tmag5723_device_config_2_t device_config_2 = {
      .operating_mode = TMAG5273_OPERATING_MODE_CONTINUOUS,
  };
  ret = i2c_master_transmit(
      magnet_i2c_device,
      (uint8_t[]){TMAG5273_REG_DEVICE_CONFIG_2, device_config_2.raw}, 2, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to write device config 2");

  tmag5723_device_config_1_t device_config_1 = {
      .conv_avg = TMAG5273_CONV_AVG_1X,
  };
  ret = i2c_master_transmit(
      magnet_i2c_device,
      (uint8_t[]){TMAG5273_REG_DEVICE_CONFIG_1, device_config_1.raw}, 2, -1);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to write device config 1");

cleanup:
  BOARD_I2C_MUTEX_UNLOCK();
  return ret;
}

esp_err_t magnet_read(int16_t *x, int16_t *y, int16_t *z) {
  ESP_RETURN_ON_FALSE(x && y && z, ESP_ERR_INVALID_ARG, TAG,
                      "x, y, and z must not be NULL");

  uint8_t buffer[6];
  BOARD_I2C_MUTEX_LOCK();
  esp_err_t err = i2c_master_transmit_receive(
      magnet_i2c_device, (uint8_t[]){TMAG5273_REG_DATA_X_MSB}, 1, buffer, 6,
      -1);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(err, TAG, "Failed to read magnetometer data");

  *x = (buffer[0] << 8) | buffer[1];
  *y = (buffer[2] << 8) | buffer[3];
  *z = (buffer[4] << 8) | buffer[5];

  return ESP_OK;
}