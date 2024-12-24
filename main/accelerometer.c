#include "accelerometer.h"
#include "things.h"

#include "board.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "radio:accel";
static i2c_master_dev_handle_t i2c_device;

#define I2C_ADDR 0x1c

#define MMA8451Q_REG_STATUS 0x00
#define MMA8451Q_REG_INT_SOURCE 0x0c
#define MMA8451Q_REG_WHO_AM_I 0x0d
#define MMA8451Q_REG_PL_STATUS 0x10
#define MMA8451Q_REG_PL_CFG 0x11
#define MMA8451Q_REG_CTRL_REG1 0x2a
#define MMA8451Q_REG_CTRL_REG2 0x2b
#define MMA8451Q_REG_CTRL_REG3 0x2c
#define MMA8451Q_REG_CTRL_REG4 0x2d
#define MMA8451Q_REG_CTRL_REG5 0x2e
#define MMA8451Q_REG_PULSE_CFG 0x21
#define MMA8451Q_REG_PULSE_SRC 0x22
#define MMA8451Q_REG_PULSE_THSX 0x23
#define MMA8451Q_REG_PULSE_THSY 0x24
#define MMA8451Q_REG_PULSE_THSZ 0x25
#define MMA8451Q_REG_PULSE_TMLT 0x26
#define MMA8451Q_REG_PULSE_LTCY 0x27

typedef union {
  struct {
    uint8_t bafro : 1;
    uint8_t lapo : 2;
    uint8_t reserved : 3;
    uint8_t lo : 1;
    uint8_t newlp : 1;
  } refined;
  uint8_t raw;
} mma8451q_pl_status_t;
typedef union {
  struct {
    uint8_t reserved : 6;
    uint8_t pl_en : 1;
    uint8_t dbcntm : 1;
  } refined;
  uint8_t raw;
} mma8451q_pl_cfg_t;
typedef union {
  struct {
    uint8_t active : 1;
    uint8_t f_read : 1;
    uint8_t lnoise : 1;
    uint8_t dr : 3;
    uint8_t aslp_rate : 2;
  } refined;
  uint8_t raw;
} mma8451q_ctrl_reg1_t;
typedef union {
  struct {
    uint8_t mods : 2;
    uint8_t slpe : 1;
    uint8_t smods : 2;
    uint8_t reserved : 1;
    uint8_t rst : 1;
    uint8_t st : 1;
  } refined;
  uint8_t raw;
} mma8451q_ctrl_reg2_t;
typedef union {
  struct {
    uint8_t pp_od : 1;
    uint8_t ipol : 1;
    uint8_t reserved : 1;
    uint8_t wake_ff_mt : 1;
    uint8_t wake_pulse : 1;
    uint8_t wake_lndprt : 1;
    uint8_t wake_trans : 1;
    uint8_t fifo_gate : 1;
  } refined;
  uint8_t raw;
} mma8451q_ctrl_reg3_t;
typedef union {
  struct {
    uint8_t int_en_drdy : 1;
    uint8_t reserved : 1;
    uint8_t int_en_ff_mt : 1;
    uint8_t int_en_pulse : 1;
    uint8_t int_en_lndprt : 1;
    uint8_t int_en_trans : 1;
    uint8_t int_en_fifo : 1;
    uint8_t int_en_aslp : 1;
  } refined;
  uint8_t raw;
} mma8451q_ctrl_reg4_t;
typedef union {
  struct {
    uint8_t int_cfg_drdy : 1;
    uint8_t reserved : 1;
    uint8_t int_cfg_ff_mt : 1;
    uint8_t int_cfg_pulse : 1;
    uint8_t int_cfg_lndprt : 1;
    uint8_t int_cfg_trans : 1;
    uint8_t int_cfg_fifo : 1;
    uint8_t int_cfg_aslp : 1;
  } refined;
  uint8_t raw;
} mma8451q_ctrl_reg5_t;
typedef union {
  struct {
    uint8_t src_drdy : 1;
    uint8_t reserved : 1;
    uint8_t src_ff_mt : 1;
    uint8_t src_pulse : 1;
    uint8_t src_lndprt : 1;
    uint8_t src_trans : 1;
    uint8_t src_fifo : 1;
    uint8_t src_aslp : 1;
  } refined;
  uint8_t raw;
} mma8451q_int_src_t;
typedef union {
  struct {
    uint8_t xspefe : 1;
    uint8_t xdpefe : 1;
    uint8_t yspefe : 1;
    uint8_t ydpefe : 1;
    uint8_t zspefe : 1;
    uint8_t zdpefe : 1;
    uint8_t ele : 1;
    uint8_t dpa : 1;
  } refined;
  uint8_t raw;
} mma8451q_pulse_cfg_t;
typedef union {
  struct {
    uint8_t pol_x : 1;
    uint8_t pol_y : 1;
    uint8_t pol_z : 1;
    uint8_t dpe : 1;
    uint8_t ax_x : 1;
    uint8_t ax_y : 1;
    uint8_t ax_z : 1;
    uint8_t ea : 1;
  } refined;
  uint8_t raw;
} mma8451q_pulse_src_t;

static TaskHandle_t task_handle = NULL;
static SemaphoreHandle_t mutex = NULL;
static bool active = false;

static accelerometer_pulse_callback_t pulse_callback;
static void *pulse_arg;

static accelerometer_data_callback_t data_callback;
static void *data_arg;

static void telemetry_generator() {
  things_send_telemetry_bool("accel_active", active);
  things_send_telemetry_bool("accel_pulse_active", pulse_callback != NULL);
  things_send_telemetry_bool("accel_data_active", data_callback != NULL);
}

static void accelerometer_task(void *ctx) {
  while (true) {
    while (gpio_get_level(MMA8451Q_INT1_GPIO) == 1) {
      xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(1000));
    }

    // Wait for the interrupt to clear
    while (gpio_get_level(MMA8451Q_INT1_GPIO) == 0) {
      mma8451q_int_src_t int_src;
      BOARD_I2C_MUTEX_LOCK();
      esp_err_t err = i2c_master_transmit_receive(
          i2c_device, (uint8_t[]){MMA8451Q_REG_INT_SOURCE}, 1, &int_src.raw, 1,
          -1);
      BOARD_I2C_MUTEX_UNLOCK();
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INT_SOURCE register: %d", err);
        continue;
      }

      if (int_src.refined.src_pulse) {
        // Clear the interrupt
        mma8451q_pulse_src_t pulse_src;
        BOARD_I2C_MUTEX_LOCK();
        err = i2c_master_transmit_receive(i2c_device,
                                          (uint8_t[]){MMA8451Q_REG_PULSE_SRC},
                                          1, &pulse_src.raw, 1, -1);
        BOARD_I2C_MUTEX_UNLOCK();
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to read PULSE_SRC register: %d", err);
          continue;
        }

        accelerometer_pulse_axis_t axis = 0;
        if (pulse_src.refined.ax_x) {
          axis |= (pulse_src.refined.pol_x ? ACCELEROMETER_PULSE_AXIS_X_NEG
                                           : ACCELEROMETER_PULSE_AXIS_X_POS);
        }
        if (pulse_src.refined.ax_y) {
          axis |= (pulse_src.refined.pol_y ? ACCELEROMETER_PULSE_AXIS_Y_NEG
                                           : ACCELEROMETER_PULSE_AXIS_Y_POS);
        }
        if (pulse_src.refined.ax_z) {
          axis |= (pulse_src.refined.pol_z ? ACCELEROMETER_PULSE_AXIS_Z_NEG
                                           : ACCELEROMETER_PULSE_AXIS_Z_POS);
        }

        xSemaphoreTake(mutex, portMAX_DELAY);
        accelerometer_pulse_callback_t cb = pulse_callback;
        void *arg = pulse_arg;
        xSemaphoreGive(mutex);
        if (cb) {
          cb(axis, arg);
        }

        int_src.refined.src_pulse = 0;
      }

      if (int_src.refined.src_drdy) {
        // Read data to clear the interrupt
        int16_t x, y, z;
        err = accelerometer_read_data(&x, &y, &z);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to read accelerometer data: %d", err);
          continue;
        }

        xSemaphoreTake(mutex, portMAX_DELAY);
        accelerometer_data_callback_t cb = data_callback;
        void *arg = data_arg;
        xSemaphoreGive(mutex);

        if (cb) {
          cb(x, y, z, arg);
        }

        int_src.refined.src_drdy = 0;
      }

      if (int_src.raw != 0) {
        ESP_LOGW(TAG, "Unhandled interrupt source: 0x%02x", int_src.raw);
      }

      xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(1000));
    }
  }
}

static void IRAM_ATTR accelerometer_isr(void *arg) {
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, 0, eNoAction, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static esp_err_t read_register(uint8_t reg, uint8_t *value) {
  BOARD_I2C_MUTEX_LOCK();
  esp_err_t ret =
      i2c_master_transmit_receive(i2c_device, &reg, 1, value, 1, -1);
  BOARD_I2C_MUTEX_UNLOCK();
  return ret;
}

static esp_err_t write_register(uint8_t reg, uint8_t value) {
  BOARD_I2C_MUTEX_LOCK();
  esp_err_t ret =
      i2c_master_transmit(i2c_device, (uint8_t[]){reg, value}, 2, -1);
  BOARD_I2C_MUTEX_UNLOCK();
  return ret;
}

static bool should_activate() { return pulse_callback || data_callback; }

// Must be called with mutex held
static esp_err_t set_active(bool new_status) {
  mma8451q_ctrl_reg1_t ctrl_reg1 = {};

  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG1, &ctrl_reg1.raw),
                    cleanup, TAG, "Failed to read CTRL_REG1 register");
  ctrl_reg1.refined.active = new_status ? 1 : 0;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG1, ctrl_reg1.raw),
                    cleanup, TAG, "Failed to write CTRL_REG1 register");

  active = new_status;

cleanup:
  return ret;
}

esp_err_t accelerometer_init() {
  esp_err_t ret = ESP_OK;

  mutex = xSemaphoreCreateMutex();
  if (mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return ESP_FAIL;
  }

  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(TAG, "I2C bus is not initialized");
    return ESP_FAIL;
  }

  BOARD_I2C_MUTEX_LOCK();
  ret = i2c_master_probe(i2c_bus, I2C_ADDR, 100);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_probe failed");

  i2c_device_config_t i2c_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = I2C_ADDR,
      .scl_speed_hz = 400000,
  };

  BOARD_I2C_MUTEX_LOCK();
  ret = i2c_master_bus_add_device(i2c_bus, &i2c_cfg, &i2c_device);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(ret, TAG, "i2c_master_bus_add_device failed");

  // Make sure we have the right device
  uint8_t who_am_i;
  ESP_RETURN_ON_ERROR(read_register(MMA8451Q_REG_WHO_AM_I, &who_am_i), TAG,
                      "Failed to read WHO_AM_I register");
  ESP_RETURN_ON_FALSE(who_am_i == 0x1a, ESP_FAIL, TAG,
                      "Unexpected WHO_AM_I value: 0x%02x", who_am_i);

  // Force a reset just to clear state
  mma8451q_ctrl_reg2_t ctrl_reg2 = {};
  ESP_RETURN_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG2, &ctrl_reg2.raw),
                      TAG, "Failed to read CTRL_REG2 register");
  ctrl_reg2.refined.rst = 1;
  ESP_RETURN_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG2, ctrl_reg2.raw),
                      TAG, "Failed to write CTRL_REG2 register");

  // Wait 500µs for reset to complete
  esp_rom_delay_us(500);

  mma8451q_ctrl_reg1_t ctrl_reg1 = {};
  ESP_RETURN_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG1, &ctrl_reg1.raw),
                      TAG, "Failed to read CTRL_REG1 register");
  ctrl_reg1.refined.dr = ACCELEROMETER_DR_800HZ;
  ESP_RETURN_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG1, ctrl_reg1.raw),
                      TAG, "Failed to write CTRL_REG1 register");

  ESP_RETURN_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG2, &ctrl_reg2.raw),
                      TAG, "Failed to read CTRL_REG2 register");
  ctrl_reg2.refined.mods = ACCELEROMETER_OSM_NORMAL;
  ESP_RETURN_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG2, ctrl_reg2.raw),
                      TAG, "Failed to write CTRL_REG2 register");

  // If the accelerometer is otherwise on, compute the orientation
  mma8451q_pl_cfg_t pl_cfg = {
      .refined =
          {
              .pl_en = 1,
          },
  };
  ESP_RETURN_ON_ERROR(write_register(MMA8451Q_REG_PL_CFG, pl_cfg.raw), TAG,
                      "Failed to write PL_CFG register");

  // We may not turn any of these interrupts on, but all interrupts go to INT1
  mma8451q_ctrl_reg5_t ctrl_reg5 = {.raw = 0xff};
  ESP_RETURN_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG5, ctrl_reg5.raw),
                      TAG, "Failed to write CTRL_REG5 register");

  ESP_RETURN_ON_FALSE(
      pdPASS == xTaskCreatePinnedToCore(accelerometer_task, "accelerometer",
                                        3072, NULL, 12, &task_handle, 0),
      ESP_FAIL, TAG, "Failed to create task");

  gpio_config_t cfg = {
      .pin_bit_mask = BIT64(MMA8451Q_INT1_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG,
                      "Failed to configure interrupt pin");
  ESP_RETURN_ON_ERROR(
      gpio_isr_handler_add(MMA8451Q_INT1_GPIO, accelerometer_isr, NULL), TAG,
      "Failed to add interrupt handler");

  things_register_telemetry_generator(telemetry_generator, "accel", NULL);

  return ESP_OK;
}

esp_err_t accelerometer_subscribe_pulse(const accelerometer_pulse_cfg_t *cfg,
                                        accelerometer_pulse_callback_t callback,
                                        void *arg) {
  ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");
  ESP_RETURN_ON_FALSE(callback, ESP_ERR_INVALID_ARG, TAG, "callback is NULL");
  ESP_RETURN_ON_FALSE(cfg->threshold_x < 0x80, ESP_ERR_INVALID_ARG, TAG,
                      "threshold_x must be less than 0x80");
  ESP_RETURN_ON_FALSE(cfg->threshold_y < 0x80, ESP_ERR_INVALID_ARG, TAG,
                      "threshold_y must be less than 0x80");
  ESP_RETURN_ON_FALSE(cfg->threshold_z < 0x80, ESP_ERR_INVALID_ARG, TAG,
                      "threshold_z must be less than 0x80");

  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(set_active(false), cleanup, TAG, "Failed to set inactive");

  mma8451q_pulse_cfg_t pulse_cfg = {
      .refined =
          {
              .xspefe = cfg->threshold_x > 0 ? 1 : 0,
              .yspefe = cfg->threshold_y > 0 ? 1 : 0,
              .zspefe = cfg->threshold_z > 0 ? 1 : 0,
              .ele = 1,
          },
  };
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_CFG, pulse_cfg.raw),
                    cleanup, TAG, "Failed to write PULSE_CFG register");

  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_THSX, cfg->threshold_x),
                    cleanup, TAG, "Failed to write PULSE_THSX register");
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_THSY, cfg->threshold_y),
                    cleanup, TAG, "Failed to write PULSE_THSY register");
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_THSZ, cfg->threshold_z),
                    cleanup, TAG, "Failed to write PULSE_THSZ register");
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_TMLT, cfg->timelimit),
                    cleanup, TAG, "Failed to write PULSE_TMLT register");
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_LTCY, cfg->latency),
                    cleanup, TAG, "Failed to write PULSE_LTCY register");

  mma8451q_ctrl_reg4_t ctrl_reg4 = {};
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG4, &ctrl_reg4.raw),
                    cleanup, TAG, "Failed to read CTRL_REG4 register");
  ctrl_reg4.refined.int_en_pulse = 1;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG4, ctrl_reg4.raw),
                    cleanup, TAG, "Failed to write CTRL_REG4 register");

  pulse_callback = callback;
  pulse_arg = arg;

  ESP_GOTO_ON_ERROR(set_active(true), cleanup, TAG, "Failed to set active");

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t accelerometer_unsubscribe_pulse(void) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(set_active(false), cleanup, TAG, "Failed to set inactive");

  mma8451q_ctrl_reg4_t ctrl_reg4 = {};
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG4, &ctrl_reg4.raw),
                    cleanup, TAG, "Failed to read CTRL_REG4 register");
  ctrl_reg4.refined.int_en_pulse = 0;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG4, ctrl_reg4.raw),
                    cleanup, TAG, "Failed to write CTRL_REG4 register");

  pulse_callback = NULL;
  pulse_arg = NULL;

  if (should_activate()) {
    ESP_GOTO_ON_ERROR(set_active(true), cleanup, TAG, "Failed to set active");
  }

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t accelerometer_subscribe_data(accelerometer_data_callback_t callback,
                                       void *arg) {
  ESP_RETURN_ON_FALSE(callback, ESP_ERR_INVALID_ARG, TAG, "callback is NULL");

  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(set_active(false), cleanup, TAG, "Failed to set inactive");

  mma8451q_ctrl_reg4_t ctrl_reg4 = {};
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG4, &ctrl_reg4.raw),
                    cleanup, TAG, "Failed to read CTRL_REG4 register");
  ctrl_reg4.refined.int_en_drdy = 1;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG4, ctrl_reg4.raw),
                    cleanup, TAG, "Failed to write CTRL_REG4 register");

  data_callback = callback;
  data_arg = arg;

  ESP_GOTO_ON_ERROR(set_active(true), cleanup, TAG, "Failed to set active");

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t accelerometer_unsubscribe_data(void) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(set_active(false), cleanup, TAG, "Failed to set inactive");

  mma8451q_ctrl_reg4_t ctrl_reg4 = {};
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG4, &ctrl_reg4.raw),
                    cleanup, TAG, "Failed to read CTRL_REG4 register");
  ctrl_reg4.refined.int_en_drdy = 0;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG4, ctrl_reg4.raw),
                    cleanup, TAG, "Failed to write CTRL_REG4 register");

  data_callback = NULL;
  data_arg = NULL;

  if (should_activate()) {
    ESP_GOTO_ON_ERROR(set_active(true), cleanup, TAG, "Failed to set active");
  }

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t
accelerometer_get_orientation(accelerometer_orientation_t *orientation) {
  ESP_RETURN_ON_FALSE(orientation, ESP_ERR_INVALID_ARG, TAG,
                      "orientation is NULL");

  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(active, ESP_FAIL, cleanup, TAG,
                    "Cannot get orientation while inactive");

  mma8451q_pl_status_t pl_status;
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_PL_STATUS, &pl_status.raw),
                    cleanup, TAG, "Failed to read PL_STATUS register");

  if (pl_status.refined.lo) {
    *orientation = pl_status.refined.bafro ? ACCELEROMETER_ORIENTATION_BACK_UP
                                           : ACCELEROMETER_ORIENTATION_FRONT_UP;
  } else {
    switch (pl_status.refined.lapo) {
    case 0:
      *orientation = ACCELEROMETER_ORIENTATION_TOP_UP;
      break;
    case 1:
      *orientation = ACCELEROMETER_ORIENTATION_BOTTOM_UP;
      break;
    case 2:
      *orientation = ACCELEROMETER_ORIENTATION_LEFT_UP;
      break;
    case 3:
      *orientation = ACCELEROMETER_ORIENTATION_RIGHT_UP;
      break;
    }
  }

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t accelerometer_read_data(int16_t *x, int16_t *y, int16_t *z) {
  ESP_RETURN_ON_FALSE(x && y && z, ESP_ERR_INVALID_ARG, TAG,
                      "x, y, and z must not be NULL");

  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(active, ESP_FAIL, cleanup, TAG,
                    "Cannot read data while inactive");

  uint8_t buffer[7];
  BOARD_I2C_MUTEX_LOCK();
  esp_err_t err = i2c_master_transmit_receive(
      i2c_device, (uint8_t[]){MMA8451Q_REG_STATUS}, 1, buffer, 7, -1);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_GOTO_ON_ERROR(err, cleanup, TAG, "Failed to read data");

  *x = (buffer[1] << 8) | buffer[2];
  *y = (buffer[3] << 8) | buffer[4];
  *z = (buffer[5] << 8) | buffer[6];

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

const char *
accelerometer_pulse_axis_to_string(accelerometer_pulse_axis_t axis) {
  switch (axis) {
  case ACCELEROMETER_PULSE_AXIS_X_POS:
    return "x+";
  case ACCELEROMETER_PULSE_AXIS_X_NEG:
    return "x-";
  case ACCELEROMETER_PULSE_AXIS_Y_POS:
    return "y+";
  case ACCELEROMETER_PULSE_AXIS_Y_NEG:
    return "y-";
  case ACCELEROMETER_PULSE_AXIS_Z_POS:
    return "z+";
  case ACCELEROMETER_PULSE_AXIS_Z_NEG:
    return "z-";
  default:
    return "unknown";
  }
}
