#include "accelerometer.h"
#include "things.h"

#include "board.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "radio:accel";
static i2c_master_dev_handle_t i2c_device;

#define I2C_ADDR 0x1c

#define MMA8451Q_REG_INT_SOURCE 0x0c
#define MMA8451Q_REG_WHO_AM_I 0x0d
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
bool active = false;
accelerometer_odr_t odr;
accelerometer_osm_t osm;
static void (*pulse_callback)(void *);
static void *pulse_arg;

static void telemetry_generator() {
  // TODO
}

static void accelerometer_task(void *ctx) {
  while (true) {
    BaseType_t woken = xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(1000));

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
                                        (uint8_t[]){MMA8451Q_REG_PULSE_SRC}, 1,
                                        &pulse_src.raw, 1, -1);
      BOARD_I2C_MUTEX_UNLOCK();
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PULSE_SRC register: %d", err);
        continue;
      }

      xSemaphoreTake(mutex, portMAX_DELAY);
      if (pulse_callback) {
        pulse_callback(pulse_arg);
      }
      xSemaphoreGive(mutex);

      int_src.refined.src_pulse = 0;
    }

    if (woken && int_src.raw != 0) {
      ESP_LOGW(TAG, "Unhandled interrupt source: 0x%02x", int_src.raw);
    }
  }
}

void IRAM_ATTR accelerometer_isr(void *arg) {
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, 0, eNoAction, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    portYIELD_FROM_ISR();
  }
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

// Must be called with mutex held
static esp_err_t set_active(bool new_status,
                            mma8451q_ctrl_reg1_t *final_ctrl_reg1) {
  mma8451q_ctrl_reg1_t ctrl_reg1 = {};

  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG1, &ctrl_reg1.raw),
                    cleanup, TAG, "Failed to read CTRL_REG1 register");
  ctrl_reg1.refined.active = new_status ? 1 : 0;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG1, ctrl_reg1.raw),
                    cleanup, TAG, "Failed to write CTRL_REG1 register");

  active = new_status;

cleanup:
  BOARD_I2C_MUTEX_UNLOCK();
  if (ret == ESP_OK && final_ctrl_reg1) {
    *final_ctrl_reg1 = ctrl_reg1;
  }
  return ret;
}

esp_err_t accelerometer_init() {
  esp_err_t ret = ESP_OK;

  mutex = xSemaphoreCreateMutex();
  if (mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return ESP_FAIL;
  }

  if (pdPASS != xTaskCreatePinnedToCore(accelerometer_task,
                                        "accelerometer_task", 3072, NULL, 12,
                                        &task_handle, 0)) {
    ESP_LOGE(TAG, "Failed to create task");
    return ESP_FAIL;
  }

  gpio_config_t cfg = {
      .pin_bit_mask = 1ULL << MMA8451Q_INT1_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG,
                      "Failed to configure interrupt pin");
  ESP_RETURN_ON_ERROR(
      gpio_isr_handler_add(MMA8451Q_INT1_GPIO, accelerometer_isr, NULL), TAG,
      "Failed to add interrupt handler");

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

  // We may not turn any of these interrupts on, but all interrupts go to INT1
  mma8451q_ctrl_reg5_t ctrl_reg5 = {.raw = 0xff};
  ESP_RETURN_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG5, ctrl_reg5.raw),
                      TAG, "Failed to write CTRL_REG5 register");

  things_register_telemetry_generator(telemetry_generator, "accel");

  return ESP_OK;
}

esp_err_t accelerometer_subscribe_pulse(accelerometer_pulse_cfg_t *cfg,
                                        void (*callback)(void *), void *arg) {
  ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");
  ESP_RETURN_ON_FALSE(callback, ESP_ERR_INVALID_ARG, TAG, "callback is NULL");
  ESP_RETURN_ON_FALSE(cfg->threshold < 0x80, ESP_ERR_INVALID_ARG, TAG,
                      "threshold must be less than 0x80");

  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(!active || (odr == cfg->odr && osm == cfg->osm), ESP_FAIL,
                    cleanup, TAG, "Cannot change ODR or OSM while active");

  mma8451q_ctrl_reg1_t ctrl_reg1 = {};
  ESP_GOTO_ON_ERROR(set_active(false, &ctrl_reg1), cleanup, TAG,
                    "Failed to set inactive");

  ctrl_reg1.refined.dr = cfg->odr;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG1, ctrl_reg1.raw),
                    cleanup, TAG, "Failed to write CTRL_REG1 register");

  mma8451q_ctrl_reg2_t ctrl_reg2 = {};
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG2, &ctrl_reg2.raw),
                    cleanup, TAG, "Failed to read CTRL_REG2 register");
  ctrl_reg2.refined.mods = cfg->osm;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG2, ctrl_reg2.raw),
                    cleanup, TAG, "Failed to write CTRL_REG2 register");

  mma8451q_pulse_cfg_t pulse_cfg = {
      .refined =
          {
              .xspefe = 1,
              .yspefe = 1,
              .zspefe = 1,
              .ele = 1,
          },
  };
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_CFG, pulse_cfg.raw),
                    cleanup, TAG, "Failed to write PULSE_CFG register");

  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_THSX, cfg->threshold),
                    cleanup, TAG, "Failed to write PULSE_THSX register");
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_THSY, cfg->threshold),
                    cleanup, TAG, "Failed to write PULSE_THSY register");
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_PULSE_THSZ, cfg->threshold),
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

  ESP_GOTO_ON_ERROR(set_active(true, NULL), cleanup, TAG,
                    "Failed to set active");

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}

esp_err_t accelerometer_unsubscribe_pulse(void) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(set_active(false, NULL), cleanup, TAG,
                    "Failed to set inactive");

  mma8451q_ctrl_reg4_t ctrl_reg4 = {};
  ESP_GOTO_ON_ERROR(read_register(MMA8451Q_REG_CTRL_REG4, &ctrl_reg4.raw),
                    cleanup, TAG, "Failed to read CTRL_REG4 register");
  ctrl_reg4.refined.int_en_pulse = 0;
  ESP_GOTO_ON_ERROR(write_register(MMA8451Q_REG_CTRL_REG4, ctrl_reg4.raw),
                    cleanup, TAG, "Failed to write CTRL_REG4 register");

  pulse_callback = NULL;
  pulse_arg = NULL;

  // TODO: decide whether we need to re-activate

cleanup:
  xSemaphoreGive(mutex);
  return ret;
}
