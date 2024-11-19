#include "battery.h"
#include "adc.h"
#include "led.h"
#include "main.h"
#include "things.h"

#include "board.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"

#define DISCHARGE_LOW_BATTERY_THRESHOLD_MV                                     \
  (2500) // 3.7V is roughly 20% of an 18650 * our 2/3 voltage divider
#define CHARGE_LOW_BATTERY_THRESHOLD                                           \
  (2667) // 4V * 2/3 is roughly 50% charged, but we want to encourage leaving
         // the radio plugged in
static uint32_t discharge_low_battery_threshold_raw = 0;
static uint32_t charge_low_battery_threshold_raw = 0;

typedef enum {
  BATTERY_CHARGED,
  BATTERY_CHARGING,
  BATTERY_DISCHARGING,
} battery_status_t;

struct color {
  uint32_t red;
  uint32_t green;
  uint32_t blue;
};

static struct color charged_color = {
    .red = 10,
    .green = 64,
    .blue = 0,
};

static struct color charging_color = {
    .red = 64,
    .green = 25,
    .blue = 0,
};

static struct color low_color = {
    .red = 64,
    .green = 0,
    .blue = 0,
};

static struct color discharging_color = {
    .red = 0,
    .green = 0,
    .blue = 0,
};

// TODO: these should be atomics
static uint32_t battery_average = 0;
static bool battery_low = false;
static battery_status_t battery_status = BATTERY_DISCHARGING;

static adc_cali_handle_t battery_adc_cali;
static i2c_master_dev_handle_t battery_i2c_device;
static StaticTask_t battery_monitor_task_buffer;
static EXT_RAM_BSS_ATTR StackType_t battery_monitor_task_stack[4096];
static TaskHandle_t battery_monitor_task_handle = NULL;
static size_t battery_telemetry_index = 0;

static void battery_update_led() {
  if (battery_low) {
    led_set_pixel(0, low_color.red, low_color.green, low_color.blue);
  } else {
    switch (battery_status) {
    case BATTERY_CHARGED:
      led_set_pixel(0, charged_color.red, charged_color.green,
                    charged_color.blue);
      break;
    case BATTERY_CHARGING:
      led_set_pixel(0, charging_color.red, charging_color.green,
                    charging_color.blue);
      break;
    case BATTERY_DISCHARGING:
      led_set_pixel(0, discharging_color.red, discharging_color.green,
                    discharging_color.blue);
      break;
    default:
      break;
    }
  }
}

static void battery_adc_callback(void *user_data,
                                 adc_digi_output_data_t *result) {
  if (battery_average == 0) {
    battery_average = result->type2.data;
    return;
  }

  battery_average =
      battery_average - (battery_average >> 3) + (result->type2.data >> 3);

  if (battery_status == BATTERY_DISCHARGING &&
      battery_average < discharge_low_battery_threshold_raw && !battery_low) {
    battery_low = true;
    if (battery_monitor_task_handle != NULL) {
      xTaskNotify(battery_monitor_task_handle, 0, eNoAction);
    }
  } else if (battery_status != BATTERY_DISCHARGING &&
             battery_average > charge_low_battery_threshold_raw &&
             battery_low) {
    battery_low = false;
    if (battery_monitor_task_handle != NULL) {
      xTaskNotify(battery_monitor_task_handle, 0, eNoAction);
    }
  }
}

static void battery_telemetry_generator() {
  int millivolts;
  esp_err_t err =
      adc_cali_raw_to_voltage(battery_adc_cali, battery_average, &millivolts);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to convert ADC reading to voltage: %d", err);
  } else {
    things_send_telemetry_float("battery_voltage",
                                (float)millivolts * BATTERY_SCALE_FACTOR);
  }

  const char *status;
  switch (battery_status) {
  case BATTERY_CHARGED:
    status = "charged";
    break;
  case BATTERY_CHARGING:
    status = "charging";
    break;
  case BATTERY_DISCHARGING:
    status = "discharging";
    break;
  default:
    status = "unknown";
    break;
  }
  things_send_telemetry_string("battery_status", status);
}

static void battery_monitor(void *context) {
  while (true) {
    uint8_t addr = IP5306_REG_READ0;
    ip5306_reg_read0_t reg0_value;
    ip5306_reg_read1_t reg1_value;
    BOARD_I2C_MUTEX_LOCK();
    esp_err_t err = i2c_master_transmit_receive(battery_i2c_device, &addr, 1,
                                                &reg0_value.raw, 1, -1);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to read IP5306 register 0");
      goto cleanup;
    }
    addr = IP5306_REG_READ1;
    err = i2c_master_transmit_receive(battery_i2c_device, &addr, 1,
                                      &reg1_value.raw, 1, -1);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to read IP5306 register 1");
      goto cleanup;
    }

  cleanup:
    BOARD_I2C_MUTEX_UNLOCK();
    if (err == ESP_OK) {
      battery_status_t new_status =
          !reg0_value.parsed.CHARGE_ENABLE ? BATTERY_DISCHARGING
          : reg1_value.parsed.CHARGING     ? BATTERY_CHARGING
                                           : BATTERY_CHARGED;
      if (new_status != battery_status) {
        battery_status = new_status;
        things_force_telemetry(battery_telemetry_index);
      }
    }

    battery_update_led();
    xTaskNotifyWait(0, ULONG_MAX, NULL,
                    pdMS_TO_TICKS(1000 + esp_random() % 100));
  }
}

esp_err_t battery_init() {
  adc_digi_pattern_config_t adc_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .channel = BATTERY_ADC_CHANNEL,
      .unit = ADC_UNIT_1,
      .bit_width = 12,
  };
  adc_cali_curve_fitting_config_t cali_cfg = {
      .atten = adc_cfg.atten,
      .bitwidth = adc_cfg.bit_width,
      .unit_id = adc_cfg.unit,
      .chan = adc_cfg.channel,
  };
  ESP_RETURN_ON_ERROR(
      adc_cali_create_scheme_curve_fitting(&cali_cfg, &battery_adc_cali),
      RADIO_TAG, "Failed to create calibration scheme");

  ESP_RETURN_ON_ERROR(adc_subscribe(&adc_cfg, battery_adc_callback, NULL),
                      RADIO_TAG, "Failed to subscribe to ADC channel");

  // Figure out at what raw value we consider the battery low (note that we
  // shouldn't get anywhere near max value)
  for (uint32_t i = 0; i < (1 << 12); i++) {
    int millivolts;
    adc_cali_raw_to_voltage(battery_adc_cali, i, &millivolts);

    if (discharge_low_battery_threshold_raw == 0 &&
        millivolts > DISCHARGE_LOW_BATTERY_THRESHOLD_MV) {
      discharge_low_battery_threshold_raw = i - 1;
    }
    if (charge_low_battery_threshold_raw == 0 &&
        millivolts > CHARGE_LOW_BATTERY_THRESHOLD) {
      charge_low_battery_threshold_raw = i - 1;
    }

    if (discharge_low_battery_threshold_raw != 0 &&
        charge_low_battery_threshold_raw != 0) {
      break;
    }
  }

  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL) {
    ESP_LOGE(RADIO_TAG, "I2C bus is not initialized");
    return ESP_FAIL;
  }

  BOARD_I2C_MUTEX_LOCK();
  esp_err_t err = i2c_master_probe(i2c_bus, IP5306_I2C_ADDR, 100);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "i2c_master_probe failed");

  i2c_device_config_t i2c_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = IP5306_I2C_ADDR,
      .scl_speed_hz = 400000,
  };

  BOARD_I2C_MUTEX_LOCK();
  err = i2c_master_bus_add_device(i2c_bus, &i2c_cfg, &battery_i2c_device);
  BOARD_I2C_MUTEX_UNLOCK();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "i2c_master_bus_add_device failed");

  battery_monitor_task_handle = xTaskCreateStatic(
      battery_monitor, "battery_monitor",
      sizeof(battery_monitor_task_stack) / sizeof(StackType_t), NULL, 5,
      battery_monitor_task_stack, &battery_monitor_task_buffer);
  ESP_RETURN_ON_FALSE(battery_monitor_task_handle, ESP_FAIL, RADIO_TAG,
                      "Failed to create battery monitor task");
  things_register_telemetry_generator(battery_telemetry_generator, "battery",
                                      &battery_telemetry_index);

  return ESP_OK;
}
