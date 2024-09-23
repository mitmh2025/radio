#include "battery.h"
#include "things.h"
#include "main.h"
#include "adc.h"

#include "board.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static uint32_t battery_average = 0;
static adc_cali_handle_t battery_adc_cali;
static i2c_master_dev_handle_t battery_i2c_device;

static void battery_adc_callback(adc_digi_output_data_t *result)
{
  if (battery_average == 0)
  {
    battery_average = result->type2.data;
    return;
  }

  battery_average = battery_average - (battery_average >> 3) + (result->type2.data >> 3);
}

static void battery_telemetry_generator()
{
  int millivolts;
  esp_err_t err = adc_cali_raw_to_voltage(battery_adc_cali, battery_average, &millivolts);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to convert ADC reading to voltage: %d (%s)", err, esp_err_to_name(err));
  }
  else
  {
    things_send_telemetry_float("battery_voltage", (float)millivolts * BATTERY_SCALE_FACTOR);
  }

  uint8_t addr = IP5306_REG_READ0;
  ip5306_reg_read0_t reg0_value;
  ip5306_reg_read1_t reg1_value;
  BOARD_I2C_MUTEX_LOCK();
  err = i2c_master_transmit_receive(battery_i2c_device, &addr, 1, &reg0_value.raw, 1, -1);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to read IP5306 register 0");
    goto cleanup;
  }
  addr = IP5306_REG_READ1;
  err = i2c_master_transmit_receive(battery_i2c_device, &addr, 1, &reg1_value.raw, 1, -1);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to read IP5306 register 1");
    goto cleanup;
  }

cleanup:
  BOARD_I2C_MUTEX_UNLOCK();

  if (err == ESP_OK)
  {
    things_send_telemetry_string("battery_status",
                                 !reg0_value.parsed.CHARGE_ENABLE ? "discharging" : reg1_value.parsed.CHARGING ? "charging"
                                                                                                               : "charged");
  }
}

esp_err_t battery_init()
{
  adc_digi_pattern_config_t adc_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .channel = BATTERY_ADC_CHANNEL,
      .unit = ADC_UNIT_1,
      .bit_width = 12,
  };
  ESP_RETURN_ON_ERROR(adc_subscribe(&adc_cfg, battery_adc_callback), RADIO_TAG, "Failed to subscribe to ADC channel");

  adc_cali_curve_fitting_config_t cali_cfg = {
      .atten = adc_cfg.atten,
      .bitwidth = adc_cfg.bit_width,
      .unit_id = adc_cfg.unit,
      .chan = adc_cfg.channel,
  };
  ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_cfg, &battery_adc_cali), RADIO_TAG, "Failed to create calibration scheme");

  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL)
  {
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

  things_register_telemetry_generator(battery_telemetry_generator);

  return ESP_OK;
}
