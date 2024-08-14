#include "battery.h"
#include "things.h"
#include "main.h"

#include "board.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t battery_adc_unit;
static adc_cali_handle_t battery_adc_cali;
static i2c_master_dev_handle_t battery_i2c_device;

static void battery_telemetry_generator()
{
  int battery_sample_sum = 0;
  int battery_sample_count = 64;
  esp_err_t err = ESP_OK;

  for (int i = 0; i < battery_sample_count; i++)
  {
    int sample;
    err = adc_oneshot_get_calibrated_result(battery_adc_unit, battery_adc_cali, BATTERY_ADC_CHANNEL, &sample);
    if (err != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to get raw ADC sample");
      return;
    }
    battery_sample_sum += sample;
  }

  things_send_telemetry_double("battery_voltage", (battery_sample_sum / (float)battery_sample_count) * BATTERY_SCALE_FACTOR);

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
                                 !reg1_value.parsed.CHARGING ? "charged" : reg0_value.parsed.CHARGE_ENABLE ? "charging"
                                                                                                             : "discharging");
  }
}

esp_err_t battery_init()
{
  adc_oneshot_unit_init_cfg_t cfg = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
      .ulp_mode = false,
  };
  esp_err_t err = adc_oneshot_new_unit(&cfg, &battery_adc_unit);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to create ADC unit");

  adc_channel_t channel = BATTERY_ADC_CHANNEL;
  adc_oneshot_chan_cfg_t chan_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  err = adc_oneshot_config_channel(battery_adc_unit, channel, &chan_cfg);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to configure ADC channel");

  adc_cali_curve_fitting_config_t cali_cfg = {
      .atten = chan_cfg.atten,
      .bitwidth = chan_cfg.bitwidth,
      .unit_id = cfg.unit_id,
      .chan = channel,
  };
  err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &battery_adc_cali);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to create calibration scheme");

  i2c_master_bus_handle_t i2c_bus = board_i2c_get_handle();
  if (i2c_bus == NULL)
  {
    ESP_LOGE(RADIO_TAG, "I2C bus is not initialized");
    return ESP_FAIL;
  }

  BOARD_I2C_MUTEX_LOCK();
  err = i2c_master_probe(i2c_bus, IP5306_I2C_ADDR, 100);
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
