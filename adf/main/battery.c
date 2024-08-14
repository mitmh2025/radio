#include "battery.h"
#include "things.h"
#include "main.h"

#include "board.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t battery_adc_unit;
static adc_cali_handle_t battery_adc_cali;

static void battery_telemetry_generator()
{
  int battery_mv = 0;

  esp_err_t err = adc_oneshot_get_calibrated_result(battery_adc_unit, battery_adc_cali, BATTERY_ADC_CHANNEL, &battery_mv);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to get calibrated ADC result");
    return;
  }

  things_send_telemetry_double("battery_voltage", battery_mv * BATTERY_SCALE_FACTOR);
}

esp_err_t battery_init()
{
  adc_oneshot_unit_init_cfg_t cfg = {
    .unit_id = ADC_UNIT_1,
    .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    .ulp_mode = false,
  };
  esp_err_t err = adc_oneshot_new_unit(&cfg, &battery_adc_unit);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to initialize ADC unit");
    return err;
  }

  adc_channel_t channel = BATTERY_ADC_CHANNEL;
  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  err = adc_oneshot_config_channel(battery_adc_unit, channel, &chan_cfg);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to configure ADC channel");
    return err;
  }

  adc_cali_curve_fitting_config_t cali_cfg = {
    .atten = chan_cfg.atten,
    .bitwidth = chan_cfg.bitwidth,
    .unit_id = cfg.unit_id,
    .chan = channel,
  };
  err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &battery_adc_cali);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to create calibration scheme");
    return err;
  }

  things_register_telemetry_generator(battery_telemetry_generator);

  return ESP_OK;
}
