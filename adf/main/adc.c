#include "adc.h"

static adc_oneshot_unit_handle_t adc_unit_handle = NULL;

esp_err_t adc_init()
{
  adc_oneshot_unit_init_cfg_t cfg = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
      .ulp_mode = false,
  };
  esp_err_t err = adc_oneshot_new_unit(&cfg, &adc_unit_handle);

  return err;
}

adc_oneshot_unit_handle_t adc_get_handle()
{
  return adc_unit_handle;
}
