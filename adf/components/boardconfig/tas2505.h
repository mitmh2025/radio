#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define TAS2505_ADDR 0b0011000

typedef enum {
  TAS2505_OUTPUT_SPEAKER = 0,
  TAS2505_OUTPUT_HEADPHONE = 1,
  TAS2505_OUTPUT_BOTH = 2,
  TAS2505_OUTPUT_MAX = 3,
} tas2505_output_t;

typedef enum {
  TAS2505_INPUT_DAC = 0,
  TAS2505_INPUT_LINE = 1,
  TAS2505_INPUT_BOTH = 2,
  TAS2505_INPUT_MAX = 3,
} tas2505_input_t;

esp_err_t tas2505_init();
esp_err_t tas2505_deinit(void);
esp_err_t tas2505_enable_pa(bool enable);
esp_err_t tas2505_set_output(tas2505_output_t output);
esp_err_t tas2505_set_input(tas2505_input_t input);
esp_err_t tas2505_read_gpio(bool *val);

#ifdef __cplusplus
}
#endif
