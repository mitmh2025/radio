#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define TAS2505_ADDR 0b0011000

esp_err_t tas2505_init();
esp_err_t tas2505_deinit(void);
esp_err_t tas2505_enable_pa(bool enable);

#ifdef __cplusplus
}
#endif
