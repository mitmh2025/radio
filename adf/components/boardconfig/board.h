#pragma once

#include "driver/i2c_master.h"
#include "board_pins_config.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ESP-ADF uses the same #defines for both SDIO and SPI, but we didn't wire all
// of the pins for SDIO
#define SDCARD_OPEN_FILE_NUM_MAX (5)
#define ESP_SD_PIN_CLK GPIO_NUM_12 // CLK
#define ESP_SD_PIN_CMD GPIO_NUM_11 // DI
#define ESP_SD_PIN_CD GPIO_NUM_14  // CD
#define ESP_SD_PIN_D0 GPIO_NUM_13  // DO
#define ESP_SD_PIN_D3 GPIO_NUM_10  // CS

#define ESP_SD_PIN_D1 (-1)
#define ESP_SD_PIN_D2 (-1)
#define ESP_SD_PIN_D4 (-1)
#define ESP_SD_PIN_D5 (-1)
#define ESP_SD_PIN_D6 (-1)
#define ESP_SD_PIN_D7 (-1)
#define ESP_SD_PIN_WP (-1)

#define BOARD_PA_GAIN (6) /* Power amplifier gain defined by board (dB) */
#define PA_ENABLE_GPIO GPIO_NUM_15

extern SemaphoreHandle_t i2c_mutex;
esp_err_t board_i2c_init(void);
// Note: ESP-IDF I2C APIs are not threadsafe, so any usage of this I2C bus
// (including devices) must be protected by i2c_mutex
i2c_master_bus_handle_t board_i2c_get_handle(void);

#define BOARD_I2C_MUTEX_LOCK() xSemaphoreTake(i2c_mutex, portMAX_DELAY)
#define BOARD_I2C_MUTEX_UNLOCK() xSemaphoreGive(i2c_mutex)

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_8
#define BATTERY_SCALE_FACTOR (3.0f / (2.0f * 1000.0f))

#ifdef __cplusplus
}
#endif
