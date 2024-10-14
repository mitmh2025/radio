#pragma once

#include "board_pins_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// These #defines are required by ESP-ADF, even if you're not using its SD card
// integration (or don't have an SD card slot at all). We've populated them to
// match our board layout. It uses the same #defines for both SDIO and SPI, but
// we didn't wire all of the pins for SDIO
#define SDCARD_OPEN_FILE_NUM_MAX (5)
#define ESP_SD_PIN_CLK GPIO_NUM_12 // CLK
#define ESP_SD_PIN_CMD GPIO_NUM_11 // DI / MOSI
#define ESP_SD_PIN_CD GPIO_NUM_14  // CD
#define ESP_SD_PIN_D0 GPIO_NUM_13  // DO / MISO
#define ESP_SD_PIN_D3 GPIO_NUM_10  // CS

#define ESP_SD_PIN_D1 GPIO_NUM_NC
#define ESP_SD_PIN_D2 GPIO_NUM_NC
#define ESP_SD_PIN_D4 GPIO_NUM_NC
#define ESP_SD_PIN_D5 GPIO_NUM_NC
#define ESP_SD_PIN_D6 GPIO_NUM_NC
#define ESP_SD_PIN_D7 GPIO_NUM_NC
#define ESP_SD_PIN_WP GPIO_NUM_NC

// Note: The v0.2 board does not have D2 or D3 connected and pins 21 and 47 are
// on the GPIO breakout (i.e. normally not connected). The v0.3 board uses two
// W25Q128JV SPI flash chips, controlled by the same SPI bus with their CS
// connected to CS1 and CS2.
//
// We can probe for which board we're running on by trying to communicate with
// the flash chips.
#define RADIO_SPI_PIN_CLK GPIO_NUM_12
#define RADIO_SPI_PIN_MISO GPIO_NUM_13
#define RADIO_SPI_PIN_MOSI GPIO_NUM_11
#define RADIO_SPI_PIN_D2 GPIO_NUM_14
#define RADIO_SPI_PIN_D3 GPIO_NUM_10
#define RADIO_SPI_PIN_SD_CD GPIO_NUM_14
#define RADIO_SPI_PIN_CS1 GPIO_NUM_21
#define RADIO_SPI_PIN_CS2 GPIO_NUM_47
#define RADIO_SPI_PIN_SD_CS GPIO_NUM_10

#define BOARD_PA_GAIN (6) /* Power amplifier gain defined by board (dB) */
#define TAS2505_RST_GPIO GPIO_NUM_15
#define PA_ENABLE_GPIO TAS2505_RST_GPIO

#define SI4702_RST_GPIO GPIO_NUM_42

extern SemaphoreHandle_t i2c_mutex;
extern UBaseType_t i2c_mutex_holder_priority;
esp_err_t board_i2c_init(void);
// Note: ESP-IDF I2C APIs are not threadsafe, so any usage of this I2C bus
// (including devices) must be protected by i2c_mutex
i2c_master_bus_handle_t board_i2c_get_handle(void);

#define BOARD_I2C_MUTEX_LOCK()                                                 \
  do {                                                                         \
    int64_t start = esp_timer_get_time();                                      \
    TaskHandle_t holder = xSemaphoreGetMutexHolder(i2c_mutex);                 \
    UBaseType_t priority = uxTaskPriorityGet(NULL);                            \
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);                          \
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);                                  \
    i2c_mutex_holder_priority = priority;                                      \
    int64_t end = esp_timer_get_time();                                        \
    if (end - start > 10000) {                                                 \
      TaskStatus_t status;                                                     \
      vTaskGetInfo(holder, &status, pdTRUE, eInvalid);                         \
      ESP_LOGW("radio:board",                                                  \
               "%s(%d): Acquiring I2C bus took %lldus (held by %s)",           \
               __FUNCTION__, __LINE__, end - start, status.pcTaskName);        \
    }                                                                          \
  } while (0)
#define BOARD_I2C_MUTEX_UNLOCK()                                               \
  do {                                                                         \
    UBaseType_t orig_priority = i2c_mutex_holder_priority;                     \
    xSemaphoreGive(i2c_mutex);                                                 \
    vTaskPrioritySet(NULL, orig_priority);                                     \
  } while (0)

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_8
#define BATTERY_SCALE_FACTOR (3.0f / (2.0f * 1000.0f))

#define VOLUME_ADC_CHANNEL ADC_CHANNEL_0
#define FREQUENCY_ADC_CHANNEL ADC_CHANNEL_1

#define LED_COUNT (2)
#define LED_PIN GPIO_NUM_48

#define BUTTON_TRIANGLE_PIN GPIO_NUM_4
#define BUTTON_CIRCLE_PIN GPIO_NUM_33
#define TOGGLE_PIN GPIO_NUM_34

#ifdef __cplusplus
}
#endif
