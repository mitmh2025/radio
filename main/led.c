#include "led.h"
#include "main.h"
#include "board.h"
#include "things.h"

#include <string.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "led_strip.h"

static SemaphoreHandle_t led_mutex = NULL;
static led_strip_handle_t led = NULL;
static struct
{
  uint32_t red;
  uint32_t green;
  uint32_t blue;
} led_colors[LED_COUNT] = {};
static TaskHandle_t led_telemetry_task_handle = NULL;

void led_telemetry_task(void *arg)
{
  while (true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    char buf[sizeof("[#ffffff") * LED_COUNT + 2];
    char *ptr = buf;
    ptr += sprintf(ptr, "[");
    for (int i = 0; i < LED_COUNT; i++)
    {
      ptr += sprintf(ptr, "#%02x%02x%02x", (uint8_t)led_colors[i].red, (uint8_t)led_colors[i].green, (uint8_t)led_colors[i].blue);
      if (i < LED_COUNT - 1)
      {
        ptr += sprintf(ptr, ",");
      }
    }
    ptr += sprintf(ptr, "]");
    xSemaphoreGive(led_mutex);
    things_send_telemetry_string("led_strip", buf);
  }
}

void led_telemetry_generator()
{
  if (led_telemetry_task_handle != NULL)
  {
    xTaskNotifyGive(led_telemetry_task_handle);
  }
}

esp_err_t led_init()
{
  led_mutex = xSemaphoreCreateMutex();

  led_strip_config_t config = {
      .strip_gpio_num = LED_PIN,
      .max_leds = LED_COUNT,
      .led_pixel_format = LED_PIXEL_FORMAT_GRB,
      .led_model = LED_MODEL_WS2812,
      .flags.invert_out = false,
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .flags.with_dma = false,
  };
  ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&config, &rmt_config, &led), RADIO_TAG, "led_init failed");

  xTaskCreate(led_telemetry_task, "led_telemetry", 4096, NULL, 5, &led_telemetry_task_handle);
  things_register_telemetry_generator(led_telemetry_generator);

  return ESP_OK;
}

esp_err_t led_set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
  ESP_RETURN_ON_FALSE(led_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG, "led_init must be called before led_set_pixel");
  ESP_RETURN_ON_FALSE(index < LED_COUNT, ESP_ERR_INVALID_ARG, RADIO_TAG, "index must be less than LED_COUNT");
  xSemaphoreTake(led_mutex, portMAX_DELAY);

  esp_err_t ret = ESP_OK;

  // Our LEDs seem to be RGB, not GRB but the library expects GRB
  ESP_GOTO_ON_ERROR(led_strip_set_pixel(led, index, green, red, blue), cleanup, RADIO_TAG, "led_set_pixel failed");
  ESP_GOTO_ON_ERROR(led_strip_refresh(led), cleanup, RADIO_TAG, "led_set_pixel flush failed");
  led_colors[index].red = red;
  led_colors[index].green = green;
  led_colors[index].blue = blue;

  xTaskNotifyGive(led_telemetry_task_handle);

cleanup:
  xSemaphoreGive(led_mutex);
  return ret;
}
