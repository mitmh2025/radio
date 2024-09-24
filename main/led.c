#include "led.h"
#include "main.h"
#include "board.h"

#include "esp_check.h"

#include "led_strip.h"

static led_strip_handle_t led = NULL;

esp_err_t led_init()
{
  led_strip_config_t config = {
      .strip_gpio_num = LED_PIN,
      .max_leds = 2,
      .led_pixel_format = LED_PIXEL_FORMAT_GRB,
      .led_model = LED_MODEL_WS2812,
      .flags.invert_out = false,
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .flags.with_dma = false,
  };
  ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&config, &rmt_config, &led), RADIO_TAG, "led_init failed");

  return ESP_OK;
}

esp_err_t led_set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
  // Our LEDs seem to be RGB, not GRB but the library expects GRB
  ESP_RETURN_ON_ERROR(led_strip_set_pixel(led, index, green, red, blue), RADIO_TAG, "led_set_pixel failed");
  ESP_RETURN_ON_ERROR(led_strip_refresh(led), RADIO_TAG, "led_set_pixel flush failed");

  return ESP_OK;
}
