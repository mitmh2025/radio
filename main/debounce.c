#include "debounce.h"
#include "main.h"

#include "esp_check.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE debounce_spinlock = portMUX_INITIALIZER_UNLOCKED;
struct debounce_config {
  uint64_t last_trigger;
  bool state;

  gpio_int_type_t intr_type;
  uint64_t timeout;
  gpio_isr_t isr;
  void *arg;
};
static struct debounce_config debounce_handlers[GPIO_NUM_MAX] = {};

static void IRAM_ATTR debounce_isr(void *ctx) {
  gpio_num_t gpio_num = (gpio_num_t)ctx;
  portENTER_CRITICAL_ISR(&debounce_spinlock);
  struct debounce_config *handler = &debounce_handlers[gpio_num];
  uint64_t now = esp_timer_get_time();
  if (now - handler->last_trigger < handler->timeout) {
    portEXIT_CRITICAL_ISR(&debounce_spinlock);
    return;
  }

  bool new_state = gpio_get_level(gpio_num);
  if (handler->state == new_state) {
    portEXIT_CRITICAL_ISR(&debounce_spinlock);
    return;
  }

  handler->last_trigger = now;
  handler->state = new_state;
  if (handler->intr_type == GPIO_INTR_POSEDGE && new_state) {
    handler->isr(handler->arg);
  } else if (handler->intr_type == GPIO_INTR_NEGEDGE && !new_state) {
    handler->isr(handler->arg);
  } else if (handler->intr_type == GPIO_INTR_ANYEDGE) {
    handler->isr(handler->arg);
  }
  portEXIT_CRITICAL_ISR(&debounce_spinlock);
}

esp_err_t debounce_init() {
  return gpio_install_isr_service(ESP_INTR_FLAG_SHARED);
}

esp_err_t debounce_handler_add(gpio_num_t gpio_num, gpio_int_type_t intr_type,
                               gpio_isr_t isr_handler, void *args,
                               uint64_t timeout_us) {
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio_num), ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "Invalid GPIO number: %d", gpio_num);
  ESP_RETURN_ON_FALSE(
      intr_type == GPIO_INTR_POSEDGE || intr_type == GPIO_INTR_NEGEDGE ||
          intr_type == GPIO_INTR_ANYEDGE,
      ESP_ERR_INVALID_ARG, RADIO_TAG, "Invalid interrupt type: %d", intr_type);
  portENTER_CRITICAL(&debounce_spinlock);
  debounce_handlers[gpio_num].last_trigger = 0;
  debounce_handlers[gpio_num].state = gpio_get_level(gpio_num);
  debounce_handlers[gpio_num].intr_type = intr_type;
  debounce_handlers[gpio_num].timeout = timeout_us;
  debounce_handlers[gpio_num].isr = isr_handler;
  debounce_handlers[gpio_num].arg = args;
  portEXIT_CRITICAL(&debounce_spinlock);
  // Get notified for all interrupts, we'll decide which ones to forward
  ESP_RETURN_ON_ERROR(gpio_set_intr_type(gpio_num, GPIO_INTR_ANYEDGE),
                      RADIO_TAG, "Failed to set interrupt type");
  ESP_RETURN_ON_ERROR(gpio_intr_enable(gpio_num), RADIO_TAG,
                      "Failed to enable interrupt");
  return gpio_isr_handler_add(gpio_num, debounce_isr, (void *)gpio_num);
}

esp_err_t debounce_handler_remove(gpio_num_t gpio_num) {
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio_num), ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "Invalid GPIO number: %d", gpio_num);
  ESP_RETURN_ON_ERROR(gpio_isr_handler_remove(gpio_num), RADIO_TAG,
                      "Failed to remove ISR handler");
  portENTER_CRITICAL(&debounce_spinlock);
  memset(&debounce_handlers[gpio_num], 0, sizeof(struct debounce_config));
  portEXIT_CRITICAL(&debounce_spinlock);
  return ESP_OK;
}
