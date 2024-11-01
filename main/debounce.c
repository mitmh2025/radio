#include "debounce.h"
#include "main.h"

#include "esp_check.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE debounce_spinlock = portMUX_INITIALIZER_UNLOCKED;
struct debounce_config {
  TimerHandle_t timer;
  bool state;

  gpio_int_type_t intr_type;
  TickType_t timeout;
  debounce_isr_t isr;
  void *arg;
};
static struct debounce_config debounce_handlers[GPIO_NUM_MAX] = {};

static void IRAM_ATTR debounce_isr(void *ctx) {
  gpio_num_t gpio_num = (gpio_num_t)ctx;
  portENTER_CRITICAL_ISR(&debounce_spinlock);
  struct debounce_config *handler = &debounce_handlers[gpio_num];
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  xTimerChangePeriodFromISR(handler->timer, handler->timeout,
                            &pxHigherPriorityTaskWoken);
  portEXIT_CRITICAL_ISR(&debounce_spinlock);

  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

static void debounce_timer_cb(TimerHandle_t xTimer) {
  gpio_num_t gpio_num = (gpio_num_t)pvTimerGetTimerID(xTimer);
  bool state = gpio_get_level(gpio_num);

  portENTER_CRITICAL(&debounce_spinlock);
  struct debounce_config *handler = &debounce_handlers[gpio_num];

  if (handler->state == state) {
    portEXIT_CRITICAL(&debounce_spinlock);
    return;
  }

  handler->state = state;
  if (handler->intr_type == GPIO_INTR_ANYEDGE ||
      (handler->intr_type == GPIO_INTR_POSEDGE && state) ||
      (handler->intr_type == GPIO_INTR_NEGEDGE && !state)) {
    handler->isr(handler->arg, state);
  }

  portEXIT_CRITICAL(&debounce_spinlock);
}

esp_err_t debounce_init() {
  for (int i = 0; i < GPIO_NUM_MAX; i++) {
    debounce_handlers[i].timer =
        xTimerCreate("debounce", 1, pdFALSE, (void *)i, debounce_timer_cb);
    if (!debounce_handlers[i].timer) {
      return ESP_ERR_NO_MEM;
    }
  }
  return gpio_install_isr_service(ESP_INTR_FLAG_SHARED);
}

esp_err_t debounce_handler_add(gpio_num_t gpio_num, gpio_int_type_t intr_type,
                               debounce_isr_t isr_handler, void *args,
                               TickType_t timeout) {
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio_num), ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "Invalid GPIO number: %d", gpio_num);
  ESP_RETURN_ON_FALSE(
      intr_type == GPIO_INTR_POSEDGE || intr_type == GPIO_INTR_NEGEDGE ||
          intr_type == GPIO_INTR_ANYEDGE,
      ESP_ERR_INVALID_ARG, RADIO_TAG, "Invalid interrupt type: %d", intr_type);
  portENTER_CRITICAL(&debounce_spinlock);
  debounce_handlers[gpio_num].state = gpio_get_level(gpio_num);
  debounce_handlers[gpio_num].intr_type = intr_type;
  debounce_handlers[gpio_num].timeout = timeout;
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
  xTimerStop(debounce_handlers[gpio_num].timer, portMAX_DELAY);
  memset(&debounce_handlers[gpio_num], 0, sizeof(struct debounce_config));
  portEXIT_CRITICAL(&debounce_spinlock);
  return ESP_OK;
}
