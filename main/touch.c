#include "touch.h"
#include "main.h"

#include "board.h"

#include "esp_check.h"

static uint32_t touch_threshold = 6000;

esp_err_t touch_init() {
  ESP_RETURN_ON_ERROR(touch_pad_init(), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_config(TOUCH_PAD_CHANNEL), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_filter_set_config(&(touch_filter_config_t){
                          .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
                          .mode = TOUCH_PAD_FILTER_IIR_64,
                      }),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_filter_enable(), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_denoise_set_config(&(touch_pad_denoise_t){
                          .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
                          .grade = TOUCH_PAD_DENOISE_BIT4,
                      }),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_denoise_enable(), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_set_thresh(TOUCH_PAD_CHANNEL, touch_threshold),
                      RADIO_TAG, "Failed to initialize touch");

  return ESP_OK;
}

esp_err_t touch_register_isr(intr_handler_t fn, void *arg) {
  ESP_RETURN_ON_ERROR(touch_pad_isr_register(fn, arg,
                                             TOUCH_PAD_INTR_MASK_ACTIVE |
                                                 TOUCH_PAD_INTR_MASK_INACTIVE),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE |
                                            TOUCH_PAD_INTR_MASK_INACTIVE),
                      RADIO_TAG, "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER), RADIO_TAG,
                      "Failed to initialize touch");
  ESP_RETURN_ON_ERROR(touch_pad_fsm_start(), RADIO_TAG,
                      "Failed to initialize touch");

  return ESP_OK;
}

esp_err_t touch_deregister_isr(intr_handler_t fn, void *arg) {
  ESP_RETURN_ON_ERROR(touch_pad_fsm_stop(), RADIO_TAG,
                      "Failed to stop touch FSM");
  ESP_RETURN_ON_ERROR(touch_pad_intr_disable(TOUCH_PAD_INTR_MASK_ACTIVE |
                                             TOUCH_PAD_INTR_MASK_INACTIVE),
                      RADIO_TAG, "Failed to disable touch interrupt");
  ESP_RETURN_ON_ERROR(touch_pad_isr_deregister(fn, arg), RADIO_TAG,
                      "Failed to deregister touch ISR");

  return ESP_OK;
}
