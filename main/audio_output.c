#include "audio_output.h"
#include "main.h"
#include "tas2505.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NOTIFY_SUSPENDED BIT(0)

static TaskHandle_t output_task_handle = NULL;

static void output_task(void *arg) {
  bool suspended = false;
  while (1) {
    TickType_t wait =
        suspended ? portMAX_DELAY : pdMS_TO_TICKS(100 + esp_random() % 100);
    uint32_t notification = 0;
    xTaskNotifyWait(0, 0, &notification, wait);
    if (notification & NOTIFY_SUSPENDED) {
      suspended = true;
      continue;
    }
    if (!(notification & NOTIFY_SUSPENDED)) {
      suspended = false;
    }

    bool gpio;
    esp_err_t err = tas2505_read_gpio(&gpio);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to read GPIO: %d (%s)", err,
               esp_err_to_name(err));
      continue;
    }

    if (gpio) {
      tas2505_set_output(TAS2505_OUTPUT_SPEAKER);
    } else {
      tas2505_set_output(TAS2505_OUTPUT_HEADPHONE);
    }
  }
}

esp_err_t audio_output_init(void) {
  xTaskCreatePinnedToCore(output_task, "audio_output", 4096, NULL, 17,
                          &output_task_handle, 0);

  return ESP_OK;
}

esp_err_t audio_output_suspend() {
  ESP_RETURN_ON_FALSE(output_task_handle, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Audio output task not initialized");
  xTaskNotify(output_task_handle, NOTIFY_SUSPENDED, eSetBits);
  return ESP_OK;
}

esp_err_t audio_output_resume() {
  ESP_RETURN_ON_FALSE(output_task_handle, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Audio output task not initialized");
  xTaskNotify(output_task_handle, ~NOTIFY_SUSPENDED, eSetBits);
  return ESP_OK;
}
