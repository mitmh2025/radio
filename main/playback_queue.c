#include "playback_queue.h"
#include "main.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static QueueHandle_t playback_queue = NULL;
static SemaphoreHandle_t current_playback_mutex = NULL;
static playback_handle_t current_playback = NULL;

static SemaphoreHandle_t empty_cb_mutex = NULL;
static playback_queue_empty_cb_t empty_cb = NULL;

static void playback_task(void *ctx) {
  while (true) {
    playback_queue_entry_t cfg;
    xQueueReceive(playback_queue, &cfg, portMAX_DELAY);

    xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
    esp_err_t err = playback_file(
        &(playback_cfg_t){
            .path = cfg.path,
            .duck_others = cfg.duck_others,
            .tuned = cfg.tuned,
            .skip_samples = cfg.skip_samples,
        },
        &current_playback);
    xSemaphoreGive(current_playback_mutex);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to play file: %d", err);
      goto next;
    }

    err = playback_wait_for_completion(current_playback);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to wait for completion: %d", err);
    }

    xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
    playback_free(current_playback);
    current_playback = NULL;
    xSemaphoreGive(current_playback_mutex);

next:
    if (uxQueueMessagesWaiting(playback_queue) == 0) {
      xSemaphoreTake(empty_cb_mutex, portMAX_DELAY);
      if (empty_cb) {
        empty_cb();
      }
      xSemaphoreGive(empty_cb_mutex);
    }
  }
}

esp_err_t playback_queue_add(playback_queue_entry_t *cfg) {
  ESP_RETURN_ON_FALSE(playback_queue, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Playback queue not initialized");

  ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Playback configuration must not be NULL");

  ESP_RETURN_ON_FALSE(pdTRUE == xQueueSendToBack(playback_queue, cfg, 0),
                      ESP_FAIL, RADIO_TAG, "Failed to add playback to queue");

  return ESP_OK;
}

esp_err_t playback_queue_subscribe_empty(playback_queue_empty_cb_t cb) {
  xSemaphoreTake(empty_cb_mutex, portMAX_DELAY);
  empty_cb = cb;
  xSemaphoreGive(empty_cb_mutex);

  return ESP_OK;
}

esp_err_t playback_queue_unsubscribe_empty(playback_queue_empty_cb_t cb) {
  xSemaphoreTake(empty_cb_mutex, portMAX_DELAY);
  if (empty_cb == cb) {
    empty_cb = NULL;
  }
  xSemaphoreGive(empty_cb_mutex);

  return ESP_OK;
}

esp_err_t playback_queue_skip() {
  xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
  if (current_playback) {
    playback_stop(current_playback);
  }
  xSemaphoreGive(current_playback_mutex);

  return ESP_OK;
}

esp_err_t playback_queue_drain() {
  ESP_RETURN_ON_FALSE(playback_queue, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Playback queue not initialized");

  playback_queue_entry_t cfg;
  while (xQueueReceive(playback_queue, &cfg, 0) == pdTRUE) {
    // Drain the queue
  }

  playback_queue_skip();

  return ESP_OK;
}

esp_err_t playback_queue_pause() {
  xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
  if (current_playback) {
    playback_pause(current_playback);
  }
  xSemaphoreGive(current_playback_mutex);

  return ESP_OK;
}

esp_err_t playback_queue_resume() {
  xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
  if (current_playback) {
    playback_resume(current_playback);
  }
  xSemaphoreGive(current_playback_mutex);

  return ESP_OK;
}

esp_err_t playback_queue_pause_toggle() {
  xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
  if (current_playback) {
    playback_pause_toggle(current_playback);
  }
  xSemaphoreGive(current_playback_mutex);

  return ESP_OK;
}

esp_err_t playback_queue_init() {
  playback_queue = xQueueCreate(8, sizeof(playback_queue_entry_t));
  ESP_RETURN_ON_FALSE(playback_queue, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create playback queue");

  current_playback_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(current_playback_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create current playback mutex");

  empty_cb_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(empty_cb_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create empty callback mutex");

  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreatePinnedToCore(playback_task,
                                                        "playback", 4096, NULL,
                                                        11, NULL, 1),
                      ESP_FAIL, RADIO_TAG, "Failed to create playback task");

  return ESP_OK;
}

bool playback_queue_active() {
  xSemaphoreTake(current_playback_mutex, portMAX_DELAY);
  bool active = current_playback != NULL;
  xSemaphoreGive(current_playback_mutex);
  return active;
}