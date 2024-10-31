#include "adc.h"
#include "main.h"
#include "things.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t adc_mutex = NULL;
static EXT_RAM_BSS_ATTR struct adc_config {
  bool populated;
  adc_digi_pattern_config_t config;
  void (*callback)(void *user_data, adc_digi_output_data_t *result);
  void *user_data;
} adc_configs[SOC_ADC_MAX_CHANNEL_NUM] = {};
static adc_continuous_handle_t adc_handle = NULL;
static bool adc_running = false;

static atomic_int_fast64_t last_adc_conv_done = 0;
static atomic_int_fast64_t last_adc_read = 0;

static void telemetry_generator() {
  things_send_telemetry_int("adc_conv_done", atomic_load(&last_adc_conv_done));
  things_send_telemetry_int("adc_read", atomic_load(&last_adc_read));
  things_send_telemetry_bool("adc_running", adc_running);
}

static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata,
                                       void *user_data) {
  atomic_store(&last_adc_conv_done, esp_timer_get_time());

  TaskHandle_t adc_task_handle = (TaskHandle_t)user_data;
  BaseType_t must_yield = pdFALSE;
  vTaskNotifyGiveFromISR(adc_task_handle, &must_yield);

  return must_yield == pdTRUE;
}

static void adc_task(void *context) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // Continue reading as long as there is data to read
    while (true) {
      uint8_t result[SOC_ADC_DIGI_DATA_BYTES_PER_CONV *
                     SOC_ADC_MAX_CHANNEL_NUM] = {};
      uint32_t out_length;
      esp_err_t ret = adc_continuous_read(adc_handle, result, sizeof(result),
                                          &out_length, 100);
      if (ret == ESP_ERR_TIMEOUT          // No pending data
          || ret == ESP_ERR_INVALID_STATE // The driver is already stopped
      ) {
        // Wait until the interrupt wakes us up
        break;
      } else if (ret != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "adc_continuous_read failed: %s",
                 esp_err_to_name(ret));
        break;
      }

      atomic_store(&last_adc_read, esp_timer_get_time());

      xSemaphoreTake(adc_mutex, portMAX_DELAY);
      for (int i = 0; i < out_length; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *data = (adc_digi_output_data_t *)&result[i];
        struct adc_config *config = &adc_configs[data->type2.channel];
        if (config->callback) {
          config->callback(config->user_data, data);
        } else {
          ESP_LOGW(RADIO_TAG, "No callback for channel %d",
                   data->type2.channel);
        }
      }
      xSemaphoreGive(adc_mutex);
    }
  }
}

esp_err_t adc_init() {
  adc_mutex = xSemaphoreCreateMutex();
  if (adc_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  adc_continuous_handle_cfg_t cfg = {
      .conv_frame_size =
          SOC_ADC_DIGI_DATA_BYTES_PER_CONV * SOC_ADC_MAX_CHANNEL_NUM,
      .max_store_buf_size =
          SOC_ADC_DIGI_DATA_BYTES_PER_CONV * SOC_ADC_MAX_CHANNEL_NUM * 8,
      .flags = {.flush_pool = true},
  };
  ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&cfg, &adc_handle), RADIO_TAG,
                      "adc_init failed");

  TaskHandle_t adc_task_handle = NULL;
  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreatePinnedToCore(adc_task, "adc", 4096,
                                                        NULL, 18,
                                                        &adc_task_handle, 0),
                      ESP_FAIL, RADIO_TAG, "Failed to create adc task");

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = adc_conv_done_cb,
  };
  ESP_RETURN_ON_ERROR(adc_continuous_register_event_callbacks(adc_handle, &cbs,
                                                              adc_task_handle),
                      RADIO_TAG, "adc_init failed");

  things_register_telemetry_generator(telemetry_generator, "adc", NULL);

  return ESP_OK;
}

static esp_err_t adc_config_and_start() {
  int pattern_count = 0;
  for (int i = 0; i < SOC_ADC_MAX_CHANNEL_NUM; i++) {
    if (adc_configs[i].populated) {
      pattern_count++;
    }
  }

  if (pattern_count == 0) {
    return ESP_OK;
  }

  adc_digi_pattern_config_t patterns[pattern_count];
  memset(patterns, 0, sizeof(patterns));
  for (int i = 0, j = 0; i < SOC_ADC_MAX_CHANNEL_NUM; i++) {
    if (adc_configs[i].populated) {
      memcpy(&patterns[j++], &adc_configs[i].config,
             sizeof(adc_digi_pattern_config_t));
    }
  }

  adc_continuous_config_t config = {
      .pattern_num = pattern_count,
      .adc_pattern = patterns,
      .sample_freq_hz = 1000,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  ESP_RETURN_ON_ERROR(adc_continuous_config(adc_handle, &config), RADIO_TAG,
                      "Failed to set ADC continuous config");

  ESP_RETURN_ON_ERROR(adc_continuous_start(adc_handle), RADIO_TAG,
                      "Failed to start continuous ADC");

  adc_running = true;
  return ESP_OK;
}

esp_err_t adc_subscribe(adc_digi_pattern_config_t *config,
                        void (*callback)(void *user_data,
                                         adc_digi_output_data_t *result),
                        void *user_data) {
  ESP_RETURN_ON_FALSE(adc_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "adc_subscribe must be called after adc_init");
  ESP_RETURN_ON_FALSE(config && callback, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "adc_subscribe config and callback must not be NULL");
  ESP_RETURN_ON_FALSE(config->unit == ADC_UNIT_1, ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "adc_subscribe only supports ADC_UNIT_1");
  ESP_RETURN_ON_FALSE(
      config->channel < SOC_ADC_MAX_CHANNEL_NUM, ESP_ERR_INVALID_ARG, RADIO_TAG,
      "adc_subscribe channel must be less than %d", SOC_ADC_MAX_CHANNEL_NUM);

  xSemaphoreTake(adc_mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(!adc_configs[config->channel].populated,
                    ESP_ERR_INVALID_STATE, cleanup, RADIO_TAG,
                    "adc_subscribe channel %d already subscribed",
                    config->channel);

  if (adc_running) {
    adc_continuous_stop(adc_handle);
    adc_running = false;
  }

  adc_configs[config->channel].populated = true;
  adc_configs[config->channel].config = *config;
  adc_configs[config->channel].callback = callback;
  adc_configs[config->channel].user_data = user_data;

  ret = adc_config_and_start();

cleanup:
  xSemaphoreGive(adc_mutex);
  return ret;
}

esp_err_t adc_unsubscribe(uint8_t channel) {
  ESP_RETURN_ON_FALSE(adc_mutex, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "adc_unsubscribe must be called after adc_init");
  ESP_RETURN_ON_FALSE(channel < SOC_ADC_MAX_CHANNEL_NUM, ESP_ERR_INVALID_ARG,
                      RADIO_TAG, "adc_unsubscribe channel must be less than %d",
                      SOC_ADC_MAX_CHANNEL_NUM);

  xSemaphoreTake(adc_mutex, portMAX_DELAY);
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(adc_configs[channel].populated, ESP_ERR_INVALID_STATE,
                    cleanup, RADIO_TAG,
                    "adc_unsubscribe channel %d not subscribed", channel);

  if (adc_running) {
    adc_continuous_stop(adc_handle);
    adc_running = false;
  }

  adc_configs[channel].populated = false;
  adc_configs[channel].callback = NULL;

  ret = adc_config_and_start();

cleanup:
  xSemaphoreGive(adc_mutex);
  return ret;
}