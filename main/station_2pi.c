#include "station_2pi.h"
#include "bluetooth.h"
#include "main.h"
#include "mixer.h"
#include "playback.h"
#include "things.h"
#include "tuner.h"
#include "webrtc_manager.h"

#include <math.h>
#include <sys/queue.h>

#include "esp_check.h"
#include "esp_log.h"

struct playback_entry {
  playback_handle_t handle;
  TAILQ_ENTRY(playback_entry) entries;
};

TAILQ_HEAD(playback_entry_list, playback_entry);

static bool tuned = false;
static bool need_wifi_announcement = false;
static bool made_wifi_announcement = false;

static SemaphoreHandle_t playback_mutex = NULL;
static struct playback_entry_list playbacks = TAILQ_HEAD_INITIALIZER(playbacks);

static void playback_task(void *ctx) {
  struct playback_entry *ent = ctx;

  ESP_ERROR_CHECK_WITHOUT_ABORT(playback_wait_for_completion(ent->handle));

  xSemaphoreTake(playback_mutex, portMAX_DELAY);
  playback_free(ent->handle);
  TAILQ_REMOVE(&playbacks, ent, entries);
  xSemaphoreGive(playback_mutex);

  vTaskDelete(NULL);
}

static esp_err_t play_announcement(const char *filename, bool duck) {
  ESP_LOGI(RADIO_TAG, "Playing audio (%s ducking): %s",
           duck ? "with" : "without", filename);

  bool locked = false;
  playback_cfg_t cfg = {
      .path = filename,
      .duck_others = duck,
      .tuned = true,
  };
  playback_handle_t handle;
  ESP_RETURN_ON_ERROR(playback_file(&cfg, &handle), RADIO_TAG,
                      "Failed to play audio file %s: %d", cfg.path, err_rc_);

  esp_err_t ret = ESP_OK;
  struct playback_entry *ent = calloc(1, sizeof(struct playback_entry));
  ESP_GOTO_ON_FALSE(ent, ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "Unable to allocate playback list entry");
  ent->handle = handle;

  xSemaphoreTake(playback_mutex, portMAX_DELAY);
  locked = true;
  // Spawn the task before adding to the list so that we can clean up if it
  // fails
  BaseType_t result =
      xTaskCreate(playback_task, "playback", 4096, (void *)ent, 5, NULL);
  ESP_GOTO_ON_FALSE(result == pdPASS, ESP_FAIL, cleanup, RADIO_TAG,
                    "Unable to create playback task");

  // Now that we've created the task, we are no longer responsible for cleanup
  handle = NULL;
  TAILQ_INSERT_HEAD(&playbacks, ent, entries);

cleanup:
  if (locked) {
    xSemaphoreGive(playback_mutex);
  }

  if (handle != NULL) {
    playback_free(handle);
  }

  return ret;
}

static void entune_two_pi(void *ctx) {
  if (need_wifi_announcement && !made_wifi_announcement) {
    made_wifi_announcement = true;
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        play_announcement("wifi-not-connected.opus", true));
  }

  tuned = true;
  bluetooth_set_mode(BLUETOOTH_MODE_DISABLED);
  webrtc_manager_entune();
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_COMFORT));
  ESP_ERROR_CHECK_WITHOUT_ABORT(things_set_updates_blocked(false));
}

static void detune_two_pi(void *ctx) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(things_set_updates_blocked(true));
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_static(MIXER_STATIC_MODE_DEFAULT));
  webrtc_manager_detune();
  bluetooth_set_mode(BLUETOOTH_MODE_DEFAULT);
  tuned = false;

  xSemaphoreTake(playback_mutex, portMAX_DELAY);
  struct playback_entry *ent = NULL;
  TAILQ_FOREACH(ent, &playbacks, entries) { playback_stop(ent->handle); }
  xSemaphoreGive(playback_mutex);
}

static esp_err_t play_audio_internal(const things_attribute_t *param,
                                     bool duck) {
  if (!tuned) {
    return ESP_OK;
  }

  if (param->type != THINGS_ATTRIBUTE_TYPE_STRING) {
    return ESP_ERR_INVALID_ARG;
  }

  return play_announcement(param->value.string, duck);
}

static esp_err_t play_unducked_audio(const things_attribute_t *param) {
  return play_audio_internal(param, false);
}

static esp_err_t play_ducked_audio(const things_attribute_t *param) {
  return play_audio_internal(param, true);
}

esp_err_t station_2pi_need_wifi_announcement(bool need_announcement) {
  if (!need_announcement) {
    made_wifi_announcement = false;
    need_wifi_announcement = false;
    return ESP_OK;
  }

  need_wifi_announcement = true;
  if (!made_wifi_announcement && tuned) {
    made_wifi_announcement = true;
    return play_announcement("wifi-not-connected.opus", true);
  }

  return ESP_OK;
}

esp_err_t station_2pi_init() {
  playback_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(playback_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Unable to create playback mutex");

  frequency_config_t config = {
      .frequency = 2.0f * M_PI,
      .enabled = true,
      .entune = entune_two_pi,
      .detune = detune_two_pi,
      .ctx = NULL,
  };

  // We don't need to make config changes so we don't need to save the handle
  frequency_handle_t handle;
  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &handle), RADIO_TAG,
                      "Failed to register 2pi frequency");

  ESP_RETURN_ON_ERROR(
      things_register_rpc("play_unducked_audio", play_unducked_audio),
      RADIO_TAG, "Unable to register play_unducked_audio RPC: %d", err_rc_);
  ESP_RETURN_ON_ERROR(
      things_register_rpc("play_ducked_audio", play_ducked_audio), RADIO_TAG,
      "Unable to register play_ducked_audio RPC: %d", err_rc_);

  return ESP_OK;
}