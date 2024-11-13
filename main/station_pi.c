#include "nvs.h"

#include "accelerometer.h"
#include "adc.h"
#include "audio_output.h"
#include "bounds.h"
#include "debounce.h"
#include "led.h"
#include "magnet.h"
#include "main.h"
#include "mixer.h"
#include "playback.h"
#include "station_pi.h"
#include "station_pi_activation.h"
#include "tas2505.h"
#include "things.h"
#include "tone_generator.h"
#include "tuner.h"

#include "board.h"

#include <math.h>
#include <string.h>

#include "driver/touch_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#define STATION_PI_NVS_NAMESPACE "radio:pi"
static nvs_handle_t pi_nvs_handle;
static TaskHandle_t pi_task = NULL;
static size_t telemetry_index = 0;

static const int64_t play_time_idle_timeout = 30000000;
static SemaphoreHandle_t play_time_mutex = NULL;
static bool currently_playing = false;
static int64_t last_flushed_total_play_time = 0;
static int64_t total_play_time = 0;
static int64_t last_play_time_check = 0;
static esp_timer_handle_t idle_timer = NULL;
static esp_timer_handle_t play_flush_timer = NULL;

static frequency_handle_t freq_handle;
static bool entuned = false;
static bool instrument_enabled = true;

static QueueHandle_t playback_queue = NULL;
static SemaphoreHandle_t playback_mutex = NULL;
static playback_handle_t playback = NULL;

#define SHIFT_MAGNET BIT(0)
#define SHIFT_HEADPHONE BIT(1)

uint8_t previous_shift_state = 0;
uint8_t shift_state = 0;
int64_t last_headphone_change = 0;
float pitch_bend = 1.0f;

static uint64_t magnet_threshold =
    2400 * 2400; // roughly 3mT, but we test with magnitude^2
static uint64_t magnet_hysteresis = 100 * 100;

// Requires harder knocks than we require for activation
static const accelerometer_pulse_cfg_t knock_cfg = {
    .threshold_x = 34,
    .threshold_y = 34,
    .threshold_z = 34,
    .timelimit = 6,
    .latency = 1,
};
#define PULSE_DEBOUNCE_US (50 * 1000)
static int64_t knock_last_time = 0;
static esp_timer_handle_t knock_start_timer = NULL;
static esp_timer_handle_t knock_stop_timer = NULL;
static const float knock_frequencies[] = {
    [0] = FREQUENCY_G_4,
    [SHIFT_MAGNET] = FREQUENCY_D_4,
    [SHIFT_HEADPHONE] = FREQUENCY_D_5,
    [SHIFT_MAGNET + SHIFT_HEADPHONE] = FREQUENCY_G_5,
};
static tone_generator_t knock_tone = NULL;

static bounds_handle_t light_bounds;
static const uint16_t light_threshold = 1500;
static const uint16_t light_hysteresis = 25;
static uint16_t light_smooth = 0xffff;
static const float light_frequencies[] = {
    [0] = FREQUENCY_C_5,
    [SHIFT_MAGNET] = FREQUENCY_G_4,
    [SHIFT_HEADPHONE] = FREQUENCY_G_5,
    [SHIFT_MAGNET + SHIFT_HEADPHONE] = FREQUENCY_C_6,
};
static bool light_triggered = false;
static tone_generator_t light_tone = NULL;

static uint32_t touch_threshold = 6000;
static const float touch_frequencies[] = {
    [0] = FREQUENCY_A_4,
    [SHIFT_MAGNET] = FREQUENCY_E_4,
    [SHIFT_HEADPHONE] = FREQUENCY_E_5,
    [SHIFT_MAGNET + SHIFT_HEADPHONE] = FREQUENCY_A_5,
};
static bool touch_triggered = false;
static tone_generator_t touch_tone = NULL;

static const float button_frequencies[] = {
    [0] = FREQUENCY_B_4,
    [SHIFT_MAGNET] = FREQUENCY_F_SHARP_4,
    [SHIFT_HEADPHONE] = FREQUENCY_F_SHARP_5,
    [SHIFT_MAGNET + SHIFT_HEADPHONE] = FREQUENCY_B_5,
};
bool button_triggered = false;
static tone_generator_t button_tone = NULL;

#define NOTES_TRACK_SIZE 32
struct note_t {
  float frequency;
  int64_t start;
};
static EXT_RAM_BSS_ATTR struct note_t notes_played[NOTES_TRACK_SIZE] = {};
static size_t notes_played_index = 0;
static int64_t sequence_check_timeout = 500000;
static esp_timer_handle_t sequence_check_timer = NULL;

static uint8_t current_stage = 0;
#define STAGE_COUNT 5

static const char *intros[STAGE_COUNT] = {
    [0] = "practical-fighter/stage-0-intro.opus",
    [1] = "practical-fighter/stage-1-intro.opus",
    [2] = "practical-fighter/stage-2-intro.opus",
    [3] = "practical-fighter/stage-3-intro.opus",
    [4] = "practical-fighter/stage-4-intro.opus",
};

static const char *examples[STAGE_COUNT] = {
    [0] = "practical-fighter/stage-0-example.opus",
    [1] = "practical-fighter/stage-1-example.opus",
    [2] = "practical-fighter/stage-2-example.opus",
    [3] = "practical-fighter/stage-3-example.opus",
    [4] = "practical-fighter/stage-4-example.opus",
};

static const char *completions[STAGE_COUNT] = {
    [0] = "practical-fighter/stage-0-completion.opus",
    [1] = "practical-fighter/stage-1-completion.opus",
    [2] = "practical-fighter/stage-2-completion.opus",
    [3] = "practical-fighter/stage-3-completion.opus",
    [4] = "practical-fighter/stage-4-completion.opus",
};

static const char *final_completion = "practical-fighter/completion.opus";

static void play_on_success(uint8_t stage) {
  if (stage < STAGE_COUNT) {
    const char *completion = completions[stage];
    xQueueSend(playback_queue, &completion, portMAX_DELAY);
  }

  // intro and example are from the next stage
  stage++;

  if (stage < STAGE_COUNT) {
    const char *intro = intros[stage];
    xQueueSend(playback_queue, &intro, portMAX_DELAY);
    const char *example = examples[stage];
    xQueueSend(playback_queue, &example, portMAX_DELAY);
  } else {
    xQueueSend(playback_queue, &final_completion, portMAX_DELAY);
  }
}

static void play_on_entune(uint8_t stage) {
  if (stage < STAGE_COUNT) {
    const char *intro = intros[stage];
    xQueueSend(playback_queue, &intro, portMAX_DELAY);
    const char *example = examples[stage];
    xQueueSend(playback_queue, &example, portMAX_DELAY);
  }
}

static void play_on_button(uint8_t stage) {
  if (stage < STAGE_COUNT) {
    const char *intro = intros[stage];
    xQueueSend(playback_queue, &intro, portMAX_DELAY);
    const char *example = examples[stage];
    xQueueSend(playback_queue, &example, portMAX_DELAY);
  } else {
    xQueueSend(playback_queue, &final_completion, portMAX_DELAY);
  }
}

// Mary Had a Little Lamb - 7 notes
static const float note_sequence_0[] = {
    FREQUENCY_B_4, FREQUENCY_A_4, FREQUENCY_G_4, FREQUENCY_A_4,
    FREQUENCY_B_4, FREQUENCY_B_4, FREQUENCY_B_4,
};
// Never Gonna Give You Up - 7 notes
static const float note_sequence_1[] = {
    FREQUENCY_D_4, FREQUENCY_E_4, FREQUENCY_G_4, FREQUENCY_E_4,
    FREQUENCY_B_4, FREQUENCY_B_4, FREQUENCY_A_4,
};
// Somewhere Over the Rainbow - 10 notes
static const float note_sequence_2[] = {
    FREQUENCY_G_4, FREQUENCY_G_5,       FREQUENCY_F_SHARP_5, FREQUENCY_D_5,
    FREQUENCY_E_5, FREQUENCY_F_SHARP_5, FREQUENCY_G_5,       FREQUENCY_G_4,
    FREQUENCY_E_5, FREQUENCY_D_5,
};
// Hot To Go - 16 notes
static const float note_sequence_3[] = {
    FREQUENCY_B_4, FREQUENCY_D_5, FREQUENCY_D_5, FREQUENCY_D_5,
    FREQUENCY_E_5, FREQUENCY_D_5, FREQUENCY_E_5, FREQUENCY_D_5,
    FREQUENCY_B_4, FREQUENCY_D_5, FREQUENCY_G_5, FREQUENCY_A_5,
    FREQUENCY_A_5, FREQUENCY_B_5, FREQUENCY_A_5, FREQUENCY_G_5,
};
// Final Countdown - 20 notes
static const float note_sequence_4[] = {
    FREQUENCY_B_4, FREQUENCY_A_4,       FREQUENCY_B_4, FREQUENCY_E_4,
    FREQUENCY_C_5, FREQUENCY_B_4,       FREQUENCY_C_5, FREQUENCY_B_4,
    FREQUENCY_A_4, FREQUENCY_C_5,       FREQUENCY_B_4, FREQUENCY_C_5,
    FREQUENCY_E_4, FREQUENCY_A_4,       FREQUENCY_G_4, FREQUENCY_A_4,
    FREQUENCY_G_4, FREQUENCY_F_SHARP_4, FREQUENCY_A_4, FREQUENCY_G_4,
};

static void telemetry_generator() {
  things_send_telemetry_int("pi_stage", current_stage);
  things_send_telemetry_int("pi_total_play_time",
                            total_play_time / (1000 * 1000));
  things_send_telemetry_bool("pi_note_knock", knock_tone != NULL && entuned &&
                                                  instrument_enabled);
  things_send_telemetry_bool("pi_note_light", light_tone != NULL && entuned &&
                                                  instrument_enabled);
  things_send_telemetry_bool("pi_note_touch", touch_tone != NULL && entuned &&
                                                  instrument_enabled);
  things_send_telemetry_bool("pi_note_button", button_tone != NULL && entuned &&
                                                   instrument_enabled);
  things_send_telemetry_bool("pi_shift_magnet", shift_state & SHIFT_MAGNET &&
                                                    entuned &&
                                                    instrument_enabled);
  things_send_telemetry_bool("pi_shift_headphone",
                             shift_state & SHIFT_HEADPHONE && entuned &&
                                 instrument_enabled);
}

static void force_telemetry() { things_force_telemetry(telemetry_index); }

static void record_note(float frequency) {
  notes_played[notes_played_index] = (struct note_t){
      .frequency = frequency,
      .start = esp_timer_get_time(),
  };
  notes_played_index = (notes_played_index + 1) % NOTES_TRACK_SIZE;
}

static void check_sequence(void *arg) {
  // Only check sequences when no notes are playing
  if (light_tone || touch_tone || button_tone || knock_tone) {
    return;
  }

  const float *sequence = NULL;
  size_t sequence_size = 0;

  switch (current_stage) {
  case 0:
    sequence = &note_sequence_0[0];
    sequence_size = sizeof(note_sequence_0) / sizeof(note_sequence_0[0]);
    break;
  case 1:
    sequence = &note_sequence_1[0];
    sequence_size = sizeof(note_sequence_1) / sizeof(note_sequence_1[0]);
    break;
  case 2:
    sequence = &note_sequence_2[0];
    sequence_size = sizeof(note_sequence_2) / sizeof(note_sequence_2[0]);
    break;
  case 3:
    sequence = &note_sequence_3[0];
    sequence_size = sizeof(note_sequence_3) / sizeof(note_sequence_3[0]);
    break;
  case 4:
    sequence = &note_sequence_4[0];
    sequence_size = sizeof(note_sequence_4) / sizeof(note_sequence_4[0]);
    break;
  default:
    return;
  }

  size_t played_sequence_start =
      (notes_played_index + NOTES_TRACK_SIZE - sequence_size) %
      NOTES_TRACK_SIZE;
  for (size_t i = 0; i < sequence_size; i++) {
    if (fabsf(notes_played[(played_sequence_start + i) % NOTES_TRACK_SIZE]
                  .frequency -
              sequence[i]) > 0.1) {
      return;
    }
  }

  ESP_LOGI(RADIO_TAG, "Sequence matched for stage %d", current_stage);
  play_on_success(current_stage);
  current_stage++;
  esp_err_t err = nvs_set_u8(pi_nvs_handle, "stage", current_stage);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to save stage: %d", err);
  }
}

static void schedule_sequence_check() {
  esp_timer_stop(sequence_check_timer);
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_start_once(sequence_check_timer, sequence_check_timeout));
}

static void flush_total_play_time() {
  if (last_flushed_total_play_time == total_play_time) {
    return;
  }

  ESP_ERROR_CHECK_WITHOUT_ABORT(
      nvs_set_i64(pi_nvs_handle, "total_play_time", total_play_time));
  ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(pi_nvs_handle));
  last_flushed_total_play_time = total_play_time;
}

static void idle_timer_cb(void *arg) {
  int64_t now = esp_timer_get_time();
  xSemaphoreTake(play_time_mutex, portMAX_DELAY);
  total_play_time += now - last_play_time_check;
  last_play_time_check = now;
  currently_playing = false;
  flush_total_play_time();
  xSemaphoreGive(play_time_mutex);
}

static void start_play_tracking() {
  xSemaphoreTake(play_time_mutex, portMAX_DELAY);
  if (!currently_playing) {
    esp_timer_stop(idle_timer);
    last_play_time_check = esp_timer_get_time();
    currently_playing = true;
  }
  xSemaphoreGive(play_time_mutex);
}

static void schedule_idle_timer() {
  esp_timer_stop(idle_timer);
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_start_once(idle_timer, play_time_idle_timeout));
}

static void play_flush_timer_cb(void *arg) {
  xSemaphoreTake(play_time_mutex, portMAX_DELAY);
  if (currently_playing) {
    int64_t now = esp_timer_get_time();
    total_play_time += now - last_play_time_check;
    last_play_time_check = now;
  }
  flush_total_play_time();
  xSemaphoreGive(play_time_mutex);
}

static void light_stop_tone() {
  tone_generator_release(light_tone);
  light_tone = NULL;

  schedule_sequence_check();
  schedule_idle_timer();
  force_telemetry();
}

static void light_start_tone() {
  start_play_tracking();
  esp_timer_stop(sequence_check_timer);
  float frequency = light_frequencies[shift_state];
  if (current_stage >= STAGE_COUNT) {
    frequency *= pitch_bend;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned && instrument_enabled,
          .volume = 0x8000,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = frequency,
      },
      &light_tone));
  record_note(frequency);
  force_telemetry();
}

static void touch_stop_tone() {
  tone_generator_release(touch_tone);
  touch_tone = NULL;

  schedule_sequence_check();
  schedule_idle_timer();
  force_telemetry();
}

static void touch_start_tone() {
  start_play_tracking();
  esp_timer_stop(sequence_check_timer);
  float frequency = touch_frequencies[shift_state];
  if (current_stage >= STAGE_COUNT) {
    frequency *= pitch_bend;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned && instrument_enabled,
          .volume = 0x8000,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = frequency,
      },
      &touch_tone));
  record_note(frequency);
  force_telemetry();
}

static void button_stop_tone() {
  tone_generator_release(button_tone);
  button_tone = NULL;

  schedule_sequence_check();
  schedule_idle_timer();
  force_telemetry();
}

static void button_start_tone() {
  start_play_tracking();
  esp_timer_stop(sequence_check_timer);
  float frequency = button_frequencies[shift_state];
  if (current_stage >= STAGE_COUNT) {
    frequency *= pitch_bend;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned && instrument_enabled,
          .volume = 0x8000,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = frequency,
      },
      &button_tone));
  record_note(frequency);
  force_telemetry();
}

static void knock_stop_tone(void *arg) {
  esp_timer_stop(knock_stop_timer);
  if (knock_tone) {
    tone_generator_release(knock_tone);
    knock_tone = NULL;

    schedule_sequence_check();
    schedule_idle_timer();
    force_telemetry();
  }
}

static void knock_start_tone(void *arg) {
  if (esp_timer_get_time() - last_headphone_change < 400000) {
    return;
  }

  if (esp_timer_is_active(knock_stop_timer) || knock_tone) {
    knock_stop_tone(NULL);
  }

  start_play_tracking();
  esp_timer_stop(sequence_check_timer);
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(knock_stop_timer, 400000));
  float frequency = knock_frequencies[shift_state];
  if (current_stage >= STAGE_COUNT) {
    frequency *= pitch_bend;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(tone_generator_init(
      &(tone_generator_config_t){
          .entuned = entuned && instrument_enabled,
          .volume = 0x8000,
          .attack_time = 20000,
          .decay_time = 20000,
          .sustain_level = 0x8000,
          .release_time = 100000,
          .frequency = frequency,
      },
      &knock_tone));
  record_note(frequency);
  force_telemetry();
}

static void update_led() {
  if (!entuned) {
    return;
  }

  if (!instrument_enabled) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
    return;
  }

  switch (shift_state) {
  case 0:
    ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
    break;
  case SHIFT_MAGNET:
    ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 255, 128, 0));
    break;
  case SHIFT_HEADPHONE:
    ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 255, 255));
    break;
  case SHIFT_MAGNET + SHIFT_HEADPHONE:
    ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 255, 128, 255));
    break;
  default:
    break;
  }
}

static void update_state() {
  if (shift_state != previous_shift_state) {
    if (light_tone) {
      light_stop_tone();
      light_start_tone();
    }
    if (touch_tone) {
      touch_stop_tone();
      touch_start_tone();
    }
    if (button_tone) {
      button_stop_tone();
      button_start_tone();
    }
    // Don't need to restart knock tone since it is fixed duration
    update_led();
    force_telemetry();
    previous_shift_state = shift_state;
  }

  if (pitch_bend != 1.0f) {
    if (light_tone) {
      tone_generator_adjust_frequency(
          light_tone, light_frequencies[shift_state] * pitch_bend);
    }
    if (touch_tone) {
      tone_generator_adjust_frequency(
          touch_tone, touch_frequencies[shift_state] * pitch_bend);
    }
    if (button_tone) {
      tone_generator_adjust_frequency(
          button_tone, button_frequencies[shift_state] * pitch_bend);
    }
  }

  if (light_triggered && !light_tone) {
    light_start_tone();
  } else if (!light_triggered && light_tone) {
    light_stop_tone();
  }

  if (touch_triggered && !touch_tone) {
    touch_start_tone();
  } else if (!touch_triggered && touch_tone) {
    touch_stop_tone();
  }

  if (button_triggered && !button_tone) {
    button_start_tone();
  } else if (!button_triggered && button_tone) {
    button_stop_tone();
  }

  if (light_tone) {
    tone_generator_set_entuned(light_tone, entuned && instrument_enabled);
  }
  if (knock_tone) {
    tone_generator_set_entuned(knock_tone, entuned && instrument_enabled);
  }
  if (touch_tone) {
    tone_generator_set_entuned(touch_tone, entuned && instrument_enabled);
  }
  if (button_tone) {
    tone_generator_set_entuned(button_tone, entuned && instrument_enabled);
  }
}

static void light_adc_cb(void *ctx, adc_digi_output_data_t *result) {
  uint16_t value = result->type2.data;

  if (light_smooth == 0xffff) {
    light_smooth = value;
    return;
  }
  light_smooth = light_smooth - (light_smooth >> 3) + (value >> 3);

  bounds_update(light_bounds, light_smooth);

  bool was_triggered = light_triggered;
  uint16_t threshold = bounds_get_max(light_bounds) - light_threshold;
  if (was_triggered) {
    if (threshold + light_hysteresis < light_smooth) {
      light_triggered = false;
    }
  } else {
    if (threshold - light_hysteresis > light_smooth) {
      light_triggered = true;
    }
  }

  if (light_triggered != was_triggered) {
    xTaskNotify(pi_task, 0, eNoAction);
  }
}

#define NOTIFY_TOUCH_ACTIVE BIT(0)
#define NOTIFY_TOUCH_INACTIVE BIT(1)
#define NOTIFY_TRIANGLE_BUTTON_TRIGGERED BIT(2)
#define NOTIFY_CIRCLE_BUTTON_TRIGGERED BIT(3)
#define NOTIFY_KNOCK_START BIT(4)

static int16_t accel_x_smoothed, accel_y_smoothed, accel_z_smoothed;

static void station_pi_task(void *ctx) {
  while (true) {
    uint32_t notification = 0;
    TickType_t wait_time = pdMS_TO_TICKS(entuned ? (50 + esp_random() % 5)
                                                 : 1000 + esp_random() % 100);
    xTaskNotifyWait(0, ULONG_MAX, &notification, wait_time);

    if (notification & NOTIFY_CIRCLE_BUTTON_TRIGGERED) {
      xSemaphoreTake(playback_mutex, portMAX_DELAY);
      if (playback) {
        playback_stop(playback);
      } else {
        play_on_button(current_stage);
      }
      xSemaphoreGive(playback_mutex);
    }

    if (notification & NOTIFY_TOUCH_ACTIVE && !touch_tone) {
      touch_triggered = true;
    }
    if (notification & NOTIFY_TOUCH_INACTIVE && touch_tone) {
      touch_triggered = false;
    }

    bool button_active = gpio_get_level(BUTTON_TRIANGLE_PIN) == 0;

    if (button_active && !button_tone) {
      button_triggered = true;
    }
    if (!button_active && button_tone) {
      button_triggered = false;
    }
    if (notification & NOTIFY_KNOCK_START) {
      if (esp_timer_restart(knock_start_timer, 100000) ==
          ESP_ERR_INVALID_STATE) {
        esp_timer_start_once(knock_start_timer, 100000);
      }
    }

    bool old_gpio = !(shift_state & SHIFT_HEADPHONE);
    bool gpio;
    esp_err_t err = tas2505_read_gpio(&gpio);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to read GPIO: %d", err);
      // default to not connected, i.e. high
      gpio = true;
    }

    if (old_gpio != gpio) {
      last_headphone_change = esp_timer_get_time();
    }
    if (gpio || current_stage < 2) {
      shift_state &= ~SHIFT_HEADPHONE;
    } else {
      shift_state |= SHIFT_HEADPHONE;
    }

    if (current_stage < 1) {
      shift_state &= ~SHIFT_MAGNET;
    } else {
      int16_t magnet_x, magnet_y, magnet_z;
      err = magnet_read(&magnet_x, &magnet_y, &magnet_z);
      if (err != ESP_OK) {
        ESP_LOGE(RADIO_TAG, "Failed to read magnetometer: %d", err);
        // default to not connected
        magnet_x = magnet_y = magnet_z = 0;
      }

      uint64_t magnet_magnitude = (uint64_t)magnet_x * magnet_x +
                                  (uint64_t)magnet_y * magnet_y +
                                  (uint64_t)magnet_z * magnet_z;
      if (magnet_magnitude > magnet_threshold + magnet_hysteresis) {
        shift_state |= SHIFT_MAGNET;
      } else if (magnet_magnitude < magnet_threshold - magnet_hysteresis) {
        shift_state &= ~SHIFT_MAGNET;
      }
    }

    if (current_stage >= STAGE_COUNT) {
      // Don't pitch bend if there's an active knock going on because that
      // throws off the accelerometer
      if (knock_last_time + PULSE_DEBOUNCE_US < esp_timer_get_time()) {
        int16_t accel_x, accel_y, accel_z;
        err = accelerometer_read_data(&accel_x, &accel_y, &accel_z);
        if (err != ESP_OK) {
          ESP_LOGE(RADIO_TAG, "Failed to read accelerometer: %d", err);
          accel_x = accel_y = accel_z = 0;
        }

        accel_x_smoothed =
            accel_x_smoothed - (accel_x_smoothed >> 3) + (accel_x >> 3);
        accel_y_smoothed =
            accel_y_smoothed - (accel_y_smoothed >> 3) + (accel_y >> 3);
        accel_z_smoothed =
            accel_z_smoothed - (accel_z_smoothed >> 3) + (accel_z >> 3);

        float roll = atan2f(
            accel_y, (accel_x < 0 ? 1 : -1) *
                         sqrtf(accel_x * accel_x + 0.001 * accel_z * accel_z));
        // To turn this into a pitch bend, take the absolute value, scale it
        // from [0, pi] to [-2, 2], and then compute 2^(x/12). (This means the
        // range goes from 2^(-2/12) to 2^(2/12) which is the ratio of a full
        // step in either direction)
        float new_pitch_bend =
            powf(2.0f, -((4.0f * fabsf(roll) / M_PI) - 2.0f) / 12.0f);
        pitch_bend = pitch_bend * 0.25f + new_pitch_bend * 0.75f;
      }
    } else {
      pitch_bend = 1.0f;
    }

    update_state();
  }
}

static void playback_task(void *ctx) {
  while (true) {
    const char *file;
    xQueueReceive(playback_queue, &file, portMAX_DELAY);
    if (!file || !entuned) {
      continue;
    }

    start_play_tracking();
    instrument_enabled = false;
    xTaskNotify(pi_task, 0, eNoAction);
    update_led();

    xSemaphoreTake(playback_mutex, portMAX_DELAY);
    esp_err_t err = playback_file(
        &(playback_cfg_t){
            .path = file,
            .tuned = true,
        },
        &playback);
    xSemaphoreGive(playback_mutex);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to play queued file: %d", err);
      continue;
    }

    err = playback_wait_for_completion(playback);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to wait for completion: %d", err);
    }

    // If we're about to play another file, don't re-enable the instrument
    if (uxQueueMessagesWaiting(playback_queue) == 0) {
      memset(notes_played, 0, sizeof(notes_played));
      instrument_enabled = true;
      xTaskNotify(pi_task, 0, eNoAction);
      update_led();
      schedule_idle_timer();
    }

    xSemaphoreTake(playback_mutex, portMAX_DELAY);
    playback_handle_t p = playback;
    playback = NULL;
    playback_free(p);
    xSemaphoreGive(playback_mutex);
  }
}

static void IRAM_ATTR touch_intr(void *ctx) {
  TaskHandle_t task_handle = (TaskHandle_t)ctx;
  uint32_t touch_status = touch_pad_read_intr_status_mask();
  uint32_t notification = 0;
  if (touch_status & TOUCH_PAD_INTR_MASK_ACTIVE) {
    notification |= NOTIFY_TOUCH_ACTIVE;
  }
  if (touch_status & TOUCH_PAD_INTR_MASK_INACTIVE) {
    notification |= NOTIFY_TOUCH_INACTIVE;
  }
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, notification, eSetBits,
                     &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void IRAM_ATTR triangle_button_intr(void *ctx, bool state) {
  TaskHandle_t task_handle = (TaskHandle_t)ctx;
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, NOTIFY_TRIANGLE_BUTTON_TRIGGERED, eSetBits,
                     &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void IRAM_ATTR circle_button_intr(void *ctx, bool state) {
  TaskHandle_t task_handle = (TaskHandle_t)ctx;
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, NOTIFY_CIRCLE_BUTTON_TRIGGERED, eSetBits,
                     &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void knock_cb(accelerometer_pulse_axis_t axis, void *arg) {
  int64_t now = esp_timer_get_time();

  // Debounce the pulse
  if (now - knock_last_time < PULSE_DEBOUNCE_US) {
    goto cleanup;
  }

  TaskHandle_t task_handle = (TaskHandle_t)arg;
  BaseType_t higher_priority_task_woken = pdFALSE;
  xTaskNotifyFromISR(task_handle, NOTIFY_KNOCK_START, eSetBits,
                     &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);

cleanup:
  knock_last_time = now;
}

static void entune(void *ctx) {
  station_pi_activation_enable(false);
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(false));
  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_output_suspend());
  ESP_ERROR_CHECK_WITHOUT_ABORT(tas2505_set_output(TAS2505_OUTPUT_SPEAKER));
  memset(notes_played, 0, sizeof(notes_played));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      accelerometer_subscribe_pulse(&knock_cfg, knock_cb, pi_task));
  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&(gpio_config_t){
      .pin_bit_mask = BIT64(BUTTON_TRIANGLE_PIN) | BIT64(BUTTON_CIRCLE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  }));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      debounce_handler_add(BUTTON_TRIANGLE_PIN, GPIO_INTR_ANYEDGE,
                           triangle_button_intr, pi_task, pdMS_TO_TICKS(10)));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      debounce_handler_add(BUTTON_CIRCLE_PIN, GPIO_INTR_POSEDGE,
                           circle_button_intr, pi_task, pdMS_TO_TICKS(10)));

  play_on_entune(current_stage);
  entuned = true;
  xTaskNotify(pi_task, 0, eNoAction);
  update_led();
}

static void detune(void *ctx) {
  entuned = false;
  xTaskNotify(pi_task, 0, eNoAction);
  xSemaphoreTake(playback_mutex, portMAX_DELAY);
  if (playback) {
    playback_stop(playback);
  }
  xSemaphoreGive(playback_mutex);
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_CIRCLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(debounce_handler_remove(BUTTON_TRIANGLE_PIN));
  ESP_ERROR_CHECK_WITHOUT_ABORT(accelerometer_unsubscribe_pulse());
  ESP_ERROR_CHECK_WITHOUT_ABORT(led_set_pixel(1, 0, 0, 0));
  ESP_ERROR_CHECK_WITHOUT_ABORT(audio_output_resume());
  ESP_ERROR_CHECK_WITHOUT_ABORT(mixer_set_default_static(true));
  station_pi_activation_enable(true);
}

esp_err_t station_pi_init() {
  ESP_RETURN_ON_ERROR(things_register_telemetry_generator(
                          telemetry_generator, "pi", &telemetry_index),
                      RADIO_TAG, "Failed to register pi telemetry generator");

  ESP_RETURN_ON_ERROR(
      nvs_open(STATION_PI_NVS_NAMESPACE, NVS_READWRITE, &pi_nvs_handle),
      RADIO_TAG, "Failed to open NVS handle for station pi");

  esp_err_t err =
      nvs_get_i64(pi_nvs_handle, "total_play_time", &total_play_time);
  if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                        "Failed to get total play time from NVS");
  }
  last_flushed_total_play_time = total_play_time;

  play_time_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(play_time_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create play time mutex");
  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "pi_play_flush",
                              .callback = play_flush_timer_cb,
                          },
                          &play_flush_timer),
                      RADIO_TAG, "Failed to initialize play time flush timer");
  ESP_RETURN_ON_ERROR(esp_timer_start_periodic(play_flush_timer, 60000000),
                      RADIO_TAG, "Failed to start play time flush timer");
  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "pi_idle",
                              .callback = idle_timer_cb,
                          },
                          &idle_timer),
                      RADIO_TAG, "Failed to initialize idle timer");

  playback_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(playback_mutex, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create playback mutex");
  playback_queue = xQueueCreate(4, sizeof(const char *));
  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreatePinnedToCore(playback_task,
                                                        "pi_playback", 4096,
                                                        NULL, 11, NULL, 1),
                      ESP_FAIL, RADIO_TAG, "Failed to create playback task");

  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreatePinnedToCore(station_pi_task, "pi",
                                                        4096, NULL, 11,
                                                        &pi_task, 0),
                      ESP_FAIL, RADIO_TAG, "Failed to create station pi task");

  ESP_RETURN_ON_ERROR(bounds_init(
                          &(bounds_config_t){
                              .buckets = 5,
                              .interval = 1000000,
                          },
                          &light_bounds),
                      RADIO_TAG, "Failed to initialize light bounds");
  ESP_RETURN_ON_ERROR(adc_subscribe(
                          &(adc_digi_pattern_config_t){
                              .atten = ADC_ATTEN_DB_12,
                              .channel = LIGHT_ADC_CHANNEL,
                              .unit = ADC_UNIT_1,
                              .bit_width = 12,
                          },
                          light_adc_cb, NULL),
                      RADIO_TAG, "Failed to subscribe to light ADC");

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "knock_tone",
                              .callback = knock_stop_tone,
                          },
                          &knock_stop_timer),
                      RADIO_TAG, "Failed to initialize knock timer");
  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "knock_tone",
                              .callback = knock_start_tone,
                          },
                          &knock_start_timer),
                      RADIO_TAG, "Failed to initialize knock timer");

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
  ESP_RETURN_ON_ERROR(touch_pad_isr_register(touch_intr, pi_task,
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

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "sequence_check",
                              .callback = check_sequence,
                          },
                          &sequence_check_timer),
                      RADIO_TAG, "Failed to initialize sequence check timer");

  uint8_t enabled = 0;
  esp_err_t ret = nvs_get_u8(pi_nvs_handle, "enabled", &enabled);
  if (ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get enabled from NVS");
  }

  ret = nvs_get_u8(pi_nvs_handle, "stage", &current_stage);
  if (ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, RADIO_TAG, "Failed to get stage from NVS");
  }

  frequency_config_t config = {
      .frequency = M_PI,
      .enabled = enabled != 0,
      .entune = entune,
      .detune = detune,
  };

  ESP_RETURN_ON_ERROR(tuner_register_pm_frequency(&config, &freq_handle),
                      RADIO_TAG, "Failed to register pi frequency");

  return ESP_OK;
}

esp_err_t station_pi_enable() {
  ESP_RETURN_ON_ERROR(nvs_set_u8(pi_nvs_handle, "enabled", 1), RADIO_TAG,
                      "Failed to set enabled in NVS");

  ESP_RETURN_ON_ERROR(tuner_enable_pm_frequency(freq_handle), RADIO_TAG,
                      "Failed to enable pi frequency");

  return ESP_OK;
}

esp_err_t station_pi_set_stage(uint8_t stage) {
  current_stage = stage;
  ESP_RETURN_ON_ERROR(nvs_set_u8(pi_nvs_handle, "stage", current_stage),
                      RADIO_TAG, "Failed to save stage");

  return ESP_OK;
}
