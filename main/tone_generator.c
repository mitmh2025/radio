#include "tone_generator.h"
#include "main.h"

#include <math.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_timer.h"

// This tone generation approach is based on this blog post:
// https://jamesmunns.com/blog/fixed-point-math/

// sin lookup from 0 to 2pi in 256 steps, generated in Python with this code:
// [round(0x7fff * math.sin(2 *math.pi * i / 256)) for i in range(256)]
int16_t sin_lookup[256] = {
    0,      804,    1608,   2410,   3212,   4011,   4808,   5602,   6393,
    7179,   7962,   8739,   9512,   10278,  11039,  11793,  12539,  13279,
    14010,  14732,  15446,  16151,  16846,  17530,  18204,  18868,  19519,
    20159,  20787,  21403,  22005,  22594,  23170,  23731,  24279,  24811,
    25329,  25832,  26319,  26790,  27245,  27683,  28105,  28510,  28898,
    29268,  29621,  29956,  30273,  30571,  30852,  31113,  31356,  31580,
    31785,  31971,  32137,  32285,  32412,  32521,  32609,  32678,  32728,
    32757,  32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
    32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,  30273,
    29956,  29621,  29268,  28898,  28510,  28105,  27683,  27245,  26790,
    26319,  25832,  25329,  24811,  24279,  23731,  23170,  22594,  22005,
    21403,  20787,  20159,  19519,  18868,  18204,  17530,  16846,  16151,
    15446,  14732,  14010,  13279,  12539,  11793,  11039,  10278,  9512,
    8739,   7962,   7179,   6393,   5602,   4808,   4011,   3212,   2410,
    1608,   804,    0,      -804,   -1608,  -2410,  -3212,  -4011,  -4808,
    -5602,  -6393,  -7179,  -7962,  -8739,  -9512,  -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530, -18204,
    -18868, -19519, -20159, -20787, -21403, -22005, -22594, -23170, -23731,
    -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105,
    -28510, -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113,
    -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609,
    -32678, -32728, -32757, -32767, -32757, -32728, -32678, -32609, -32521,
    -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852,
    -30571, -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731, -23170,
    -22594, -22005, -21403, -20787, -20159, -19519, -18868, -18204, -17530,
    -16846, -16151, -15446, -14732, -14010, -13279, -12539, -11793, -11039,
    -10278, -9512,  -8739,  -7962,  -7179,  -6393,  -5602,  -4808,  -4011,
    -3212,  -2410,  -1608,  -804,
};

struct tone_generator {
  uint32_t frequency;
  int64_t start_time; // TODO: use this for computing attack/decay
  int64_t attack_time;
  int64_t decay_time;
  uint16_t sustain_level;

  // These are both 8.24 fixed point numbers
  uint32_t increment;
  uint32_t phase;
};

esp_err_t tone_generator_init(tone_generator_config_t *config,
                              tone_generator_t *generator) {
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, RADIO_TAG, "config is NULL");
  ESP_RETURN_ON_FALSE(generator, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "generator is NULL");
  ESP_RETURN_ON_FALSE(config->frequency > 0, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "frequency must be greater than 0");

  *generator = calloc(1, sizeof(struct tone_generator));
  if (*generator == NULL) {
    return ESP_ERR_NO_MEM;
  }

  (*generator)->frequency = config->frequency;
  (*generator)->attack_time = config->attack_time;
  (*generator)->decay_time = config->decay_time;
  (*generator)->sustain_level = config->sustain_level;

  float samples_per_cycle = 48000.0f / config->frequency;
  float float_increment = 256.0f / samples_per_cycle;
  (*generator)->increment = lroundf((1 << 24) * float_increment);
  (*generator)->start_time = esp_timer_get_time();
  (*generator)->phase = 0;

  return ESP_OK;
}

int tone_generator_play(void *ctx, char *data, int len,
                        TickType_t ticks_to_wait) {
  tone_generator_t generator = (tone_generator_t)ctx;
  int16_t *samples = (int16_t *)data;
  size_t sample_count = len / 2;

  int64_t now = esp_timer_get_time();

  for (size_t i = 0; i < sample_count; i++) {
    // Use the first 8 (integer) bits of the phase for the index into the sin
    // table
    uint8_t index = (generator->phase >> 24) & 0xff;

    // Extend the sample before and after our current position to 32-bits so we
    // can scale it up by up to 256 without overflowing (16 bits * 8 bits = 24
    // bits)
    int32_t current_sample = sin_lookup[index];
    int32_t next_sample = sin_lookup[(index + 1) & 0xff]; // wrap around

    // Use the next 8 bits of the phase to interpolate between the two samples
    uint8_t offset = (generator->phase >> 16) & 0xff;

    current_sample *= (256 - offset);
    next_sample *= offset;

    int32_t sample = (current_sample + next_sample) >> 8;
    int64_t time_since_start =
        now - generator->start_time + i * 1000000 / 48000;
    if (time_since_start < generator->attack_time) {
      uint16_t attack_factor =
          (time_since_start * 0xffff) / generator->attack_time;
      sample = (sample * attack_factor) >> 16;
    } else if (time_since_start <
               generator->attack_time + generator->decay_time) {
      int64_t time_since_decay = time_since_start - generator->attack_time;
      // decay_factor needs to go from 0xffff at time_since_decay=0 to
      // sustain_level at time_since_decay=decay_time
      uint16_t decay_factor = 0xffff - (0xffff - generator->sustain_level) *
                                           time_since_decay /
                                           generator->decay_time;
      sample = (sample * decay_factor) >> 16;
    } else if (time_since_start >
               generator->attack_time + generator->decay_time) {
      sample = (sample * generator->sustain_level) >> 16;
    }

    samples[i] = sample;

    generator->phase += generator->increment;
  }

  return len;
}

void tone_generator_free(tone_generator_t generator) { free(generator); }
