#include "static.h"

#include <stdlib.h>

static uint16_t lookback = 0;

// TODO: provide some way of applying gain to the audio (either here or as a
// mixer param)
int static_read_audio(void *ctx, char *data, int len,
                      TickType_t ticks_to_wait) {
  int16_t *samples = (int16_t *)data;
  for (int i = 0; i < len / 2; i++) {
    uint16_t sample = rand() & 0xFFFF;
    lookback = lookback - (lookback >> 2) + (sample >> 2);

    // Hard code a gain of 1/8 for now
    samples[i] = lookback >> 3;
  }
  return len;
}
