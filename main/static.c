#include "static.h"

#include <stdlib.h>

// TODO: provide some way of applying gain to the audio (either here or as a
// mixer param)
int static_read_audio(void *ctx, char *data, int len,
                      TickType_t ticks_to_wait) {
  int16_t *samples = (int16_t *)data;
  for (int i = 0; i < len / 2; i++) {
    // Hard code a gain of 1/8 for now
    samples[i] = (int16_t)(rand() & 0xFFFF) >> 3;
  }
  return len;
}
