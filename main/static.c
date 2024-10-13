#include "static.h"

#include <stdlib.h>

// TODO: provide some way of applying gain to the audio (either here or as a
// mixer param)
int static_read_audio(void *ctx, char *data, int len,
                      TickType_t ticks_to_wait) {
  uint16_t *samples = (uint16_t *)data;
  for (int i = 0; i < len / 2; i++) {
    samples[i] = (uint16_t)(rand() & 0xFFFF);
  }
  return len;
}
