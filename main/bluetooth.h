#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
  BLUETOOTH_MODE_DEFAULT,
  BLUETOOTH_MODE_DISABLED,
  BLUETOOTH_MODE_AGGRESSIVE,
  BLUETOOTH_MODE_MAX,
} bluetooth_mode_t;

typedef struct {
  uint16_t major;
  uint16_t minor;
  int8_t rssi;
  int8_t tx_power;
  int64_t last_seen;
  int64_t period;
} bluetooth_beacon_t;

typedef void (*bluetooth_beacon_callback_t)(bluetooth_beacon_t *newest,
                                            bluetooth_beacon_t *strongest,
                                            void *arg);

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bluetooth_init(void);
esp_err_t bluetooth_set_mode(bluetooth_mode_t mode);
esp_err_t bluetooth_subscribe_beacon(uint16_t major,
                                     bluetooth_beacon_callback_t callback,
                                     void *arg);

#ifdef __cplusplus
}
#endif
