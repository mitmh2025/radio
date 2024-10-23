#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ACCELEROMETER_DR_800HZ = 0,
  ACCELEROMETER_DR_400HZ = 1,
  ACCELEROMETER_DR_200HZ = 2,
  ACCELEROMETER_DR_100HZ = 3,
  ACCELEROMETER_DR_50HZ = 4,
  ACCELEROMETER_DR_12_5HZ = 5,
  ACCELEROMETER_DR_6_25HZ = 6,
  ACCELEROMETER_DR_1_56HZ = 7,
} accelerometer_odr_t;

typedef enum {
  ACCELEROMETER_OSM_NORMAL = 0,
  ACCELEROMETER_OSM_LNLP = 1,
  ACCELEROMETER_OSM_HR = 2,
  ACCELEROMETER_OSM_LP = 3,
} accelerometer_osm_t;

// Not going to attempt to fully document these, see the MMA8451Q datasheet
typedef struct {
  accelerometer_odr_t odr;
  accelerometer_osm_t osm;
  uint8_t threshold;
  uint8_t timelimit;
  uint8_t latency;
} accelerometer_pulse_cfg_t;

esp_err_t accelerometer_init(void);
esp_err_t accelerometer_subscribe_pulse(const accelerometer_pulse_cfg_t *cfg,
                                        void (*callback)(void *), void *arg);
esp_err_t accelerometer_unsubscribe_pulse(void);

#ifdef __cplusplus
}
#endif
