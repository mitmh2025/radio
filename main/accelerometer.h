#pragma once

#include "esp_bit_defs.h"
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

typedef enum {
  ACCELEROMETER_PULSE_AXIS_X_POS = BIT(0), // 1
  ACCELEROMETER_PULSE_AXIS_X_NEG = BIT(1), // 2
  ACCELEROMETER_PULSE_AXIS_Y_POS = BIT(2), // 4
  ACCELEROMETER_PULSE_AXIS_Y_NEG = BIT(3), // 8
  ACCELEROMETER_PULSE_AXIS_Z_POS = BIT(4), // 16
  ACCELEROMETER_PULSE_AXIS_Z_NEG = BIT(5), // 32
} accelerometer_pulse_axis_t;

// Remember that these are relative to the accelerometer's position on the
// board, i.e. facing towards the back of the assembled radio
typedef enum {
  ACCELEROMETER_ORIENTATION_TOP_UP = 0,
  ACCELEROMETER_ORIENTATION_BOTTOM_UP = 1,
  ACCELEROMETER_ORIENTATION_LEFT_UP = 2,
  ACCELEROMETER_ORIENTATION_RIGHT_UP = 3,
  ACCELEROMETER_ORIENTATION_FRONT_UP = 4,
  ACCELEROMETER_ORIENTATION_BACK_UP = 5,
} accelerometer_orientation_t;

// Not going to attempt to fully document these, see the MMA8451Q datasheet
typedef struct {
  uint8_t threshold_x;
  uint8_t threshold_y;
  uint8_t threshold_z;
  uint8_t timelimit;
  uint8_t latency;
} accelerometer_pulse_cfg_t;

esp_err_t accelerometer_init(void);
typedef void (*accelerometer_pulse_callback_t)(accelerometer_pulse_axis_t,
                                               void *);
esp_err_t accelerometer_subscribe_pulse(const accelerometer_pulse_cfg_t *cfg,
                                        accelerometer_pulse_callback_t callback,
                                        void *arg);
esp_err_t accelerometer_unsubscribe_pulse(void);
esp_err_t
accelerometer_get_orientation(accelerometer_orientation_t *orientation);

#ifdef __cplusplus
}
#endif
