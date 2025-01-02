#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  THINGS_ATTRIBUTE_TYPE_UNSET = 0,
  THINGS_ATTRIBUTE_TYPE_STRING,
  THINGS_ATTRIBUTE_TYPE_FLOAT,
  THINGS_ATTRIBUTE_TYPE_INT,
  THINGS_ATTRIBUTE_TYPE_BOOL,
  THINGS_ATTRIBUTE_TYPE_MAX,
} things_attribute_type_t;

typedef struct {
  things_attribute_type_t type;
  union {
    const char *string;
    float f;
    long long i;
    bool b;
  } value;
} things_attribute_t;

typedef void (*things_attribute_callback_t)(const char *key,
                                            const things_attribute_t *attr);
typedef void (*things_telemetry_generator_t)(void);
typedef esp_err_t (*things_rpc_handler_t)(const things_attribute_t *param);

esp_err_t things_init();
esp_err_t things_provision(const char *hostname, const char *token);
bool things_send_telemetry_string(char const *const key,
                                  char const *const value);
bool things_send_telemetry_float(char const *const key, float value);
bool things_send_telemetry_int(char const *const key, long long value);
bool things_send_telemetry_bool(char const *const key, bool value);
// Functions which will be called periodically to report telemtry. They should
// call the things_send_telemetry_* functions as needed.
esp_err_t
things_register_telemetry_generator(things_telemetry_generator_t generator,
                                    const char *name, size_t *index);
void things_force_telemetry(size_t index);

esp_err_t things_set_updates_blocked(bool blocked);

esp_err_t things_subscribe_attribute(const char *key,
                                     things_attribute_callback_t callback);
esp_err_t things_register_rpc(const char *method, things_rpc_handler_t handler);

#ifdef __cplusplus
}
#endif
