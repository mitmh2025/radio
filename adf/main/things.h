#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  esp_err_t things_init();
  esp_err_t things_provision(const char *token);
  esp_err_t things_deprovision();
  bool things_send_telemetry_string(char const * const key, char const * const value);
  bool things_send_telemetry_double(char const * const key, double value);
  bool things_send_telemetry_int(char const * const key, long long value);
  bool things_send_telemetry_bool(char const * const key, bool value);
  // Functions which will be called periodically to report telemtry. They should
  // call the things_send_telemetry_* functions as needed.
  void things_register_telemetry_generator(void (*generator)(void));

#ifdef __cplusplus
}
#endif
