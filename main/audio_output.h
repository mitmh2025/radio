#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t audio_output_init();
esp_err_t audio_output_suspend();
esp_err_t audio_output_resume();

#ifdef __cplusplus
}
#endif
