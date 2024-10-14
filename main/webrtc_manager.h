#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t webrtc_manager_init();
void webrtc_manager_entune();
void webrtc_manager_detune();

#ifdef __cplusplus
}
#endif
