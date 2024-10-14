#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t webrtc_manager_init();
void webrtc_manager_entune(void *);
void webrtc_manager_detune(void *);

#ifdef __cplusplus
}
#endif
