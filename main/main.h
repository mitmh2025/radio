#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const char *RADIO_TAG;

#define RADIO_EVENT_GROUP_WIFI_DISCONNECTED BIT0
#define RADIO_EVENT_GROUP_WIFI_CONNECTED BIT1
#define RADIO_EVENT_GROUP_THINGS_PROVISIONED BIT2
#define RADIO_EVENT_GROUP_THINGS_FORCE_TELEMETRY BIT3
extern EventGroupHandle_t radio_event_group;

#ifdef __cplusplus
}
#endif
