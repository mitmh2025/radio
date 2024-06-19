#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_EVENT_GROUP_STA_STARTED BIT0
#define WIFI_EVENT_GROUP_STA_CONNECTED BIT1
#define WIFI_EVENT_GROUP_STA_GOT_IP BIT2

extern EventGroupHandle_t wifi_event_group;

void wifi_setup();
