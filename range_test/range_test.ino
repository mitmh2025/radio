#include <atomic>
#include <inttypes.h>
#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEEddystoneURL.h>
#include <BLEEddystoneTLM.h>
#include <BLEBeacon.h>
#include <U8x8lib.h>
#include "esp_wifi.h"
#include "esp_err.h"
#include "ping/ping_sock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define BOARD_VERSION_0_2
#include "boardconfig.h"

#include "localconfig.h"

U8X8_SSD1306_128X32_UNIVISION_HW_I2C display(I2C_SCL, I2C_SDA);

uint8_t wifi_enabled_icon_top[] = {0x4, 0xe, 0x6, 0x86, 0x83, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x83, 0x86, 0x6, 0x6, 0x4, 0x0};
uint8_t wifi_enabled_icon_bottom[] = {0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0xf0, 0xf0, 0xf0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0};

uint8_t wifi_disabled_icon_top[] = {0x4, 0xe, 0x6, 0x86, 0x83, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xa3, 0x96, 0xe, 0x6, 0x6, 0x1};
uint8_t wifi_disabled_icon_bottom[] = {0x80, 0x40, 0x20, 0x10, 0x9, 0x4, 0xf2, 0xf1, 0xf0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0};

// latency in ms
std::atomic<int16_t> ping_latency(-1);

void on_ping_success(esp_ping_handle_t hdl, void *args)
{
  uint32_t elapsed_time;
  esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
  ping_latency.store(elapsed_time > INT16_MAX ? INT16_MAX : elapsed_time);
}

void on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
  ping_latency.store(-1);
}

void ping_setup()
{
  esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
  ipaddr_aton("8.8.8.8", &config.target_addr);
  config.count = ESP_PING_COUNT_INFINITE;
  esp_ping_callbacks_t cbs = {
      .on_ping_success = on_ping_success,
      .on_ping_timeout = on_ping_timeout,
  };

  esp_ping_handle_t ping;
  esp_ping_new_session(&config, &cbs, &ping);
  esp_ping_start(ping);
}

std::atomic<int> bluetooth_rssi(INT_MIN);
BLEAddress bluetooth_beacon_address(BLUETOOTH_BEACON_MAC);

void bluetooth_loop(void *arg)
{
  BLEDevice::init("");
  BLEScan *scan = BLEDevice::getScan();
  scan->setInterval(3);
  scan->setWindow(3);
  while (true)
  {
    BLEScanResults *results = scan->start(1, false);
    bool found = false;
    for (int i = 0; i < results->getCount(); i++)
    {
      if (results->getDevice(i).getAddress() == bluetooth_beacon_address)
      {
        found = true;
        bluetooth_rssi.store(results->getDevice(i).getRSSI());
        break;
      }
    }
    if (!found)
    {
      bluetooth_rssi.store(INT_MIN);
    }
    scan->clearResults();
  }
}

void setup()
{
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  display.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ping_setup();
  xTaskCreatePinnedToCore(bluetooth_loop, "bluetooth_loop", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  display.drawTile(0, 0, 2, WiFi.isConnected() ? wifi_enabled_icon_top : wifi_disabled_icon_top);
  display.drawTile(0, 1, 2, WiFi.isConnected() ? wifi_enabled_icon_bottom : wifi_disabled_icon_bottom);

  int wifi_rssi;
  esp_err_t wifi_rssi_ok = esp_wifi_sta_get_rssi(&wifi_rssi);
  display.setFont(u8x8_font_8x13_1x2_r);
  display.setCursor(2, 0);
  if (wifi_rssi_ok == ESP_OK)
  {
    display.printf("%4d", wifi_rssi);
  }
  else
  {
    display.print("    ");
  }

  int16_t latency = ping_latency.load();
  if (latency >= 0)
  {
    display.setFont(u8x8_font_open_iconic_embedded_2x2);
    display.drawGlyph(7, 0, 0x4f); // reload
    display.setFont(u8x8_font_8x13_1x2_r);
    display.setCursor(9, 0);
    display.printf("%5" PRIi16 "ms", latency);
  }
  else
  {
    display.setCursor(7, 0);
    display.print("         ");
  }

  int bt_rssi = bluetooth_rssi.load();
  display.setFont(u8x8_font_open_iconic_embedded_2x2);
  display.drawGlyph(0, 2, 0x4a);
  display.setFont(u8x8_font_open_iconic_check_2x2);
  display.drawGlyph(2, 2, bt_rssi == INT_MIN ? 0x44 : 0x40);
  display.setFont(u8x8_font_8x13_1x2_r);
  display.setCursor(4, 2);
  if (bt_rssi != INT_MIN)
  {
    display.printf("%4d", bt_rssi);
  }
  else
  {
    display.print("    ");
  }

  delay(50);
}
