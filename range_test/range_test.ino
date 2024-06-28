#include <atomic>
#include <inttypes.h>
#include <arpa/inet.h>
#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <U8x8lib.h>
#include "esp_wifi.h"
#include "esp_err.h"
#include "ping/ping_sock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"

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
  uint8_t ttl;
  uint16_t seqno;
  uint32_t elapsed_time, recv_len;
  ip_addr_t target_addr;
  esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
  esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
  esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
  esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
  esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
  Serial.printf("%d bytes from %s icmp_seq=%d ttl=%d time=%d ms\n",
                recv_len, inet_ntoa(target_addr.u_addr.ip4), seqno, ttl, elapsed_time);
  ping_latency.store(elapsed_time > INT16_MAX ? INT16_MAX : elapsed_time);
}

void on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
  if (WiFi.isConnected() && ping_latency.load() != -1) {
    WiFi.reconnect();
  }
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
esp_bd_addr_t bluetooth_beacon_address = {};
esp_timer_handle_t bluetooth_scan_timeout_timer;
uint64_t bluetooth_scan_timeout_us = 1000000;

static void bluetooth_scan_timeout(void *arg)
{
  bluetooth_rssi.store(INT_MIN);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event)
  {
  case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    esp_ble_gap_start_scanning(0);
    esp_timer_start_once(bluetooth_scan_timeout_timer, bluetooth_scan_timeout_us);
    break;
  case ESP_GAP_BLE_SCAN_RESULT_EVT:
    if (memcmp(param->scan_rst.bda, bluetooth_beacon_address, ESP_BD_ADDR_LEN) == 0)
    {
      bluetooth_rssi.store(param->scan_rst.rssi);
      esp_err_t err = esp_timer_restart(bluetooth_scan_timeout_timer, bluetooth_scan_timeout_us);
      if (err == ESP_ERR_INVALID_STATE)
      {
        esp_timer_start_once(bluetooth_scan_timeout_timer, bluetooth_scan_timeout_us);
      }
    }
    break;
  }
}

void bluetooth_setup()
{
  sscanf(BLUETOOTH_BEACON_MAC, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &bluetooth_beacon_address[0], &bluetooth_beacon_address[1], &bluetooth_beacon_address[2],
         &bluetooth_beacon_address[3], &bluetooth_beacon_address[4], &bluetooth_beacon_address[5]);

  esp_timer_create_args_t timer_config = {
      .callback = bluetooth_scan_timeout,
      .name = "bluetooth_scan_timeout",
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_config, &bluetooth_scan_timeout_timer));
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);

  esp_bluedroid_init();
  esp_bluedroid_enable();
  ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
  esp_ble_scan_params_t ble_scan_params = {
      .scan_type = BLE_SCAN_TYPE_PASSIVE,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval = 0x50,
      .scan_window = 0x30,
      .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
  };
  esp_ble_gap_set_scan_params(&ble_scan_params);
}

void setup()
{
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  display.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  esp_wifi_set_ps(WIFI_PS_NONE);
  ping_setup();
  bluetooth_setup();
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

  uint32_t batt_mv = analogReadMilliVolts(9);
  display.setFont(u8x8_font_open_iconic_embedded_2x2);
  display.drawGlyph(9, 2, 0x49);
  display.setFont(u8x8_font_8x13_1x2_r);
  display.setCursor(11, 2);
  display.printf("%.2fV", (float)batt_mv * 3 / (2 * 1000));

  delay(50);
}
