#include <WiFi.h>
#include <Audio.h>
#include <Wire.h>
#include <SI470X.h>
#include <U8x8lib.h>

#define BOARD_VERSION_0_2
#include "boardconfig.h"

#include "localconfig.h"

SI470X fm;
U8X8_SSD1306_128X32_UNIVISION_HW_I2C display(I2C_SCL, I2C_SDA);

bool touch1Pressed = false;
bool touch3Pressed = false;

void touchInterrupt(void *arg) {
  if ((int)arg == 1) touch1Pressed = true;
  else if ((int)arg == 3) touch3Pressed = true;
}

void setup() {
  Serial.begin(115200);

  Serial.print("Connecting to wifi...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.waitForConnectResult();
  Serial.println("connected");

  pinMode(TAS2505_RST, OUTPUT);
  pinMode(FM_RST, OUTPUT);
  #ifdef FM_SEN
  pinMode(FM_SEN, OUTPUT);
  #endif
  #ifdef FM_GPIO1
  pinMode(FM_GPIO1, OUTPUT);
  #endif
  #ifdef FM_GPIO3
  pinMode(FM_GPIO3, OUTPUT);
  #endif

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  fm.setup(FM_RST, -1, -1, -1, OSCILLATOR_TYPE_REFCLK);
  fm.setVolume(15);
  fm.setBand(FM_BAND_USA_EU);
  fm.setSeekThreshold(30);
  delay(500);

  display.begin();
  display.setFont(u8x8_font_8x13_1x2_r);

  touch_value_t touch1Threshold = touchRead(TOUCH1) + 100;
  touch_value_t touch3Threshold = touchRead(TOUCH3) + 100;
  touchAttachInterruptArg(TOUCH1, touchInterrupt, (void *)1, touch1Threshold);
  touchAttachInterruptArg(TOUCH3, touchInterrupt, (void *)3, touch3Threshold);

  display.drawString(0, 0, "Freq: ");
  display.drawString(0, 2, "RSSI: ");
  display.drawString(10, 2, "Mono");
}

uint16_t lastFreq = ~0;
int lastRssi = ~0;
bool lastStereo = false;

void loop() {
  if (touch1Pressed) {
    touch1Pressed = false;
    if (!touchInterruptGetLastStatus(TOUCH1)) {
      fm.seek(SI470X_SEEK_WRAP, SI470X_SEEK_UP);
    }
  }
  if (touch3Pressed) {
    touch3Pressed = false;
    if (!touchInterruptGetLastStatus(TOUCH3)) {
      fm.seek(SI470X_SEEK_WRAP, SI470X_SEEK_DOWN);
    }
  }

  uint16_t freq = fm.getFrequency();
  int rssi = fm.getRssi();
  bool stereo = fm.isStereo();
  if (freq != lastFreq) {
    display.setCursor(6, 0);
    display.printf("%6.2f MHz", freq / 100.0);
    lastFreq = freq;
  }
  if (rssi != lastRssi) {
    display.setCursor(6, 2);
    display.printf("%3u", rssi);
    lastRssi = rssi;
  }
  if (stereo != lastStereo) {
    display.setCursor(10, 2);
    display.print(stereo ? "Stereo" : "Mono  ");
    lastStereo = stereo;
  }
}
