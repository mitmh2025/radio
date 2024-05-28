#include <WiFi.h>
#include <Audio.h>
#include <Wire.h>
#include <SI470X.h>

#define BOARD_VERSION_0_2
#include "boardconfig.h"

#define ERR_CHECK(expr) \
  do { \
    int error = (expr); \
    if (error > 0) { \
      Serial.printf("%s:%s: Error on I2C: %d", __FILE__, __LINE__, error); \
      abort(); \
    } \
  } while (0)

uint8_t tas2505I2CWrite(uint8_t reg, uint8_t data) {
  uint8_t msg[2] = { reg, data };
  Wire.beginTransmission(TAS2505_I2C_ADDR);
  Wire.write(msg, sizeof(msg) / sizeof(msg[0]));
  return Wire.endTransmission();
}

SI470X fm;

void setup() {
  Serial.begin(115200);

  // Presumably the crystal oscillator stabilizes well before we boot up, but give it a few milliseconds just in case
  delay(10);

  pinMode(TAS2505_RST, OUTPUT);
  pinMode(FM_RST, OUTPUT);
  pinMode(FM_SEN, OUTPUT);
  pinMode(FM_GPIO1, OUTPUT);
  pinMode(FM_GPIO3, OUTPUT);

  fm.setup(FM_RST, I2C_SDA, OSCILLATOR_TYPE_REFCLK);
  fm.setVolume(15); // max volume
  delay(500);
  fm.setFrequency(8810); // MHz * 100, 881000

  // Re-initialize TAS2505 by pulling RST low briefly, just in case there was
  // power weirdness. 10ns is sufficient (1µs is excessive). Register
  // initialization subsequently takes 1ms
  digitalWrite(TAS2505_RST, LOW);
  delayMicroseconds(1);
  digitalWrite(TAS2505_RST, HIGH);
  delay(1);

  // Note that for purely analog inputs, we don't need to configure clock or DAC

  // Switch to page 0
  ERR_CHECK(tas2505I2CWrite(0x0, 0x0));
  // Trigger software reset (likely redundant but just for safety)
  ERR_CHECK(tas2505I2CWrite(0x1, 0x1));
  // Switch to page 1
  ERR_CHECK(tas2505I2CWrite(0x0, 0x1));
  // Power up HP level shifters (probably unnecessary)
  ERR_CHECK(tas2505I2CWrite(0x2, 0x0));
  // Enable master reference
  ERR_CHECK(tas2505I2CWrite(0x1, 0x10));
  // Enable analog inputs
  ERR_CHECK(tas2505I2CWrite(0x9, 0x3));
  // Route AINL/AINR to Mixer P
  ERR_CHECK(tas2505I2CWrite(0xc, 0xc0));
  // Enable Mixer P, set AINL gain to 0dB
  ERR_CHECK(tas2505I2CWrite(0x18, 0x80));
  // Set analog speaker gain to 0dB
  ERR_CHECK(tas2505I2CWrite(0x2e, 0x0));
  // Set driver volume to 6dB gain
  ERR_CHECK(tas2505I2CWrite(0x30, 0x10));
  // Power up driver
  ERR_CHECK(tas2505I2CWrite(0x2d, 0x2));
}

void loop() {
  Serial.printf("You are tuned to %u MHz | RSSI: %3.3u dbUv | Vol: %2.2u | Stereo: %s\n", fm.getFrequency(), fm.getRssi(), fm.getVolume(), (fm.isStereo()) ? "Yes" : "No");
  delay(1000);
}
