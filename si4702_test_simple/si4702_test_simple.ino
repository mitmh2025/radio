#include <WiFi.h>
#include <Audio.h>
#include <Wire.h>
#include <SI470X.h>

#define BOARD_VERSION_0_2
#include "boardconfig.h"

SI470X fm;

void setup() {
  Serial.begin(115200);

  // Presumably the crystal oscillator stabilizes well before we boot up, but give it a few milliseconds just in case
  delay(10);
    Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(FM_RST, OUTPUT);
  pinMode(FM_SEN, OUTPUT);
  pinMode(FM_GPIO1, OUTPUT);
  pinMode(FM_GPIO3, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  digitalWrite(FM_GPIO3, HIGH);
  digitalWrite(FM_GPIO1, HIGH);

  Serial.println("Starting fm setup");
  fm.setup(FM_RST, -1,-1,-1, 0);
  fm.getAllRegisters();
for (int i = 0; i < 0x0f; i++) {
  Serial.printf("Register 0x%02X: 0x%04X\n", i, fm.getShadownRegister(i));
}
  Serial.println("done");

  fm.setVolume(15); // max volume
  delay(500);
  Serial.println("volume set");
  fm.setFrequency(8870); // MHz * 100
  Serial.println("printf next");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.printf("You are tuned to %u MHz | RSSI: %3.3u dbUv | Vol: %2.2u | Stereo: %s\n", fm.getFrequency(), fm.getRssi(), fm.getVolume(), (fm.isStereo()) ? "Yes" : "No");
  delay(10);
}
