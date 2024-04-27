//#include <WiFi.h>
//#include <Audio.h>
#include <Wire.h>

#define I2C_SDA 42
#define I2C_SCL 41

#define FM_I2C_ADDR 0b0010000
#define FM_RST 3
#define FM_SEN 4
#define FM_GPIO1 40
#define FM_GPIO2 39
#define FM_GPIO3 38

void setup()
{
  Serial.begin(115200);
  digitalWrite(RGB_BUILTIN, LOW);
  // Presumably the crystal oscillator stabilizes well before we boot up, but give it a few milliseconds just in case
  delay(10);

  pinMode(FM_RST, OUTPUT);
  pinMode(FM_SEN, OUTPUT);
  pinMode(FM_GPIO1, OUTPUT);
  pinMode(FM_GPIO3, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  digitalWrite(FM_GPIO3, HIGH);
  digitalWrite(FM_GPIO1, HIGH);
  digitalWrite(FM_RST, LOW);
  delay(1);
  digitalWrite(FM_RST, HIGH);
  delay(1);

  Wire.requestFrom(FM_I2C_ADDR, 32);
  for (int i = 0x0A; i <= 0x0F; i++)
  {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    uint16_t value = (high << 8) | low;
    Serial.printf("Register 0x%02X: 0x%04X\n", i, value);
  }

  for (int i = 0x00; i <= 0x09; i++)
  {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    uint16_t value = (high << 8) | low;
    Serial.printf("Register 0x%02X: 0x%04X\n", i, value);
  }
}

void loop()
{
}
