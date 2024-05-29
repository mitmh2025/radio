#include <WiFi.h>
#include <Audio.h>
#include <Wire.h>

#define BOARD_VERSION_0_2
#include "boardconfig.h"

#include "localconfig.h"

#define ERR_CHECK(expr) do { \
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

Audio audio;

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

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  // We're using BCLK as the clock input for the TAS2505, and it needs to be on
  // and stable before initializing
  audio.setPinout(I2S_BCLK, I2S_WCLK, I2S_DIN);

  // Re-initialize TAS2505 by pulling RST low briefly, just in case there was
  // power weirdness. 10ns is sufficient (1µs is excessive). Register
  // initialization subsequently takes 1ms
  digitalWrite(TAS2505_RST, LOW);
  delayMicroseconds(1);
  digitalWrite(TAS2505_RST, HIGH);
  delay(1);

  // Section 3.4.12 of the reference guide
  // (https://www.ti.com/lit/ug/slau472c/slau472c.pdf) outlines startup
  // sequence.
  //
  // Values for clock multipliers/dividers should work for 44.1kHz and 48kHz 16
  // bit stereo audio (DAC_fS=44.1kHz or 48kHz, BCLK=1.4112MHz or 1.536MHz) but
  // may not work for other values.
  //
  // DOSR * DAC_fS = DAC_MOD_CLK. 2.8 MHz < DAC_MOD_CLK < 6.2 MHz, DOSR must be
  // a multiple of 8, and higher is better so DOSR=128
  //
  // Next we need to get to CODEC_CLKIN, which is DAC_fS * DOSR * MDAC * NDAC.
  // NDAC should be as large as possible, CODEC_CLKIN must be less than 110 MHz,
  // and MDAC * DOSR/32 >= RC. I think RC is "Resource Class" of the various
  // filter configurations, and is either 4 or 6.
  //
  // DOSR/32 = 4. MDAC=2 is safe for any filter configuration. So NDAC=8, and
  // CODEC_CLKIN=90.318MHz or 98.304MHz
  //
  // CODEC_CLKIN is 2048 * DAC_fS or 64 * BCLK, so we need PLL values to get us
  // there. We don't need fractional multipliers, so PLL_P=1 and PLL_D=0. PLL_J
  // maxes at 63, so we need PLL_J=32 and PLL_R=2  

  uint8_t PLL_P = 1, PLL_R = 2, PLL_J = 32;
  uint8_t NDAC = 8, MDAC = 2;
  uint16_t DOSR = 128; // can be up to 10 bits

  // Switch to page 0
  ERR_CHECK(tas2505I2CWrite(0x0, 0x0));
  // Trigger software reset (likely redundant but just for safety)
  ERR_CHECK(tas2505I2CWrite(0x1, 0x1));
  // Switch to page 1
  ERR_CHECK(tas2505I2CWrite(0x0, 0x1));
  // Power up HP level shifters (probably unnecessary)
  ERR_CHECK(tas2505I2CWrite(0x2, 0x0));
  // Switch to page 0
  ERR_CHECK(tas2505I2CWrite(0x0, 0x0));
  // Set PLL_CLKIN=BCLK, CODEC_CLKIN=PLL_CLK
  ERR_CHECK(tas2505I2CWrite(0x4, 0x7));
  // Power up PLL, set P, R
  ERR_CHECK(tas2505I2CWrite(0x5, 0x80 | (PLL_P << 4) | (PLL_R << 0)));
  // Set J
  ERR_CHECK(tas2505I2CWrite(0x6, PLL_J));
  // Set D=0
  ERR_CHECK(tas2505I2CWrite(0x7, 0x0));
  ERR_CHECK(tas2505I2CWrite(0x8, 0x0));

  // Wait 15ms for PLL to lock
  delay(15);

  // Power up NDAC
  ERR_CHECK(tas2505I2CWrite(0xb, NDAC | 0x80));
  // Power up MDAC
  ERR_CHECK(tas2505I2CWrite(0xc, MDAC | 0x80));
  // Configure OSR
  ERR_CHECK(tas2505I2CWrite(0xd, (0x300 & DOSR) > 8));
  ERR_CHECK(tas2505I2CWrite(0xe, DOSR & 0xff));
  // Set processing block PRB_P2 (simplest, lowest power)
  ERR_CHECK(tas2505I2CWrite(0x3c, 0x2));

  // Power up DAC channel, set to average L+R channel, enable volume soft-stepping
  ERR_CHECK(tas2505I2CWrite(0x3f, 0xb0));
  // Set digital gain to 0dB
  ERR_CHECK(tas2505I2CWrite(0x41, 0x0));
  // Unmute DAC channel
  ERR_CHECK(tas2505I2CWrite(0x40, 0x4));

  // Switch to page 1
  ERR_CHECK(tas2505I2CWrite(0x0, 0x1));
  // Enable master reference
  ERR_CHECK(tas2505I2CWrite(0x1, 0x10));
  // Set analog speaker gain to 0dB
  ERR_CHECK(tas2505I2CWrite(0x2e, 0x0));
  // Set driver volume to 6dB gain
  ERR_CHECK(tas2505I2CWrite(0x30, 0x10));
  // Power up driver
  ERR_CHECK(tas2505I2CWrite(0x2d, 0x2));

  Serial.println("TAS2505 initialized; starting audio stream");

  // Fully initialized, now we can play audio
  audio.connecttohost("https://ebroder.net/assets/take5.mp3");
}

void loop() {
  audio.loop();
}

void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
