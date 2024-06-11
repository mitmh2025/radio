#include <WiFi.h>
#include <Audio.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <SI470X.h>
#include <U8x8lib.h>

#define BOARD_VERSION_0_2
#include "boardconfig.h"

#include "localconfig.h"

#define ERR_CHECK(expr) do { \
  bool success = (expr); \
  if (!success) { \
    Serial.printf("%s:%s: Error on I2C", __FILE__, __LINE__); \
    abort(); \
  } \
} while (0)

#define TAS2505_REG_PAGE_SELECT 0x0000
#define TAS2505_REG_SOFTWARE_RESET 0x0001
#define TAS2505_REG_CLOCK1 0x0004
#define TAS2505_REG_CLOCK2 0x0005
#define TAS2505_REG_CLOCK3 0x0006
#define TAS2505_REG_CLOCK4 0x0007
#define TAS2505_REG_CLOCK5 0x0008
#define TAS2505_REG_CLOCK6 0x000B
#define TAS2505_REG_CLOCK7 0x000C
#define TAS2505_REG_DOSR1 0x000D
#define TAS2505_REG_DOSR2 0x000E
#define TAS2505_REG_DAC_INSTRUCTION_SET 0x003C
#define TAS2505_REG_DAC_SETUP1 0x003F
#define TAS2505_REG_DAC_SETUP2 0x0040
#define TAS2505_REG_DAC_VOLUME 0x0041
#define TAS2505_REG_POWER_CONTROL 0x0101
#define TAS2505_REG_LDO_CONTROL 0x0102
#define TAS2505_REG_OUTPUT_CONTROL 0x0109
#define TAS2505_REG_OUTPUT_ROUTING 0x010C
#define AINR_TO_HP 0x01
#define AINL_TO_HP 0x02
#define MIXERP_TO_HP 0x04
#define DAC_TO_HP 0x08
#define AINR_TO_MIXERP 0x40
#define AINLR_TO_SPEAKER_INV 0x60
#define AINL_TO_MIXERP 0x80
#define AINLR_TO_SPEAKER 0x90
#define AINLR_TO_MIXERP 0xC0
#define TAS2505_REG_HP_VOLUME 0x0116
#define TAS2505_REG_AINL_VOLUME 0x0118
#define MIXERPM_FORCE_ENABLE 0x80
#define TAS2505_REG_SPEAKER_CONTROL 0x012D
#define TAS2505_REG_SPEAKER_VOLUME 0x012E
#define TAS2505_REG_SPEAKER_VOLUME_RANGE 0x0130

class TAS2505 : Adafruit_I2CDevice {
    public:
    TAS2505(TwoWire *theWire = &Wire) : Adafruit_I2CDevice(TAS2505_I2C_ADDR, theWire) {

    };

    protected:
    uint8_t currentPage = 0xFF;
  
    bool writeReg(uint16_t reg, uint8_t data) {
      uint8_t page = reg >> 8;
      uint8_t ureg = reg & 0xFF;
      Serial.printf("TAS2505 R%02xP%02x = %02x\n", page, ureg, data);
      setPage(page);
      return write(&data, 1, true, &ureg, 1);
    }

    bool setPage(uint8_t page) {
      uint8_t reg = TAS2505_REG_PAGE_SELECT;
      if (page != currentPage) {
        if (!write(&page, 1, true, &reg, 1)) {
          return false;
        }
        currentPage = page;
      }
      return true;
    }

    public:
    void initialize() {
      // Trigger software reset (likely redundant but just for safety)
      ERR_CHECK(writeReg(TAS2505_REG_SOFTWARE_RESET, 0x1));
      // Power up HP level shifters (probably unnecessary)
      ERR_CHECK(writeReg(TAS2505_REG_LDO_CONTROL, 0x0));

      // Enable master reference
      ERR_CHECK(writeReg(TAS2505_REG_POWER_CONTROL, 0x10));
      // Enable AINL and AINR (P1, R9, D1-D0=11)
      ERR_CHECK(writeReg(TAS2505_REG_OUTPUT_CONTROL, 0x03));
      // AINL/R to speaker + HP driver via Mixer P (P1, R12, D7-D6=11, D2=1)
      ERR_CHECK(writeReg(TAS2505_REG_OUTPUT_ROUTING, AINLR_TO_MIXERP | MIXERP_TO_HP));

      // Enable Mixer P and Mixer M, AINL Voulme, 0dB Gain (P1, R24, D7=1, D6-D0=0000000)
      ERR_CHECK(writeReg(TAS2505_REG_AINL_VOLUME, MIXERPM_FORCE_ENABLE | 0));
      // Enable AINL and AINR and Power up HP (P1, R9, D5=1, D1-D0=11)
      ERR_CHECK(writeReg(0x0109, 0x23));

      // Unmute HP with 0dB gain (P1, R16, D4=1)
      //ERR_CHECK(writeReg(0x0110, 0));

      // Set analog speaker gain to 0dB
      setSpeakerVolume(0);
      // Set driver volume to 18dB gain
      ERR_CHECK(writeReg(TAS2505_REG_SPEAKER_VOLUME_RANGE, 0x30));
      // Power up driver
      setSpeakerPower(true);
    }

    // Set digital volume in 0.5 dB steps from -127 (-63.5 dB) to +48 (+24 dB).
    void setDACVolume(int8_t volume) {
      if (volume > 48) {
        volume = 48;
      }
      ERR_CHECK(writeReg(TAS2505_REG_DAC_VOLUME, (uint8_t)volume));
    }

    // Set speaker volume in ~0.5 dB steps from +0 (0 dB) to -116 (-72.3 dB).
    void setSpeakerVolume(int8_t volume) {
      if (volume > 0) volume = 0;
      uint8_t value = (uint8_t) -volume;
      if (value > 0x74) {
        value = 0x7F; // mute
      }
      ERR_CHECK(writeReg(TAS2505_REG_SPEAKER_VOLUME, value));
    }
    void setSpeakerPower(bool power) {
      ERR_CHECK(writeReg(TAS2505_REG_SPEAKER_CONTROL, power ? 0x02 : 0));
    }
};

TAS2505 tas2505;

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

  analogReadResolution(8);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  // Re-initialize TAS2505 by pulling RST low briefly, just in case there was
  // power weirdness. 10ns is sufficient (1µs is excessive). Register
  // initialization subsequently takes 1ms
  digitalWrite(TAS2505_RST, LOW);
  delayMicroseconds(1);
  digitalWrite(TAS2505_RST, HIGH);
  delay(1);
  tas2505.initialize();

  fm.setup(FM_RST, -1, -1, -1, OSCILLATOR_TYPE_REFCLK);
  fm.setVolume(15);
  fm.setBand(FM_BAND_USA_EU);
  fm.setSeekThreshold(30);
  fm.seek(SI470X_SEEK_WRAP, SI470X_SEEK_UP);
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

bool tas2505Active = true;

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
      if (tas2505Active) {
        digitalWrite(TAS2505_RST, LOW);
        delayMicroseconds(1);
        digitalWrite(TAS2505_RST, HIGH);
        delay(1);
        tas2505Active = false;
      } else {
        tas2505.initialize();
        tas2505Active = true;
      }
    }
  }

  uint16_t freq = fm.getFrequency();
  int rssi = fm.getRssi();
  bool stereo = fm.isStereo();
  if (freq != lastFreq) {
    display.setCursor(6, 0);
    display.printf("%6.1f MHz", freq / 100.0);
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

  uint16_t volume = analogRead(POT1);
  tas2505.setSpeakerVolume(volume);
}
