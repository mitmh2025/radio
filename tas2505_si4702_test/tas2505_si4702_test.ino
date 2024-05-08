#include <WiFi.h>
#include <Audio.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <SI470X.h>

#define I2C_SDA 42
#define I2C_SCL 41

#define POT1 1
#define POT2 2
#define TOUCH13 13
#define TOUCH14 14

#define FM_I2C_ADDR 0b0010000
#define FM_RST 3
#define FM_SEN 4
#define FM_GPIO1 40
#define FM_GPIO2 39
#define FM_GPIO3 38

#define TAS2505_I2C_ADDR 0b0011000
#define TAS2505_GPIO 33
#define TAS2505_RST 26

#define I2S_DIN 21
#define I2S_WCLK 48
#define I2S_BCLK 47

#define ERR_CHECK(expr) do { \
  bool success = (expr); \
  if (!success) { \
    Serial.printf("%s:%s: Error on I2C", __FILE__, __LINE__); \
    abort(); \
  } \
} while (0)

// uint8_t write(uint8_t reg, uint8_t data) {
//   uint8_t msg[2] = { reg, data };
//   Wire.beginTransmission(TAS2505_I2C_ADDR);
//   Wire.write(msg, sizeof(msg) / sizeof(msg[0]));
//   return Wire.endTransmission();
// }

Audio audio;

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
      // Trigger software reset (likely redundant but just for safety)
      ERR_CHECK(writeReg(TAS2505_REG_SOFTWARE_RESET, 0x1));
      // Power up HP level shifters (probably unnecessary)
      ERR_CHECK(writeReg(TAS2505_REG_LDO_CONTROL, 0x0));
      // Set PLL_CLKIN=BCLK, CODEC_CLKIN=PLL_CLK
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK1, 0x7));
      // Power up PLL, set P, R
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK2, 0x80 | (PLL_P << 4) | (PLL_R << 0)));
      // Set J
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK3, PLL_J));
      // Set D=0
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK4, 0x0));
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK5, 0x0));

      // Wait 15ms for PLL to lock
      delay(15);

      // Power up NDAC
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK6, NDAC | 0x80));
      // Power up MDAC
      ERR_CHECK(writeReg(TAS2505_REG_CLOCK7, MDAC | 0x80));
      // Configure OSR
      ERR_CHECK(writeReg(TAS2505_REG_DOSR1, (0x300 & DOSR) > 8));
      ERR_CHECK(writeReg(TAS2505_REG_DOSR2, DOSR & 0xff));
      // Set processing block PRB_P2 (simplest, lowest power)
      ERR_CHECK(writeReg(TAS2505_REG_DAC_INSTRUCTION_SET, 0x2));

      // Power up DAC channel, set to average L+R channel, enable volume soft-stepping
      ERR_CHECK(writeReg(TAS2505_REG_DAC_SETUP1, 0xb0));
      // Set digital gain to 0dB
      setDACVolume(0);
      // Unmute DAC channel
      ERR_CHECK(writeReg(TAS2505_REG_DAC_SETUP2, 0x4));

      // Enable master reference
      ERR_CHECK(writeReg(TAS2505_REG_POWER_CONTROL, 0x10));

      // Enable AINL and AINR (P1, R9, D1-D0=11)
      ERR_CHECK(writeReg(TAS2505_REG_OUTPUT_CONTROL, 0x03));
      // AINL/R to speaker + HP driver via Mixer P (P1, R12, D7-D6=11, D2=1)
      //ERR_CHECK(writeReg(TAS2505_REG_OUTPUT_ROUTING, AINLR_TO_MIXERP | MIXERP_TO_HP));
      ERR_CHECK(writeReg(TAS2505_REG_OUTPUT_ROUTING, AINR_TO_MIXERP | MIXERP_TO_HP));

      // HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
      //W 30 16 00
      // Enable Mixer P and Mixer M, AINL Voulme, 0dB Gain (P1, R24, D7=1, D6-D0=0000000)
      ERR_CHECK(writeReg(TAS2505_REG_AINL_VOLUME, MIXERPM_FORCE_ENABLE | 0));
      // Enable AINL and AINR and Power up HP (P1, R9, D5=1, D1-D0=11)
      //ERR_CHECK(writeReg(0x0109, 0x23));

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

void setup() {
  Serial.begin(115200);

  Serial.println("Connecting to wifi...");
  WiFi.disconnect();
  //WiFi.mode(WIFI_STA);
  //WiFi.begin("moonhilda", "whatnot-pooh");
  //while (WiFi.status() != WL_CONNECTED) delay(1500);
  //Serial.println("Connected");

  pinMode(TAS2505_RST, OUTPUT);
  pinMode(FM_RST, OUTPUT);
  pinMode(FM_SEN, OUTPUT);
  pinMode(FM_GPIO1, OUTPUT);
  pinMode(FM_GPIO3, OUTPUT);


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

  Serial.println("Initializing TAS2505");
  tas2505.initialize();
  Serial.println("done");

  // Fully initialized, now we can play audio
  //audio.connecttohost("https://ebroder.net/assets/take5.mp3");

  digitalWrite(FM_GPIO3, HIGH);
  digitalWrite(FM_GPIO1, HIGH);

  Serial.println("Starting fm setup");
  fm.setup(FM_RST, -1,-1,-1, 0);
  fm.getAllRegisters();
  for (int i = 0; i < 0x0f; i++) {
    Serial.printf("Register 0x%02X: 0x%04X\n", i, fm.getShadownRegister(i));
  }
  Serial.println("done");

  fm.setVolume(5); // max volume
  delay(500);
  Serial.println("volume set");
  fm.setFrequency(9290); // MHz * 100, 88.7 is clear in Boston
  Serial.println("printf next");
}

void loop() {
  //audio.loop();
  Serial.printf("You are tuned to %u0 kHz | RSSI: %3.3u dbUv | Vol: %2.2u | Stereo: %s\n", fm.getFrequency(), fm.getRssi(), fm.getVolume(), (fm.isStereo()) ? "Yes" : "No");
  delay(10);
}
