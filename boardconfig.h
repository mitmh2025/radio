#pragma once

#ifdef BOARD_VERSION_0_1

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

#else if BOARD_VERSION_0_2

  #define I2C_SDA 41
  #define I2C_SCL 40

  #define POT1 1
  #define POT2 2
  #define TOUCH1 6
  #define TOUCH2 7
  #define TOUCH3 8

  #define FM_I2C_ADDR 0b0010000
  #define FM_RST 42

  #define TAS2505_I2C_ADDR 0b0011000
  #define TAS2505_RST 15

  #define I2S_DIN 18
  #define I2S_WCLK 17
  #define I2S_BCLK 16

#endif
