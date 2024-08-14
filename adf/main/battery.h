#pragma once

#include "esp_err.h"

#define IP5306_I2C_ADDR 0x75

#define IP5306_REG_READ0 0x70
#define IP5306_REG_READ1 0x71

#ifdef __cplusplus
extern "C"
{
#endif

  typedef union
  {
    struct
    {
      uint8_t reserved : 3;
      uint8_t CHARGE_ENABLE : 1;
      uint8_t reserved2 : 4;
    } parsed;

    uint8_t raw;
  } ip5306_reg_read0_t;

  typedef union
  {
    struct
    {
      uint8_t reserved : 4;
      uint8_t CHARGING : 1;
      uint8_t reserved2 : 3;
    } parsed;

    uint8_t raw;
  } ip5306_reg_read1_t;

  esp_err_t battery_init();

#ifdef __cplusplus
}
#endif
