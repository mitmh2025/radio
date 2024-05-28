#include <Wire.h>
#include <AS5600.h> // Grove AS5600 library
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

#define BOARD_VERSION_0_2
#include "boardconfig.h"

AMS_5600 ams5600;

int ang, lang = 0;

void setup()
{
  SERIAL.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
//  if(ams5600.detectMagnet() == 0 ){
//    while(1){
//        if(ams5600.detectMagnet() == 1 ){
//            SERIAL.print("Current Magnitude: ");
//            SERIAL.println(ams5600.getMagnitude());
//            break;
//        }
//        else{
//            SERIAL.println("Can not detect magnet");
//        }
//        delay(1000);
//    }
//  }
}
/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}
void loop()
{
    SERIAL.println(String(convertRawAngleToDegrees(ams5600.getRawAngle()),DEC));
    delay(50);
}
