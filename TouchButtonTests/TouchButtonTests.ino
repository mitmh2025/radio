/*

This is an example how to use Touch Intrrerupts
The sketh will tell when it is touched and then relesased as like a push-button

This method based on touchInterruptGetLastStatus() is only available for ESP32 S2 and S3
*/

#include "Arduino.h"

int threshold13 = 60000;   // ESP32S2 
int threshold14 = 70000;   // ESP32S2 
bool touch13detected = false;
bool touch14detected = false;

void gotTouch13() {
  touch13detected = true;
}

void gotTouch14() {
  touch14detected = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor

  Serial.println("\n ESP32 Touch Interrupt Test\n");
  touchAttachInterrupt(T13, gotTouch13, threshold13); 
  touchAttachInterrupt(T14, gotTouch14, threshold14);
}

void loop() {
  if (touch13detected) {
    touch13detected = false;
    if (touchInterruptGetLastStatus(T13)) {
        Serial.println(" --- T13 Touched");
    } else {
        Serial.println(" --- T13 Released");
    }
  }
  if (touch14detected) {
    touch14detected = false;
    if (touchInterruptGetLastStatus(T14)) {
        Serial.println(" --- T14 Touched");
    } else {
        Serial.println(" --- T14 Released");
    }
  }

  delay(80);
}
