
//#include <SoftwareSerial.h>
//SoftwareSerial cameraSerial(2,3); // RX, TX

#define cameraSerial Serial1

#include "arduino.h"

//
//
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  cameraSerial.begin(115200);
}

void loop() { // run over and over
  if (cameraSerial.available()) {
    Serial.write(cameraSerial.read());
  }else if (Serial.available()) {
    cameraSerial.write(Serial.read());
  }
}

