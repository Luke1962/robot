
#include <SoftwareSerial.h>

#include "arduino.h"

//
//
SoftwareSerial cameraSerial(2,3); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  cameraSerial.begin(115200);
}

void loop() { // run over and over
  if (cameraSerial.available()) {
    Serial.write(cameraSerial.read());
  }
  if (Serial.available()) {
    cameraSerial.write(Serial.read());
  }
}

