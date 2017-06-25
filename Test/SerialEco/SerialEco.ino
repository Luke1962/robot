
//#include <SoftwareSerial.h>
//SoftwareSerial SerialToBeEchoed(2,3); // RX, TX

#define SerialToBeEchoed Serial2

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  SerialToBeEchoed.begin(115200);
}

void loop() { // run over and over
  if (SerialToBeEchoed.available()) {
    Serial.write(SerialToBeEchoed.read());
  }else if (Serial.available()) {
    SerialToBeEchoed.write(Serial.read());
  }
}