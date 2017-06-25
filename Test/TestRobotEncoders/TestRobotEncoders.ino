/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
//il motore destro va invertito come direzione (altrimenti in senso orario conta negativamente)
#include "hw_config.h"
#include <TimerThree/TimerThree.h>
#include <Encoder.h>
#include <robot/robot.h>

robot_c robot;
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
		Encoder EncR(Pin_EncRa,Pin_EncRb);
		Encoder EncL(Pin_EncLa,Pin_EncLb);

//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("  Test Encoder motori:");
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = EncL.read();
  newRight = EncR.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    EncL.write(0);
    EncR.write(0);
  }
}
