#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	

#include <TimerThree\TimerThree.h>
#include <FlexiTimer2\FlexiTimer2.h>
#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
//#include <avr/wdt.h>
#include <robot/robot.h>
#include "arduino.h"

//
//
struct robot_c robot;



#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	Servo servoSonar;
	NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif
int pos = 0;    // variable to store the servo position


void setup() {
	Serial.begin(SERIAL_BAUD_RATE);

	servoSonar.attach(Pin_ServoSonarPWM);  // attaches the servo on pin 9 to the servo object
	servoSonar.write(90); 
 
	LASER_ON
}
#define wait 1000
void loop() {
  for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoSonar.write(pos);              // tell servo to go to position in variable 'pos'
	Serial.print("alfa:");
	Serial.print(pos);

	Serial.print(" cm:");
	Serial.println(sonar.convert_cm(sonar.ping_median(10)));
    delay(wait);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
	  servoSonar.write(pos);              // tell servo to go to position in variable 'pos'
	  Serial.print("alfa:");
	  Serial.print(pos);

	  Serial.print(" cm:");
	  Serial.println(sonar.convert_cm(sonar.ping_median(10)));
	  delay(30);                       // waits 15ms for the servo to reach the position
  }
}



