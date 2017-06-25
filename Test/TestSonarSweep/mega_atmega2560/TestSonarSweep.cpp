#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	
#include <TinyGPSplus/TinyGPS++.h> //deve essere incluso anche nel main

#include <TimerThree\TimerThree.h>
#include <FlexiTimer2\FlexiTimer2.h>
#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM.h>
//#include <avr/wdt.h>
#include <robot/robot.h>
#include "arduino.h"

//
//
int getSonarDistance();
struct robot_c robot;



#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	Servo servoSonar;
	NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif
int pos = 0;    // variable to store the servo position

int wait = 1000;
#define uS_cm 58.24f
int alfa;

void setup() {
	Serial.begin(SERIAL_BAUD_RATE);

	servoSonar.attach(Pin_ServoSonarPWM);  // attaches the servo on pin 9 to the servo object
	servoSonar.write(90); 
 
	LASER_ON
		for (int i = 0; i < 100; i++)
		{
			Serial.println(sonar.ping_cm());
			delay(100);
		}

Serial.println("wait  0  30 60  90 120 150 180 :");

}
void loop() {
#if 0
	if (Serial.available()){
		alfa = Serial.parseInt();
		servoSonar.write(alfa);
	}
	Serial.println(sonar.ping_cm());
	delay(200);
#else
	wait =map( analogRead(Pin_AnaPot1),0,1000,300,2000); Serial.print(wait); Serial.print(" ");
	servoSonar.write(0); Serial.print(sonar.ping_cm()); Serial.print(" "); delay(wait);
	servoSonar.write(30); Serial.print(sonar.ping_cm()); Serial.print(" "); delay(wait);
	servoSonar.write(60); Serial.print(sonar.ping_cm()); Serial.print(" "); delay(wait);
	servoSonar.write(90); Serial.print(sonar.ping_cm()); Serial.print(" "); delay(wait);
	servoSonar.write(120); Serial.print(sonar.ping_cm()); Serial.print(" "); delay(wait);
	servoSonar.write(150); Serial.print(sonar.ping_cm()); Serial.print(" "); delay(wait);
	servoSonar.write(180); Serial.print(sonar.ping_cm()); Serial.println(" "); delay(wait);
 
/*
  for (pos = 0; pos <= 180; pos += 30) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoSonar.write(pos);              // tell servo to go to position in variable 'pos'
	Serial.print("alfa:");
	Serial.print(pos);

	Serial.print(" cm:");
	Serial.println(sonar.convert_cm(sonar.ping_median(5)));
	//Serial.println(getSonarDistance());
    delay(wait);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 15) { // goes from 180 degrees to 0 degrees
	  servoSonar.write(pos);              // tell servo to go to position in variable 'pos'
	  Serial.print("alfa:");
	  Serial.print(pos);

	  Serial.print("> cm:");
	  //Serial.println(sonar.ping_cm());
	  Serial.println(getSonarDistance());
	  delay(wait);                       // waits 15ms for the servo to reach the position
  }
  
  */
#endif // 0
}

int getSonarDistance()
{
	long duration, distanceCm, distanceIn;

	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	digitalWrite(Pin_SonarTrig, LOW);
	delayMicroseconds(2);
	digitalWrite(Pin_SonarTrig, HIGH);
	delayMicroseconds(10);
	digitalWrite(Pin_SonarTrig, LOW);
	duration = pulseIn(Pin_SonarEcho, HIGH);

	// convert the time into a distance
	distanceCm = duration / 29.1 / 2;

	return (int)distanceCm;


}

