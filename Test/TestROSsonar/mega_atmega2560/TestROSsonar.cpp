/*
* rosserial SRF08 Ultrasonic Ranger Example
*
* This example is calibrated for the SRF08 Ultrasonic Ranger.
*/


#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	

//#include <TimerThree\TimerThree.h>
//#include <FlexiTimer2\FlexiTimer2.h>
//#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
//#include <PWM\PWM.h>
//#include <avr/wdt.h>
#include <robot/robot.h>



// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////

#if OPT_SERVOSONAR
// va messo prima dell'istanza del robot
#include "arduino.h"

float  getSonarDistance();
//
//
Servo servoSonar;
NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif
int pos = 0;    // variable to store the servo position



struct robot_c robot;
// ////////////////////////////////////////////////////////////////////////////////////////////

/*
* rosserial Ultrasound Example
* using seeestudio ultrasound sensor
* (compatible with PING))))
*/

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range1("/ultrasound1", &range_msg);
ros::Publisher pub_range2("/ultrasound2", &range_msg);
// "/arduino_ultrasound"

char frameid[] = "/ultrasound";

long duration;

/*
float getRange_Ultrasound(int pin_num)
{
	// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	pinMode(pin_num, OUTPUT);
	digitalWrite(pin_num, LOW);
	delayMicroseconds(2);
	digitalWrite(pin_num, HIGH);
	delayMicroseconds(10);
	digitalWrite(pin_num, LOW);

	// The same pin is used to read the signal from the PING))): a HIGH
	// pulse whose duration is the time (in microseconds) from the sending
	// of the ping to the reception of its echo off of an object.
	pinMode(pin_num, INPUT);
	duration = pulseIn(pin_num, HIGH);

	// convert the time into a distance
	return duration / 58; //    duration/29/2, return centimeters
}
*/

float  getSonarDistance()
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

	return (float)distanceCm;


}

void setup()
{
	nh.initNode();
	nh.advertise(pub_range1);
	//nh.advertise(pub_range2);

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.header.frame_id = frameid;
	range_msg.field_of_view = 0.1;  // fake
	range_msg.min_range = 0.0;
	range_msg.max_range = 2.0;
}

long range_time;

void loop()
{
	//publish the adc value every 50 milliseconds
	//since it takes that long for the sensor to stabilize
	if (millis() >= range_time) {
		range_msg.range = getSonarDistance();
		range_msg.header.stamp = nh.now();
		pub_range1.publish(&range_msg);

		//range_msg.range = getSonarDistance(6);
		//range_msg.header.stamp = nh.now();
		//pub_range2.publish(&range_msg);

		range_time = millis() + 50;
	}

	nh.spinOnce();
}

