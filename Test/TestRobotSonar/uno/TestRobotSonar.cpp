// ---------------------------------------------------------------------------
// attende un angolo , ritorna lettura sonar a quell'angolo
// ---------------------------------------------------------------------------
#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS

//#include <FrequencyTimer2\FrequencyTimer2.h>	
#include <TimerThree\TimerThree.h>
#include <FlexiTimer2\FlexiTimer2.h>
#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
#include <avr/wdt.h>
#include <robot/robot.h>

#if OPT_SERVOSONAR
// va messo prima dell'istanza del robot
#include "arduino.h"

//
//
void writeData(float cm, float angle);
void checkPing();
float microsecondsToCentimeters(long microseconds);
void floatToBuffer(byte *buffer, float data);
void longToBuffer(byte *buffer, long data);
Servo servoSonar;
NewPing Sonar( Pin_SonarTrig, Pin_SonarEcho );
#endif

struct robot_c robot;

int cm = 0;
int pos = 0;


#define TRIGGER_PIN  30  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     32  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define SCANSPEED 50	//OK 60	// considera 3ms al metro , con max_distance 2m, e 5 impulsi per posizione ci vogliono 3*2*5=30ms
#define StepAngle 8 //OK 10 //ampiezza step in gradi della scansione
#define SCANANGLE 180
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myservo;


int ping_delay = 50;
long last_ping_time = 0;
long duration; // duration of return pulse

boolean new_reading = false;
const int max_write_delay = 50;
long last_write_time = 0;

const int bufSize = 4;
float cm;
unsigned int uS;

boolean stopMotor = false;
int rpm = 15; // default RPM
int pos = 0;    // variable to store the servo position 
int i = 0;



void setup() {
	myservo.attach(8);  // attaches the servo on pin 8 to the servo object 
	myservo.write(0);
	// initialize serial communication
	Serial.begin( SERIAL_BAUD_RATE ); // Open serial monitor at 115200 baud to see ping results.
	Serial.print("attendo un angolo , ritorno lettura sonar a quell'angolo ");

	#if  OPT_SERVOSONAR  
		//NewPing Sonar(Pin_SonarTrig,Pin_SonarEcho,robot.parameter.sonarMaxDistance);
		// passo a robot gli indirizzi  di servoSonar e Sonar
		robot.initServoAndSonar( &servoSonar, &Sonar );
	#endif


}

void loop()
{
	LEDTOP_ON


	for (pos = 0; pos <= SCANANGLE; pos += StepAngle) // goes from 0 degrees to 180 degrees 
	{                                  // in steps of n degrees 
		myservo.write(pos);              // tell servo to go to position in variable 'pos'     
		delay(SCANSPEED);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
											   //   uS = 0;
											   //for (i = 0; i<4; i++){
											   //		uS+= sonar.ping() ;
											   //		delay(PING_MEDIAN_DELAY); 
											   //	}
											   //uS = uS >> 2;// divido per 4
		uS = sonar.ping();		//.ping();		//_median(3); // Send ping, get ping time in microseconds (uS).

								// convert the time into a distance
		cm = uS / US_ROUNDTRIP_CM; // Convert ping time to distance in cm and print result (0 = outside set distance range)  
		writeData(cm, pos);
	}
	for (pos = SCANANGLE; pos >= 0; pos -= StepAngle)     // goes from 180 degrees to 0 degrees 
	{
		myservo.write(pos);              // tell servo to go to position in variable 'pos' 
										 //delay(15);                       // waits 15ms for the servo to reach the position 
		delay(SCANSPEED);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
		uS = sonar.ping();	//	.ping_median(2);	//.ping_median(3);		//.ping(); // Send ping, get ping time in microseconds (uS).
							// convert the time into a distance
		cm = uS / US_ROUNDTRIP_CM; // Convert ping time to distance in cm and print result (0 = outside set distance range)  
		writeData(cm, pos);

	}
	LEDTOP_OFF

}

void writeData(float cm, float angle)
{
	byte buffer[bufSize];

	// fill the buffer with the distance reading
	floatToBuffer(buffer, cm);
	Serial.write(buffer, bufSize);
	//Serial.print(cm);
	//Serial.print(" cm @ ");

	// fill the buffer with the angle
	floatToBuffer(buffer, angle);
	Serial.write(buffer, bufSize);
	//Serial.print(angle);
	//Serial.println(" degrees");
}

void checkPing()
{
	if (sonar.check_timer()) {
		// response was received from ping sensor
		new_reading = true;

		duration = sonar.ping_result;
	}
}

float microsecondsToCentimeters(long microseconds)
{
	// speed of sound = 343.2 m/s =>
	// 29.137529 microseconds/cm conversion factor

	// Using float
	float cm = float(microseconds) / 29.137529 / 2;

	// Using long
	//long cm = microseconds / 29 / 2;

	return cm;
}

void floatToBuffer(byte *buffer, float data)
{
	long *dataPtr = (long*)(&data);

	// place the 4 bytes in a byte array

	buffer[0] = *dataPtr;
	buffer[1] = (*dataPtr >> 8);
	buffer[2] = (*dataPtr >> 16);
	buffer[3] = (*dataPtr >> 24);
}

void longToBuffer(byte *buffer, long data)
{
	buffer[0] = (byte)data;
	buffer[1] = (byte)data >> 8;
	buffer[2] = (byte)data >> 16;
	buffer[3] = (byte)data >> 24;
}

//unsigned int AvgPing( int it=3) {
//	_maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2);
//	int uS[it], last;
//	uint8_t j, i = 0;
//	uS[0] = NO_ECHO;
//	while (i < it) {
//		last = sonar.ping();           // Send ping.
//		if (last == NO_ECHO) {   // Ping out of range.
//			it--;                // Skip, don't include as part of median.
//			last = _maxEchoTime; // Adjust "last" variable so delay is correct length.
//		} else {                       // Ping in range, include as part of median.
//			if (i > 0) {               // Don't start sort till second ping.
//				for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
//					uS[j] = uS[j - 1]; // Shift ping array to correct position for sort insertion.
//			} else j = 0;              // First ping is starting point for sort.
//			uS[j] = last;              // Add last ping to array in sorted position.
//			i++;                       // Move to next ping.
//		}
//		if (i < it) delay(PING_MEDIAN_DELAY - (last >> 10)); // Millisecond delay between pings.
//	}
//	return (uS[it >> 1]); // Return the ping distance median.
//}

