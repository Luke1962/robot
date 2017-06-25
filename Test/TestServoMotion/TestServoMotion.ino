#include <Servo.h> 
#include <NewPing.h>

//#define TRIGGER_PIN  30  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//#define ECHO_PIN     32  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define Pin_SERVO 9
#define SCANSPEED 1000	//OK 60	// considera 3ms al metro , con max_distance 2m, e 5 impulsi per posizione ci vogliono 3*2*5=30ms
#define StepAngle 25 //OK 10 //ampiezza step in gradi della scansione
#define SCANANGLE 180
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myservo; 
 

//int ping_delay = 50;
//long last_ping_time = 0;
//long duration; // duration of return pulse

int pos;

 void servoGoTo(int angle) {
	myservo.write(angle);
	Serial.println(angle);
	delay(SCANSPEED);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

}

void setup() {
  Serial.begin(115200);
  Serial.println("Test Servo");
	myservo.attach(Pin_SERVO);  // attaches the servo on pin 8 to the servo object 
	servoGoTo(90);
	servoGoTo(30);
	servoGoTo(120);
	servoGoTo(90);
   // initialize serial communication
}

void loop()
{


  for(pos = 0; pos <= SCANANGLE; pos += StepAngle) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of n degrees 
	  servoGoTo(pos);
	
  } 
  for(pos = SCANANGLE; pos>=0; pos-=StepAngle)     // goes from 180 degrees to 0 degrees 
  {                                
	myservo.write(pos);              // tell servo to go to position in variable 'pos' 
	//delay(15);                       // waits 15ms for the servo to reach the position 
	delay(SCANSPEED);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
	//uS = sonar.ping();	//	.ping_median(2);	//.ping_median(3);		//.ping(); // Send ping, get ping time in microseconds (uS).
	// convert the time into a distance
	//cm = uS / US_ROUNDTRIP_CM; // Convert ping time to distance in cm and print result (0 = outside set distance range)  
	//writeData(cm, pos);   
	Serial.print(pos);
	
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
  buffer[0] = (byte) data;
  buffer[1] = (byte) data >> 8;
  buffer[2] = (byte) data >> 16;
  buffer[3] = (byte) data >> 24;
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
