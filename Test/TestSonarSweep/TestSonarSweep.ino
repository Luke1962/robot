#define SERIAL_BAUD_RATE 115200
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	
#include <TinyGPSplus/TinyGPS++.h> //deve essere incluso anche nel main

#include <TimerThree\TimerThree.h>
#include <FlexiTimer2\FlexiTimer2.h>


#include <MyRobotLibs\hw_config.h>





#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>

	// va messo prima dell'istanza del robot
	Servo myServoSonar;
	NewPing MySonar(Pin_SonarTrig, Pin_SonarEcho);

	int pos = 0;    // variable to store the servo position

int wait = 1000;
#define uS_cm 58.24f
int alfa;

	// Converte la lettura del potenziometro (da 0 a 1023) in gradi
int calcolaRotazione(int r)
{
	int posToDeg;
	//posToDeg = analogRead(Pin_AnaPot1);

	if (Serial.available()) {
		posToDeg = Serial.parseInt();
		return	posToDeg;
	}
	else
	{
		return	r;
	}

	// return (map(posToDeg, 0, 1023, 1, 180));
}
// Converte la lettura del potenziometro (da 0 a 1023) in gradi
int calcolaDelay(int v)
{
	int val;
	//posToDeg = analogRead(Pin_AnaPot1);

	if (Serial.available()) {
		val = Serial.parseInt();
		Serial.print("ok:"); Serial.println(val);
		return val;	//map(val, 0, 1000, 100, 2000);
	}
	else
	{
		return	v;
	}

	// return (map(posToDeg, 0, 1023, 1, 180));
}

void setup() {
	Serial.begin(SERIAL_BAUD_RATE);
	//robot.initServoAndSonar(&myServoSonar, &MySonar);
	myServoSonar.attach(Pin_ServoSonarPWM);  // attaches the servo on pin 9 to the servo object
	myServoSonar.write(90); 
 			delay(1000);

		for (int i = 0; i < 180; i+=20)
		{
			//Serial.println(MySonar.ping_cm());
			Serial.println(i);
			delay(1000);
			myServoSonar.write(i); 
		}
	myServoSonar.attach(Pin_ServoSonarPWM);

//Serial.println("wait  0  30 60  90 120 150 180 :");

}

int r=90; int dist; 
int w = 100; // wait di default tra due ping
#if 1
float maxStdDev = 0.0;
void loop() {

	//r = calcolaRotazione(r);
	for (size_t i = 0; i < 180; i+=5)
	{
	maxStdDev = 15;
		
		dist = robot.SonarPingAtPosAvg(i, &maxStdDev, 10 );
 		Serial.print(i); Serial.print("\t"); Serial.print(dist); Serial.print("\t"); Serial.println(maxStdDev);
		w = calcolaDelay(w);
		delay(w);
	}
	


}
#endif // 1


#if 0	// scenario 1  : sweep ------------------------------------------
void loop() {
	for (int i = 0; i < 181; i += 5)
	{
		LASER_ON
		Serial.print(i); Serial.print("\t"); Serial.println(robot.SonarPingAtPos(i));

		wait = map(analogRead(Pin_AnaPot1), 0, 1000, 300, 2000);
		//		Serial.print(wait); Serial.print(" ");
		LASER_OFF

		delay(wait);
	}
}
#endif

#if 0
void loop() {

	wait =map( analogRead(Pin_AnaPot1),0,1000,300,2000); Serial.print(wait); Serial.print(" ");
	myServoSonar.write(0); Serial.print(MySonar.ping_cm()); Serial.print(" "); delay(wait);
	myServoSonar.write(30); Serial.print(MySonar.ping_cm()); Serial.print(" "); delay(wait);
	myServoSonar.write(60); Serial.print(MySonar.ping_cm()); Serial.print(" "); delay(wait);
	myServoSonar.write(90); Serial.print(MySonar.ping_cm()); Serial.print(" "); delay(wait);
	myServoSonar.write(120); Serial.print(MySonar.ping_cm()); Serial.print(" "); delay(wait);
	myServoSonar.write(150); Serial.print(MySonar.ping_cm()); Serial.print(" "); delay(wait);
	myServoSonar.write(180); Serial.print(MySonar.ping_cm()); Serial.println(" "); delay(wait);
} 
#endif


#if 0
void loop() {


  for (pos = 0; pos <= 180; pos += 30) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServoSonar.write(pos);              // tell servo to go to position in variable 'pos'
	Serial.print("alfa:");
	Serial.print(pos);

	Serial.print(" cm:");
	Serial.println(MySonar.convert_cm(MySonar.ping_median(5)));
	//Serial.println(getSonarDistance());
    delay(wait);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 15) { // goes from 180 degrees to 0 degrees
	  myServoSonar.write(pos);              // tell servo to go to position in variable 'pos'
	  Serial.print("alfa:");
	  Serial.print(pos);

	  Serial.print("> cm:");
	  //Serial.println(MySonar.ping_cm());
	  Serial.println(getSonarDistance());
	  delay(wait);                       // waits 15ms for the servo to reach the position
  }
  
  
}
#endif // 0


int getSonarDistance()
{
	long duration, distanceCm;

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