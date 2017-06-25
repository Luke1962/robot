#include <hw_config.h>
#include <NewPing.h>
#include <AccelStepper.h>
#include <arduino.h>
#define TRIG_PIN 30 //Arduino pin for trigger
#define ECHO_PIN 32 //Arduino pin for ECHO_PIN
#define MAX_DISTANCE 200 //Max dist for ping in cm


#define HOMEPOSITION 0

AccelStepper stepper(AccelStepper::DRIVER,Pin_MotCK,Pin_MotCWR,Pin_MotENR); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
//NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE)

void setup()
{
	Serial.begin(9600); //Open serial at 115200 baud

	digitalWrite(Pin_Rele2,0);
	digitalWrite(Pin_Rele1,1);




	stepper.setMaxSpeed(250);// steps per second
	stepper.setAcceleration(500);
	stepper.setEnablePin(Pin_MotSonarEN);
	stepper.setMinPulseWidth(1);

	//stepper.setSpeed(10);		// steps per second

	stepper.moveTo(300);		// porta a fondocorsa
	while (!stepper.run()){};
	Serial.print("\n Fondocorsa\n");

	stepper.setCurrentPosition(HOMEPOSITION);

	stepper.setSpeed(3);		// steps per second


	delay(50); //Wait 50ms between pings

}	

void loop()
{  
 //	digitalWrite(Pin_Rele1,1);

	//stepper.moveTo(500);
 // while (stepper.currentPosition() != 300) // Full speed up to 300
 //   stepper.run();
 // stepper.runToNewPosition(0); // Cause an overshoot then back to 0
 //

	int i;

    for (i = 1; i < 200; i+=10)
    {
		Serial.print("\n move to pos:");		
		Serial.println(i);
		stepper.moveTo(i);
		while (!stepper.run()){};
    }

	Serial.println("\n End loop 2---------------------------------------\n");	


    for (i = 200; i > 0; i--)
    {
		stepper.moveTo(i);
		Serial.print("\n pos:");		Serial.println(i);
    }

	digitalWrite(Pin_Rele1,0);
	Serial.println("\n End loop 2---------------------------------------\n");	
	stepper.disableOutputs();
	delay(5000);

	/*
	unsigned int uS = sonar.ping_median(5); //Send 5 pings, receive median ping time in microseconds
	Serial.print("Ping: ");
	Serial.print(uS / US_ROUNDTRIP_CM); //Convert ping time to dist & print result
	Serial.println("cm");

	*/
}

////////////////////////////////////////////////////////////////////////////////////

