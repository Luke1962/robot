// test di rotazione per lo stepper dedicato al sonar
// attende che venga premuto lo switch di home
// NON va la lettura float da seriale
//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
//#define delay(ms) chThdSleepMilliseconds(ms) 
#include "math.h"
#include <MyRobotLibs\dbg.h>
#include <MyRobotLibs\systemConfig.h>
#include <MyRobotLibs\hw_config.h>
#include <PinChangeInt\PinChangeInt.h>	//https://github.com/NicoHood/PinChangeInterrupt


#pragma endregion

#include <digitalWriteFast.h>
#include <MyRobotLibs/robot.h>
//struct robot_c robot;	//was  struct robot_c robot;

#define sgn(x) ((x > 0) - (x < 0))
#define SERIAL_DBG	Serial
#define SERIAL_DBG_BAUD_RATE 115200

#include <Timer5/Timer5.h>

//#include <NewTone\NewTone.h>

#include <MyStepper\myStepper.h>
myStepper_c myLDSstepper(PIN_STEPPERLDS_CK,PIN_STEPPERLDS_ENABLE,PIN_STEPPERLDS_CW, PIN_STEPPERLDS_HOME);
String inString = "";    // string to hold input
float speed = PI;
#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1


void ISRstepperSwitchHome() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > MINIMUM_INTERRUPT_INTERVAL_MSEC)
	{
		int x  = digitalReadFast(PIN_STEPPERLDS_HOME);
		if (x ==0)
		{
			myLDSstepper.setHomePosition(true);
			LEDTOP_B_ON;
			 ///.disable();
			myLDSstepper.goRadsPerSecond(speed); //myLDSstepper.setCW(true);  ///.disable();

		}
		else
		{
			myLDSstepper.setHomePosition(false);
			LEDTOP_B_OFF;
			//myLDSstepper.enable();
				
		}

	}
	last_interrupt_time = interrupt_time;
}

void ISRstepperSwitchEnd() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > MINIMUM_INTERRUPT_INTERVAL_MSEC)
	{
		int x  = digitalReadFast(PIN_STEPPERLDS_END);
		if (x ==0)
		{
			//myLDSstepper.setHomePosition(true);
			LEDTOP_G_ON;
			myLDSstepper.goRadsPerSecond(-3*speed);//myLDSstepper.setCW(false);  ///.disable();

		}
		else
		{
			//myLDSstepper.setHomePosition(false);
			LEDTOP_G_OFF;
			//myLDSstepper.enable();
				
		}

	}
	last_interrupt_time = interrupt_time;
}

 
 byte ckState = 0;
ISR(timer5Event) //TIMER STEPPER
{
	// Reset Timer1 (resetTimer1 should be the first operation for better timer precision)
	resetTimer5();
	// For a smaller and faster code, the line above could safely be replaced with a call
	// to the function resetTimer1Unsafe() as, despite its name, it IS safe to call
	// that function in here (interrupts are disabled)

	// Make sure to do your work as fast as possible, since interrupts are automatically
	// disabled when this event happens (refer to interrupts() and noInterrupts() for
	// more information on that)

	// Toggle led's state
	ckState ^= 1;
	digitalWriteFast(13, ckState);

	digitalWriteFast(PIN_STEPPERLDS_CK, ckState);
}
void setup() {
	Serial3.begin(115200);
	Serial.begin(115200);
	dbg("\nTest LDS Stepper -------")

	pinMode(PIN_STEPPERLDS_HOME, INPUT_PULLUP);// open >+5 closed =gnd
	pinMode(PIN_STEPPERLDS_END, INPUT_PULLUP);// open >+5 closed =gnd
	SERIAL_DBG.begin(SERIAL_DBG_BAUD_RATE);
	attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome,CHANGE );  // add more attachInterrupt code as required
	attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd,CHANGE );  // add more attachInterrupt code as required
	myLDSstepper.goRadsPerSecond(0.0);



	dbg("click LDS home switch to start..")
	while (!myLDSstepper.isHomePosition()) { delay(200); }
/*
	// test home switch-------------------
	dbg("click LDS home switch..")
	dbg("BLUE LED should switch on..")
	while (true)
	{
		if (myLDSstepper.isHomePosition())
		{
			dbg("Home")
			
		}
		
		//Serial.print(digitalRead(PIN_STEPPERLDS_HOME));
		delay(200);

	}

	delay(1000);

	// test freq out
	NewTone(PIN_STEPPERLDS_CK, 30, 1800);
	//toneAC(125);
	//tone(PIN_STEPPERLDS_CK, 100);//funziona  (usa il timer2)


	delay(5000);

	// no freq out
 
	NewTone( PIN_STEPPERLDS_CK,20, 1000);


	noTone(PIN_STEPPERLDS_CK);

	myLDSstepper.goRadsPerSecond(1.0);
	delay(5000);
	myLDSstepper.goRadsPerSecond(0.5);
	delay(2000);
	myLDSstepper.goRadsPerSecond(0.0);
*/


	 

	//myLDSstepper.goHome(-5);
	//while (!digitalReadFast(PIN_STEPPERLDS_stop)){}
	//myLDSstepper.goRadsPerSecond(0);

	//speed *= -1;


	myLDSstepper.goRadsPerSecond(speed);


}
void loop() {
	//input velocità
	while (Serial.available() > 0) {
		int inChar = Serial.read();

		if (inChar != '\n') {
			// As long as the incoming byte
			// is not a newline,
			// convert the incoming byte to a char
			// and add it to the string
			inString += (char)inChar;
		}
		// if you get a newline, print the string,
		// then the string's value as a float:
		else {
			speed = inString.toFloat();
			Serial.print("Input string: ");
			Serial.print(inString);
			Serial.print("\tAfter conversion to float:");
			Serial.println(speed);
			// clear the string for new input:
			inString = "";
			myLDSstepper.goRadsPerSecond(speed);
		}
	}
/*
//	speed += sgn(speed)*0.1;
	Serial.print("Speed: ");Serial.println(speed);
	
	myLDSstepper.goRadsPerSecond(speed);
	for (size_t i = 0; i < 100; i++)
	{
		myLDSstepper.step(10);
		Serial.println(i);
	}
	delay(1000);
	Serial.println("Stop");
	myLDSstepper.goRadsPerSecond(0);
	delay(1000);
	Serial.println("Inversione \n\n");

*/


	// sostituto di goHome
	while (!myLDSstepper.isHomePosition()) { delay(200); }



	//lettura LDS dummy----------------------------
	for (size_t i = 0; i < 10; i++)
	{
		dbg2("i :", i);
		delay(500);
		
	}
	// attendo l'inversione del moto da parte dell'interrupt di finecorsa
 

 }

