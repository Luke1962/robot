// test di rotazione per lo stepper dedicato al sonar
// attende che venga premuto lo switch di home
// NON va la lettura float da seriale
//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
//#define delay(ms) chThdSleepMilliseconds(ms) 
#include "math.h"
#include <MyRobotLibs\dbgCore.h>
#include <MyRobotLibs\systemConfig.h>
#include <MyRobotLibs\hw_config.h>
#include <digitalWriteFast.h>





#pragma endregion









#if OPT_LDS
	#include <Wire.h>
	#include <VL53L0X\VL53L0X.h>
	VL53L0X LDS;
	#define LONG_RANGE
	// Uncomment ONE of these two lines to get
	// - higher speed at the cost of lower accuracy OR
	// - higher accuracy at the cost of lower speed

	//#define HIGH_SPEED
	//#define HIGH_ACCURACY


	bool setup_LDS() {
		Wire.begin();

		byte failCount = 0;
		bool initDone = false;
		while (!initDone && (failCount < 10))
		{
			MSG("LDS init...");
			if (LDS.init())
			{
				MSG("LDS OK;");
				LDS.setTimeout(500);
				initDone = true;
				return initDone;
			}
			else
			{
				MSG("LDS   ..FAIL;");
				delay(500);
				failCount++;
			}

		}

		#if defined LONG_RANGE
				MSG("LDS Long range");
				// lower the return signal rate limit (default is 0.25 MCPS)
				LDS.setSignalRateLimit(0.1);
				// increase laser pulse periods (defaults are 14 and 10 PCLKs)
				LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
				LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
		#endif

		#if defined HIGH_SPEED
				// reduce timing budget to 20 ms (default is about 33 ms)
				distanceSensor.setMeasurementTimingBudget(20000);
		#elif defined HIGH_ACCURACY
				MSG("LDS High Accuracy");
				// increase timing budget to 200 ms
				distanceSensor.setMeasurementTimingBudget(200000);
		#endif













		LDS.startContinuous(100);
		return initDone;
	}

#endif // OPT_LDS



#if OPT_STEPPERLDS
	#include <Timer5/Timer5.h>
	#include <PinChangeInt\PinChangeInt.h>	//https://github.com/NicoHood/PinChangeInterrupt
	#include <MyStepper\myStepper.h>
	myStepper_c myLDSstepper( PIN_STEPPERLDS_CK,	PIN_STEPPERLDS_ENABLE,	PIN_STEPPERLDS_CW,		PIN_STEPPERLDS_HOME,		PIN_STEPPERLDS_END);

	byte ckState = 0;


	#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1

	// Genera il clock per LDS Stepper
	ISR(timer5Event)
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
		//digitalWriteFast(13, ckState);

		digitalWriteFast(PIN_STEPPERLDS_CK, ckState);
	}


	void ISRstepperSwitchHome() {
		static unsigned long last_interrupt_timeHome = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{
			myLDSstepper.setHomePosition(true);
			myLDSstepper.setCW(false);  ///.disable();
		}
		last_interrupt_timeHome = interrupt_time;
	}

	void ISRstepperSwitchEnd() {
		static unsigned long last_interrupt_timeEnd = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{

			myLDSstepper.setEndPosition(true);
			myLDSstepper.setCW(true);  ///.disable();
		}
		last_interrupt_timeEnd = interrupt_time;
	}


	// versione pe Interrupt su Change
	void ISRldsHome() {
		static unsigned long last_interrupt_timeHome = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{
			int x = digitalReadFast(PIN_STEPPERLDS_HOME);
			if (x == 0)
			{
				myLDSstepper.setHomePosition(true);
				myLDSstepper.setCW(true);  ///.disable();

			}
			else
			{
				myLDSstepper.setHomePosition(false);
				myLDSstepper.enable();

			}

		}
		last_interrupt_timeHome = interrupt_time;
	}
	void ISRldsEnd() {
		static unsigned long last_interrupt_timeEnd = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{
			int x = digitalReadFast(PIN_STEPPERLDS_END);
			if (x == 0)
			{
				//myLDSstepper.setHomePosition(true);
				myLDSstepper.setCW(false);  ///.disable();

			}
			else
			{
				//myLDSstepper.setHomePosition(false);
				myLDSstepper.enable();

			}

		}
		last_interrupt_timeEnd = interrupt_time;
	}


	void setup_StepperLDS(float speed = 2 * PI) {
		pinMode(PIN_STEPPERLDS_HOME, INPUT_PULLUP);// open => +5v ; closed =>gnd
		pinMode(PIN_STEPPERLDS_END, INPUT_PULLUP);// open >+5 closed =gnd
												  //attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, CHANGE);  // add more attachInterrupt code as required
												  //attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, CHANGE);  // add more attachInterrupt code as required

		attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, FALLING);  // add more attachInterrupt code as required
		attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, FALLING);  // add more attachInterrupt code as required
																					 // start stepper
		myLDSstepper.goRadsPerSecond(speed);
	}

#endif












 
#define sgn(x) ((x > 0) - (x < 0))
#define SERIAL_DBG	Serial
#define SERIAL_DBG_BAUD_RATE 115200

 
 
 String inString = "";    // string to hold input
float speed = PI;
#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1


 
 
 //byte ckState = 0;
//ISR(timer5Event) //TIMER STEPPER
//{
//	// Reset Timer1 (resetTimer1 should be the first operation for better timer precision)
//	resetTimer5();
//	// For a smaller and faster code, the line above could safely be replaced with a call
//	// to the function resetTimer1Unsafe() as, despite its name, it IS safe to call
//	// that function in here (interrupts are disabled)
//
//	// Make sure to do your work as fast as possible, since interrupts are automatically
//	// disabled when this event happens (refer to interrupts() and noInterrupts() for
//	// more information on that)
//
//	// Toggle led's state
//	ckState ^= 1;
//	digitalWriteFast(13, ckState);
//
//	digitalWriteFast(PIN_STEPPERLDS_CK, ckState);
//}
void setup() {
	initRobotBaseHW();
	LEDTOP_R_ON
//	SERIAL_DBG.begin(SERIAL_DBG_BAUD_RATE);

 

	setup_StepperLDS();  // lo fa partire con la veloccità di default
	setup_LDS();

	MSG("s.END--")

	LEDTOP_R_OFF

//	MSG("click LDS home switch to start..")
//	while (!myLDSstepper.isHomePosition()) { delay(200); }
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


	 

 


}	

uint16_t range_mm = 0;
int i = 0;
bool acquiring = false;
void loop() {
	/*
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
	*/
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
	//while (!myLDSstepper.isHomePosition()) { delay(200); }

	//lettura LDS ----------------------------

		if (myLDSstepper.isEndPosition())
		{
			LASER_ON;
			acquiring = true;
			i = 0;
			//for (size_t i = 0; i < 10; i++)
			while (acquiring)
			{
				MSG2("i :", i++);
				range_mm = LDS.readRangeSingleMillimeters();
				MSG2("mm :", range_mm);
				//delay(100);
				TOGGLEPIN(Pin_LED_TOP_B);
				delay(100);
				if (myLDSstepper.isHomePosition())
				{
					acquiring = false;
					LASER_OFF;
					MSG("-----------");
					delay(100);

				}
				
			}
 

		}



	// attendo l'inversione del moto da parte dell'interrupt di finecorsa
 

 }

