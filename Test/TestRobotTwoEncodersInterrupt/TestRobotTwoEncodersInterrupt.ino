#define DEBUG_ENABLED 1
#include <../dbg/Dbg.h>
#include <hw_config.h>
#include <FlexiTimer2\FlexiTimer2.h>
#include <robot.h>
//#include <dbg/dbg.h>
//#include <../robot/RobotInterfaceCommands.h>

/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#if OPT_ENCODERS
	#include <Encoder.h>

	// Change these pin numbers to the pins connected to your encoder.
	//   Best Performance: both pins have interrupt capability
	//   Good Performance: only the first pin has interrupt capability
	//   Low Performance:  neither pin has interrupt capability

	Encoder knobLeft(Pin_EncLa, Pin_EncLb);
	Encoder knobRight(Pin_EncRa, Pin_EncRb);
	//   avoid using pins with LEDs attached
#endif

robot_c robot;
int d;
void setup() {
	Serial.begin(9600);
	Serial.println("__FILE__");
	Serial.println("test libreria Encoders con interrrupt in 5sec:");
	
	//DBG_ADDVAR("d",d)
	//DBGARRAY[DBGCOUNTER] = newChild("d",d); DBGCOUNTER++;
//	DBG_ADDPIN("13",13)
	  //DBG_VERSION;     // output the version
	  //DBG_ADDPIN("irproxy.bk",Pin_irproxy_BK)   // add a variable
	  //DBG_ADDPIN(F("irproxy.fw"), Pin_irproxy_FW)              // add another variable
	  //DBG_ADDPIN(F("Pin_EncRa"), Pin_EncRa)                             // add a pin
	  //DBG_ADDPIN(F("Pin_EncLa"), Pin_EncLa)                             // add a pin
	  //DBG_DUMP;    // debug all currently added variables            // output all debug variables

	delay(5000);
	Serial.print("run IBIT...");
	robot.runIBIT(1000);
	Serial.println("...end");
}

long positionLeft  = -999;
long positionRight = -999;

int cm=30;
void loop() {
	dbg("cmd robot.moveCm:", cm);
	d=robot.moveCm(cm);

	dbg("cm done:", d);
	delay(1000);
	cm *=-1;
/*	
	long newLeft, newRight;
	newLeft = knobLeft.read();
	newRight = knobRight.read();
	if (newLeft != positionLeft || newRight != positionRight) {
		Serial.print("encL:");    Serial.print(newLeft);
		Serial.print(", encR = ");    Serial.println(newRight);

		positionLeft = newLeft;
		positionRight = newRight;
	}
	// if a character is sent from the serial monitor,
	// reset both back to zero.
	if (Serial.available()) {
		Serial.read();
		Serial.println("Reset both knobs to zero");
		knobLeft.write(0);
		knobRight.write(0);
	}
	
	*/
}
