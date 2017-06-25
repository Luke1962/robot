// ============================================================================================
// ===																						===
// ===       LIBRERIE																		===
// ===		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		===
// ===		Configuration Properties >C++ > Path											===
// ============================================================================================
#include <TimerThree.h>


// ========================================================================
// ===        															===
// ===       CONFIGURAZIONE   HW:										===
// ===																	===
// ========================================================================
#include <hw_config.h>


#pragma region joistick_libs 
	#include <hid.h>
	#include <hiduniversal.h>
	#include <usbhub.h>
	#include "hidjoystickrptparser.h"
#pragma endregion





String s;
int sensorValue = 0;  // variable to store the value coming from the sensor
//---------------------------------------------------------------------------------


// ================================================================
// ===               SETUP SERIALE				                ===
// ================================================================
#define SERIAL_BAUD_RATE 115200

// ================================================================
// ===               variabili globali ROBOT	                ===
// ================================================================
#include <robot.h>
extern struct robot_c robot;

//void MotorClock()		//Chiamato da interrupt
//{
//   digitalWrite(Pin_MotCK, !digitalRead(Pin_MotCK));   // set the LED on
//}

// ================================================================
// ===               JOISTICK	SU USBHOST		                ===
// ================================================================
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

// ========================================================================
// ===																	===
// ===       SETUP														===
// ===																	===
// ========================================================================

void setup() {
//  portBASE_TYPE s1, s2, s3;		//puntatori ai task

	//-----------------------------------------
	// Setup SERIALE
	//-----------------------------------------	
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println("Starting stepper test.");


	//-----------------------------------------
	// Setup Robot
	//-----------------------------------------
	extern robot_c robot; // ISTANZIA L'OGGETTO ROBOT E LO INIZIALIZZA
	pinMode(Pin_irproxy_FW, INPUT);
	Serial.println("Robot initialised.");
//	Timer3.attachInterrupt(MotorClock);  // attaches callback() as a timer overflow interrupt

	//-----------------------------------------

#pragma region setup_usbhost		// inizializza USB Host

    if (Usb.Init() == -1)
        Serial.println("OSC did not start.");

    delay(200);

    if (!Hid.SetReportParser(0, &Joy))
            ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1); 
	Serial.println("USB Host init.");

#pragma endregion



}


//------------------------------------------------------------------------------
void loop() {
    // Not used.
	       Usb.Task();

}




