#include <SCoop\SCoop.h> 
#include <TimerUp\TimerUp.h>
 
// ========================================================================
// ===        															===
// ===       LIBRERIE JOISTICK USB										===
// ===																	===
// ========================================================================
 
#include "hw_config.h"
// ========================================================================
// ===        															===
// ===       LIBRERIE JOISTICK USB										===
// ===																	===
// ========================================================================
#pragma region joistick_libs 
	#include <hid.h>
	#include <hiduniversal.h>
	#include <usbhub.h>
	#include "hidjoystickrptparser.h"
#pragma endregion



 

String s;

//void printMyStack() {
//	Serial.println(SCoopTaskPtr->stackLeft());
//	SCoopTaskPtr->sleep(100); 
//}


//#################################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@												@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@			DEFINIZIONE DEI TASK				@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@												@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#################################################################################
#pragma region Thread_USB
/*========================================================================
 ===																	===
 ===       TASK:  GESTIONE EVENTI USB									===
 ===																	===
 ========================================================================*/
defineTask(task1);
//struct task1 : SCoopTask< task1,500,500 > {	// default stack (150bytes) and quantum time (400us)
void task1::setup() { }
void task1::loop() { 
// ================================================================
// ===               JOISTICK	SU USBHOST		                ===
// ================================================================
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

	//-----------------------------------------
	// Setup USB HOST
	//-----------------------------------------
	  #pragma region setup_usbhost		// inizializza USB Host

		if (Usb.Init() == -1)
			Serial.println("OSC did not start.");

		delay(200);

		if (!Hid.SetReportParser(0, &Joy))
				ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1); 
		Serial.println("USB Host init.");

	#pragma endregion

while(1){
	Usb.Task();
	//Serial.println(task1.stackLeft());
	//printMyStack();
	yield();
	} 
}
//} task1;

#pragma endregion


#pragma region Thread_Blink
// ========================================================================
// ===																	===
// ===       TASK: LED INTERMITTENTE											===
// ===																	===
// ========================================================================
// Task no.2: blink LED with 0.1 second delay.
//struct task2 : SCoopTask< task2, 100 > {	// allocate 100bytes for stack
defineTask(task2);
void  task2::setup(){
	}
void task2::loop()
	{
		// user code here
		digitalWrite(ONBOARD_LED_PIN, 1);
		sleepSync(1000);   //chThdSleepMilliseconds(1000);		//delay(100);
		digitalWrite(ONBOARD_LED_PIN, 0);
		sleepSync(500); // chThdSleepMilliseconds(1000);		//delay(100);
  

	}  
 
//}task2;
#pragma endregion

#pragma region Thread_Serial
// ========================================================================
// ===																	===
// ===       TASK: SERIALE												===
// ===																	===
// ========================================================================
//struct task3: SCoopTask< task3, 100, 150 > {// allocate 100bytes for stack and 150us
defineTask(task3);
void task3::setup() {
//  Serial.begin(9600);
  char str[20];
}
void task3::loop() { 						// example without setup()

	  // user setup here below. tasks will be automatically setup() by next call to yield.
  	//-----------------------------------------
	// Setup SERIALE
	//-----------------------------------------	
	Serial.begin(9600);
	Serial.println("Starting stepper test");

	// user code here
	if (!Serial.available()) yield();
    else
    {
		Serial.print(">");
		yield();
     }
  
} 
 
//}task3;
#pragma endregion




void setup()
{  


 


}

void loop() {
  
  yield();  // orchestrate everything.
  
//  if (timer>=1000) { timer=0;
//    // this code is ran every second
//  } 
}

