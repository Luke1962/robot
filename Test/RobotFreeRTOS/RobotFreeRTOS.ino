// ============================================================================================
// ===																						===
// ===       LIBRERIE																		===
// ===		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		===
// ===		Configuration Properties >C++ > Path											===
// ============================================================================================
#include  <FreeRTOS_AVR.h>
#include <TimerThree.h>

// ========================================================================
// ===        															===
// ===       CONFIGURAZIONE   HW:										===
// ===																	===
// ========================================================================
#include <hw_config.h>
#define SERIAL_BAUD_RATE 115200	// ===               SETUP SERIALE	



#pragma region joistick_libs 
	#include <hid.h>
	#include <hiduniversal.h>
	#include <usbhub.h>
	#include "hidjoystickrptparser.h"
#pragma endregion


// ================================================================
// ===               variabili globali E ROBOT	                ===
// ================================================================
#include  <robot.h>
extern struct robot_c robot;

String s;
int sensorValue = 0;  // variable to store the value coming from the sensor
//---------------------------------------------------------------------------------







void isrMotorClock()		//Chiamato da interrupt
{
   digitalWrite(Pin_MotCK, !digitalRead(Pin_MotCK));   // set the LED on
}

/*========================================================================
 ===																	===
 ===       TASK:  GESTIONE EVENTI USB									===
 ===																	===
 ========================================================================*/
static void ThreadUSB(void* arg) {
	// ================================================================
	// ===               JOISTICK	SU USBHOST		                ===
	// ================================================================
	taskENTER_CRITICAL();
	USB Usb;
	USBHub Hub(&Usb);
	HIDUniversal Hid(&Usb);
	JoystickEvents JoyEvents;
	JoystickReportParser Joy(&JoyEvents);
	 	#pragma region setup_usbhost		// inizializza USB Host
  		//-----------------------------------------
		// Setup USB
		//-----------------------------------------	

		if (Usb.Init() == -1)		Serial.println("OSC did not start.");

		delay(200);

		if (!Hid.SetReportParser(0, &Joy))				ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1); 
		Serial.println("USB Host init.");

	#pragma endregion 
  while (1) { 
	Usb.Task();
	taskEXIT_CRITICAL();
	yield();
  }
//			delayMicroseconds(10000000);
}

// Declare a semaphore handle.
//SemaphoreHandle_t sem;
//------------------------------------------------------------------------------
/*
 * Thread 1, turn the LED off when signalled by thread 2.
 */
// Declare the thread function for thread 1.
//------------------------------------------------------------------------------
static void ThreadSER(void* arg) {
 	taskENTER_CRITICAL();
 
  	//-----------------------------------------
	// Setup SERIALE
	//-----------------------------------------	
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println("Starting stepper test.");

  while (1) {
	// Sleep for 1000 milliseconds.
	//vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
	Serial.println("A");
	taskEXIT_CRITICAL();
	yield();
  }
}

// ========================================================================
// ===																	===
// ===       TASK: LED INTERMITTENTE									===
// ===																	===
// ========================================================================
// Task no.2: blink LED with 0.1 second delay.
static void ThreadLED(void* arg) {

  while (1) {
	// Turn LED on.
	digitalWrite(Pin_ONBOARD_LED, HIGH);

	// Sleep for 1000 milliseconds.
	vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);

	// Turn LED on.
	digitalWrite(Pin_ONBOARD_LED, LOW);

	// Sleep for 500 milliseconds.
	vTaskDelay((700L * configTICK_RATE_HZ) / 1000L);

		  yield();
  }

}





#pragma region setup
// ========================================================================
// ===																	===
// ===       SETUP														===
// ===																	===
// ========================================================================

void setup() {
  portBASE_TYPE s1, s2;	// uno per ogni task
	taskENTER_CRITICAL();




	////-----------------------------------------
	//// Setup Robot
	////-----------------------------------------
	//extern robot_c robot; // ISTANZIA L'OGGETTO ROBOT E LO INIZIALIZZA
	//pinMode(Pin_irproxy_FW, INPUT);
	//Serial.println("Robot initialised.");
	//Timer3.attachInterrupt(isrMotorClock);  // attaches callback() as a timer overflow interrupt

	////-----------------------------------------
 // 
	//taskEXIT_CRITICAL();


#pragma region setup_multitasking   // crea i vari task e li fa partire	
  // initialize fifoData semaphore to no data available
  //sem = xSemaphoreCreateCounting(1, 0);


  // create  task (Nome  , NULL, ,Stack, NULL, Priority, NULL)
  s2 = xTaskCreate(ThreadLED, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  s1 = xTaskCreate(ThreadSER, NULL, 500, NULL, 1, NULL);


  // check for creation errors
 // if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
	//Serial.println(F("Creation problem"));
	//while(1);
 // }
  // start scheduler
  vTaskStartScheduler();
  Serial.println(F("Insufficient RAM"));
  while(1);
}
#pragma endregion
#pragma endregion //setup()
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
  // Not used.
}





 


















 





 




