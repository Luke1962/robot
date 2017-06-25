// ============================================================================================
// ===																						===
// ===       LIBRERIE																		===
// ===		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		===
// ===		Configuration Properties >C++ > Path											===
// ============================================================================================
#include <TimerThree.h>
#include  <ChibiOS_AVR.h>
#include <util/atomic.h>


// You must call chYield() or other ChibiOS functions such as chThdSleep to force a context switch to other threads.
//
// Insert a delay(100) in the mainThread loop to see the effect of not
// using chThdSleep with cooperative scheduling.
//
// Setting CH_TIME_QUANTUM to zero disables the preemption for threads
// with equal priority and the round robin becomes cooperative.
// Note that higher priority threads can still preempt, the kernel
// is always preemptive.
#define CH_TIME_QUANTUM 1

    // to allow other threads to run for 1 sec use		>>>		chThdSleepMilliseconds(1000);
    // to yield so other threads can run, use			>>>		chThdYield();
	// precedere ogni thread con						>>>		static WORKING_AREA(wa[NOME THREAD], [stacksize=64] );
	// Semafori: declare and initialize					>>>		SEMAPHORE_DECL([semNomeSemaforo], [Max Accessi]);		es SEMAPHORE_DECL(semSerial,1);	
			// wait for semaforo						>>>		chSemWait(&[semNomeSemaforo]);
		    // exit region								>>>		chSemSignal(&[semNomeSemaforo]);
	// per bloccare /sbloccare gli interrupt			>>>		chSysLock() /* Protected code */  chSysUnlock();
// http://www.chibios.org/dokuwiki/doku.php?id=chibios:guides:mutual_exclusion_guide
// declare and initialize a mutex for limiting access to threads
//static MUTEX_DECL(MutexSerial);



// ================================================================
// ===					#DEFINES				                ===
// ================================================================
#define SERIAL_BAUD_RATE 115200
//#define delay chThdSleepMilliseconds



// ========================================================================
// ===        															===
// ===       CONFIGURAZIONE   HW:										===
// ===																	===
// ========================================================================
#include "hw_config.h"




// ========================================================================
// ===        															===
// ===       GLOBAL VARIABLES										===
// ===																	===
// ========================================================================
// ================================================================
// ===               ISTANZA OGGETTO ROBOT		                ===
// ================================================================
#include "robot.h"
extern struct robot_c robot;

String s;
int sensorValue = 0;  // variable to store the value coming from the sensor
//---------------------------------------------------------------------------------


// ========================================================================
// ===        															===
// ===       LIBRERIE JOISTICK USB										===
// ===																	===
// ========================================================================
#pragma region joistick_libs 
#define DEBUG_USB_HOST 
	#include <hid.h>
	#include <hiduniversal.h>
	#include <usbhub.h>
	#include "hidjoystickrptparser.h"

	//================================================================
	//===               JOISTICK	SU USBHOST		                ===
	//================================================================
	USB Usb;
	USBHub Hub(&Usb);
	HIDUniversal Hid(&Usb);
	JoystickEvents JoyEvents;
	JoystickReportParser Joy(&JoyEvents);

#pragma endregion










//#################################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@												@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@			INTERRUPT SERVICE ROUTINES			@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@												@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#################################################################################
#pragma region INTERRUPT_SERVICE_ROUTINES
//noInterrupts ();  // or ...cli ();           // clear interrupts flag
//
//interrupts ();  // or ...sei ();         // set interrupts flag
#pragma endregion

//#################################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@												@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@			DEFINIZIONE DEI TASK				@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@												@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#################################################################################
#pragma region task_definition

#pragma region task_USB
	/*========================================================================
 ===																	===
 ===       TASK:  GESTIONE EVENTI USB									===
 ===																	===
 ========================================================================*/
// data structures and stack for thread 
static WORKING_AREA(waThread_USB, 512 );

msg_t Thread_USB(void* arg){

  while (1) {
	  chSysDisable();
       Usb.Task();
	  chSysEnable();
	  chThdSleep(10);// chThdYield(); //OPPURE ? chThdSleepMilliseconds(100);
  }
}
#pragma endregion	//task_USB
#pragma region task_led
// ========================================================================
// ===																	===
// ===       TASK: LED INTERMITTENTE									===
// ===																	===
// ========================================================================
static WORKING_AREA(waThread_Blink, 64);
#define LED_PIN Pin_ONBOARD_LED
static msg_t Thread_Blink(void *arg) {
  pinMode(LED_PIN, OUTPUT);
  
  // Flash led every 200 ms.
  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);

	// Sleep for 50 milliseconds.
    chThdSleepMilliseconds(500);
    
    // Turn LED off.
    digitalWrite(LED_PIN, LOW);
    
    // Sleep for 150 milliseconds.
    chThdSleepMilliseconds(950);
  }
  return 0;
}
#pragma endregion	//task_led

#pragma region task_SERIAL
// ========================================================================
// ===																	===
// ===       TASK: INVIO SU SERIALE										===
// ===																	===
// ========================================================================
static WORKING_AREA(waThreadSer, 640);

static msg_t ThreadSer(void *arg) {
  // Flash led every 200 ms.
  while (1) {
    Serial.print(".");
    chThdSleepMilliseconds(2000);
    }
  return 0;
}
#pragma endregion	//task_SERIAL


#pragma endregion
//#################################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	FINE task	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#################################################################################





//------------------------------------------------------------------------------
// main is thread 1 and runs at NORMALPRIO
void mainThread() {

 chThdCreateStatic(waThread_Blink, sizeof(waThread_Blink), NORMALPRIO+50, Thread_Blink, (void*)"Thread_Blink");
 chThdCreateStatic(waThreadSer, sizeof(waThreadSer), NORMALPRIO+10, ThreadSer, NULL);
 chThdCreateStatic(waThread_USB, sizeof(waThread_USB), NORMALPRIO+ 100, Thread_USB, (void*)"Thread_USB");

}

//------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  cli();
  halInit();
  chSysInit();

  Serial.begin(9600);
  
  //while (!Serial) {} // wait for USB Serial
  
  chBegin(mainThread);// initialize heap/stack memory and start ChibiOS

}
//------------------------------------------------------------------------------
void loop() {
    chThdSleep(10);
}