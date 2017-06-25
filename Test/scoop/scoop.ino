/*
#pragma region joistick_libs 
// ========================================================================
// ===        															===
// ===       LIBRERIE JOISTICK USB										===
// ===																	===
// ========================================================================
	#include <hid.h>
	#include <hiduniversal.h>
	#include <usbhub.h>
	#include "hidjoystickrptparser.h"
#pragma endregion
	*/
//example 4
// analogSampling 1khz **** ONLY FOR AVR ***** 
// as it uses timer0 capture overflow isr.
// ARM version with PIT will be devlop later :)

// ========================================================================
// ===        															===
// ===       RTOS LIBRARIES		:										===
// ===																	===
// ========================================================================
#include <TimerUp.h>
#include <SCoop.h>
#ifdef SCoop_ARM
#error "not for ARM as it use AVR ISR"
#endif



// ========================================================================
// ===        															===
// ===       CONFIGURAZIONE   HW:										===
// ===																	===
// ========================================================================
#include <TimerThree.h>

#pragma region GlobalVariables
// ========================================================================
// ===        															===
// ===       GLOBAL VARIABLES										===
// ===																	===
// ========================================================================
String s;
int sensorValue = 0;  // variable to store the value coming from the sensor
//---------------------------------------------------------------------------------


	// ================================================================
	// ===               ISTANZA OGGETTO ROBOT		                ===
	// ================================================================
	#include <robot.h>
	#include <hw_config.h>
	extern struct robot_c robot;


	/*
	// ================================================================
	// ===               OGGETTO USB				                ===
	// ================================================================
	USB Usb;
	USBHub Hub(&Usb);
	HIDUniversal Hid(&Usb);
	JoystickEvents JoyEvents;
	JoystickReportParser Joy(&JoyEvents);
	*/

	//-----------------------------------------

#pragma endregion 


TimerUp T500ms(500);           // rollover every 500ms
defineFifo(fifo1,int16_t,200); // able to store 200ms of samples for Analog1 if needed (like if writing SD card)
defineFifo(fifo2,int16_t,20)   // 400ms max (20x20) for Analog2

vui32 avgAna2 = 0;
float scaleAna2=0.0;           // the real user value computed in an event

defineEventRun(event20ms)      // this event is trigger by the timer0 overflow interupt isr below
{ while (fifo2) { 
   int16_t val; 
   fifo2.get(&val); 
   avgAna2 += val - (avgAna2 >>2); } // overage mean of the 4 last value
 scaleAna2 = (avgAna2 / 16.0 * 1.75 + 0.25)/4.0;
}



// ================================================================
// ===               ISR						                ===
// ================================================================
vui32 count=0;

ISR(TIMER0_COMPA_vect) {       // same rate as Timer0: every 1024us (16mhz)
  fifo1.putInt(analogRead(1));
  count++; 
  if (count >= 20) { count =0; // every 20ms
     fifo2.putInt(analogRead(2)); 
     event20ms=true; } // trigger the event for further calculation
}

void isrMotorClock()		//Chiamato da interrupt
{
   digitalWrite(Pin_MotCK, !digitalRead(Pin_MotCK));   // set the LED on
}

//----------------------------------------------------------------------------------------
//vi32 avgAna1;
//
//defineTask(task3) // treat ana 1 : average mean over 16 last samples and print value every 500ms
//
//void task3::setup() { avgAna1=0; T500ms=0; }
//void task3::loop()  { 
//  if (fifo1) { int16_t val; fifo1.get(&val); avgAna1 += val - (avgAna1 >> 4); }
//  if (T500ms.rollOver()) {
//     SCp("avg ana1 = ");SCp(avgAna1 >> 4);  
//     SCp(", scaleAna2 = ");SCp(scaleAna2); 
//#if SCoopTIMEREPORT > 0
//     SCp(", cycle time = ");SCp((mySCoop.cycleMicros >> SCoopTIMEREPORT));
//	 SCp(", max = ");SCp((mySCoop.maxCycleMicros >> SCoopTIMEREPORT)); mySCoop.maxCycleMicros = 0;
//#endif	 
//	 SCp(", stackleft = ");SCpln(stackLeft());
//   }   
//}

#pragma region task_USB
/*========================================================================
 ===																	===
 ===       TASK:  GESTIONE EVENTI USB									===
 ===																	===
 ========================================================================*/
defineTask(taskUSB,400) //
void taskUSB::setup() { 
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
}



void taskUSB::loop()  { 



	while(1){
		//Usb.Task();
		//SCp(", stackleft taskUSB= ");SCpln(stackLeft());
		//printMyStack();
		yield();
	}

}

#pragma endregion	//task_USB



#pragma region task_led
// ========================================================================
// ===																	===
// ===       TASK: LED INTERMITTENTE									===
// ===																	===
// ========================================================================
#define LED_PIN Pin_ONBOARD_LED
//defineTimer(timerLED,2000)
//	void timerLED::run() { 
//		digitalWrite(LED_PIN, !digitalRead(LED_PIN));}

defineTask(taskLED)
void taskLED::setup() {   pinMode(LED_PIN, OUTPUT); }
void taskLED::loop()  { 

   
    digitalWrite(LED_PIN, HIGH);	// Turn LED on.
    sleep(50);	// yield(); //	 Sleep for 50 milliseconds.
    digitalWrite(LED_PIN, LOW);		// Turn LED off.
	sleepSync(950);		//	yield();   // Sleep for 150 milliseconds.

}
#pragma endregion	//task_led


#pragma region task_Sensors
// ========================================================================
// ===																	===
// ===       TASK: ACQUISIZIONE SENSORI									===
// ===																	===
// ========================================================================
defineTask(task_Sensors)
	void task_Sensors::setup() {   }
	void task_Sensors::loop()  { 
		extern robot_c robot; // ISTANZIA L'OGGETTO ROBOT E LO INIZIALIZZA
		robot.readSensors();
		if (robot.irproxy.fw==1  ) {
			Serial.println("forward obstacle");	
			if( robot.currCommand.command==GOFW){
					robot.stop();
			}
			
		}
		sleep(100);		//	yield();   // Sleep for n milliseconds.
}
#pragma endregion	//task_led



void setup() { 

  SCbegin(9600); 
  	//-----------------------------------------
	// Setup Robot
	//-----------------------------------------
	extern robot_c robot; // ISTANZIA L'OGGETTO ROBOT E LO INIZIALIZZA
	robot.irproxy.fw = digitalRead(Pin_irproxy_FW);
	pinMode(Pin_irproxy_FW, INPUT);
	Serial.println("Robot initialised.");
	Timer3.attachInterrupt(isrMotorClock);  // attaches callback() as a timer overflow interrupt

  mySCoop.start(1000); // start the scheduler with a goal of Nus per whole cycle. adjust quantum accordingly
                      // this "garantee" that the event (or timers) will be treated within this time window
  TIMSK0  |= (1 << OCIE0A);  // enable TIMER0 COMPA Interupt
  
  while (1) yield();  // by this way we are in total control of what the program does
  }
void loop() { 
  }

