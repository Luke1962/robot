///////////////////////////////////////////////////////////////////////////
#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	
// =======================================================================
// ===        															===
// ===       sCoop LIBRARIES		:									===
// ===																	===
// =======================================================================
 
#include <TimerUp.h>
#include <SCoop.h>
#ifdef SCoop_ARM
	#error "not for ARM as it use AVR ISR"
#endif
// =======================================================================

 // ============================================================================================
 // ===																						===
 // ===       LIBRERIE																		===
 // ===		Aggiungere ciascun percorso nelle propriet� del progetto in Visual Studio 		===
 // ===		Configuration Properties >C++ > Path											===
 // ============================================================================================
//#include <FrequencyTimer2\FrequencyTimer2.h>	
#include <FlexiTimer2\FlexiTimer2.h>
#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
#include <avr/wdt.h>
#include <robot/robot.h>

#if OPT_SERVOSONAR
	#include "arduino.h"

void software_Reboot();
bool hasExpired(unsigned long &prevTime, unsigned long interval);
void BlinkLed();
//
//
Servo servoSonar;
	NewPing Sonar( Pin_SonarTrig, Pin_SonarEcho );
#endif

struct robot_c robot;

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger/CmdMessenger.h>
CmdMessenger cmd = CmdMessenger( Serial );
#include <robot/RobotInterfaceCommands.h>

// special "catch all" ISR which catches all unhandled IRQs.
ISR( BADISR_vect )
{
	Serial.print( "1,BADISR_vect;" );
	for (;;) UDR0 = '!';
}
void software_Reboot()
{
	wdt_enable( WDTO_15MS );
	while (1){}
}


// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval) {
  if (  millis() - prevTime > interval ) {
    prevTime = millis();
    return true;
  } else     
    return false;
}
  
	// Toggle led state 
	void BlinkLed()
	{
		digitalWrite( Pin_ONBOARD_LED, 0 );
		sleep( 50 );	// yield(); //	 Sleep for 50 milliseconds.
		digitalWrite( Pin_ONBOARD_LED, 255 );
		sleep( 500 );	// yield(); //	 Sleep for 50 milliseconds.
	}







// Blinking led variables -------------------------------------------------------------------------------------------------------
unsigned long previousToggleLed = 0;   // Last time the led was toggled
bool ledState                   = 0;   // Current state of Led
const int kBlinkLed             = 13;  // Pin of internal Led
float ledFrequency             = 1.0; // Current blink frequency of Led
unsigned long intervalOn;
unsigned long intervalOff;
unsigned long prevBlinkTime = 0;

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

	defineTask( taskLED )
	void taskLED::setup() { pinMode( LED_PIN, OUTPUT ); Serial.print("Setup sendSensors");
	}
	void taskLED::loop()  { 
		BlinkLed(); 
		Serial.print( "L" ); 		
		mySCoop.yield();
	}
#pragma endregion	//task_led



#pragma region task_sendSensors
	// ========================================================================
	// ===       TASK:	INVIO STATO SENSORI									===
	// ========================================================================	

	defineTask(sendSensors,1800)
		void sendSensors::setup(){		Serial.print("Setup sendSensors");	}	
	void sendSensors::loop(){
		//OnCmdSonarScan();
		Serial.print("S");
		OnCmdGetSensorsHRate();
		mySCoop.yield( );
	}
#pragma endregion	//task_sendSensors



#pragma region task_CommandMessenger
// ===========================================================================
// ==																		==
// ==       TASK: GESTIONE COMANDI DA PC									==
// ==																		==
// ===========================================================================
#if 0
	defineTask(taskCommandMessenger,1000)	
	void taskCommandMessenger::setup(){

		// Setup CommandMessenger -----------------------------------------------------
		cmd.printLfCr();   // Adds newline to every command 
		attachCommandCallbacks();// Attach my application's user-defined callback methods
		//------------------------------------------------------------------------------

	};
	void taskCommandMessenger::loop(){
		// Process incoming serial data, and perform callbacks
		Serial.print("+");	//	 cmd.sendCmd(Status, "ok");
		cmd.feedinSerialData();
		mySCoop.yield();
  
		//if (hasExpired(previousToggleLed,2000)) // Toggle every 2 secs
		//{
		//	// Send the status to the PC that says the Arduino has booted
		//}
	}
#endif
#pragma endregion //task_CommandMessenger






void setup() {
	SCbegin(SERIAL_BAUD_RATE); 
	Serial1.begin(SERIAL_BAUD_RATE);
	Serial.println("Setup TestSCoopCmdMessenger");

	robot.SetMode(ROBOT_STARTING_MODE);

	Serial.print("1, -- Start IBIT.. ;");
//	robot.runIBIT(400, &servoSonar, &Sonar);
	Serial.print("1, -- End IBIT.. ;");


	Serial.print("1, Scanning...;");
	robot.SonarScanBatch(&servoSonar, &Sonar);

	kbSonarSendData();

		//-----------------------------------------
		// Setup Robot
		//-----------------------------------------
		#if  OPT_SERVOSONAR  
				NewPing Sonar( Pin_SonarTrig, Pin_SonarEcho, robot.parameter.sonarMaxDistance );
				// passo a robot gli indirizzi  di servoSonar e Sonar
				robot.initServoAndSonar( &servoSonar, &Sonar );
		#endif

	//	Timer3.attachInterrupt(isrMotorClock);  // attaches callback() as a timer overflow interrupt
	mySCoop.start(9000); // start the scheduler with a goal of Nus per whole cycle. adjust quantum accordingly
						// this "garantee" that the event (or timers) will be treated within this time window
	TIMSK0  |= (1 << OCIE0A);  // enable TIMER0 COMPA Interupt
  	while (1) yield();  // by this way we are in total control of what the program does
	} // start the scheduler 
volatile int count;
void loop() {  

	//int oldcount = -1; yield();
	//if (oldcount != count) {
	//	Serial.print("seconds spent : ");
	//	Serial.println(count); oldcount = count;
	//}

}	// non usato




