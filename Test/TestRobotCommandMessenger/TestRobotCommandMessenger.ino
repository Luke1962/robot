// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	
#include <SystemConfig.h>


//#include <FrequencyTimer2\FrequencyTimer2.h>	
#include <TimerThree\TimerThree.h>
#include <FlexiTimer2\FlexiTimer2.h>
#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
#include <avr/wdt.h>
#include <MyRobotLibs/robot.h>

#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
 	Servo servoSonar;
	NewPing Sonar(Pin_SonarTrig,Pin_SonarEcho);
#endif
#if 1

#include <TinyGPSplus/TinyGPS++.h>
//static const int RXPin = 4, TXPin = 3;
//static const uint32_t GPSBaud = SERIAL_GPS_BAUD_RATE;

// The serial connection to the GPS device
//SoftwareSerial SerialGps(RXPin, TXPin);


TinyGPSPlus gps;

#endif

struct robot_c robot;

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger/CmdMessenger.h>
CmdMessenger cmdWiFi = CmdMessenger(SERIAL_WIFI);
CmdMessenger cmdMMI = CmdMessenger(SERIAL_MMI);
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


bool hasExpired(unsigned long &prevTime, unsigned long interval) {
  if (  millis() - prevTime > interval ) {
    prevTime = millis();
    return true;
  } else     
    return false;
}



// ------------------ M A I N  ----------------------

// Setup function
void setup() 
{
	Serial.begin(SERIAL_BAUD_RATE); // Listen on serial connection for messages from the pc
	// Send the status to the PC that says the Arduino has booted

	#if OPT_SPEECH 
		SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	#endif

	#if  OPT_SERVOSONAR  
		//NewPing Sonar(Pin_SonarTrig,Pin_SonarEcho,robot.parameter.sonarMaxDistance);
		// passo a robot gli indirizzi  di servoSonar e Sonar
		robot.initServoAndSonar( &servoSonar, &Sonar );
	#endif


	// Setup CommandMessenger -----------------------------------------------------
	//cmd.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks();// Attach my application's user-defined callback methods


	//------------------------------------------------------------------------------
	
	//cmd.sendCmd(Msg,"TestRobotCommandMessenger.ino ready!");

	robot.SetMode( ROBOT_STARTING_MODE );
	Serial.print( "1, -- Start IBIT.. ;" );

	robot.runIBIT( 400, &servoSonar, &Sonar );	
	Serial.print("1, -- End IBIT.. ;");


	Serial.print("1, Scanning...;");
	robot.SonarScanBatch( &servoSonar, &Sonar );
	// invia i dati 
	kbSonarSendData();

	delay( 200 );


	/*
	//initialize all timers except for 0, to save time keeping functions
	InitTimersSafe();  // fa casino con il servo
	//sets the frequency for the specified pin
	bool success = SetPinFrequencySafe( Pin_MotCK, ROBOT_MOTOR_CLOCK_microsecondMAX );

	//if the pin frequency was set successfully, turn pin 13 on
	if (success) {
		pinMode( Pin_MotCK, OUTPUT );
		digitalWrite( Pin_MotCK, HIGH );
	}
	*/	
	cmdMsg( "Starting mode:" ); 
	cmdMsg(robot.getOperatingModeStr());

	cmdMsg( "Sensors initial status" ); 
	OnCmdGetSensorsHRate();

}



// Returns if it has been more than interval (in ms) ago. Used for periodic actions
unsigned long TimerHB = 0;  // Hearthbeat
#define msecTimerHB 500 

unsigned long TimerSensorHrate = 0;   // Task con cadenza 1s
#define msecTimer1s 1000 

#define msecTimer5s 5000 
unsigned long  TimerSensorLrate = 0;   // Task con cadenza 5s

unsigned long Timer10s = 0;   // Task con cadenza 10s
#define msecTimer10s 10000 

unsigned long TimerJoystickFeedHrate = 0;   
#define msecTimerJoystickFeedHrate 900 
unsigned long TimerJoystickFeedLrate = 0;   
#define msecTimerJoystickFeedLrate 10000 
//=========================================================================================
// Loop function
//=========================================================================================
void loop() 
{
	// Process incoming serial data, and perform callbacks
	//cmd.feedinSerialData();
	// HEART BEAT -----------------------------------------------
	if (hasExpired(TimerHB, msecTimerHB)) // Toggle every msecs
	{
		Serial.println("1,.;");
		robot.blinkLed(13,500);
	}



	if (hasExpired(TimerSensorHrate, msecTimer1s )) // Toggle every msecs
	{	
		OnCmdGetSensorsHRate();
		// chiedo chi sei solo se è attivo il pir da meno di un secondo
		if ((robot.statusOld.pirDome != robot.status.pirDome)&&(robot.status.ts-robot.statusOld.ts<1000))
		{
			SPEAK_CIAOCHISEI
		}
	}
	if (hasExpired(TimerSensorLrate, msecTimer10s )) // Toggle every msecs
	{	
		OnCmdGetSensorsLRate(); 
	}

	switch (robot.operatingMode)
	{

		//=========================================================================================
		// SLAVE
		//=========================================================================================
		case SLAVE:
			cmdMMI.feedinSerialData();
			 
		break;
		//=========================================================================================
		// JOYSTICK
		//=========================================================================================
		case JOYSTICK:
			if (!robot.obstacleFree()) 	{robot.stop();}


			if  (hasExpired(TimerJoystickFeedLrate, msecTimerJoystickFeedLrate)) // Toggle every 10 secs
			{
				OnCmdGetSensorsLRate();
			}			 
			if  (hasExpired(TimerJoystickFeedHrate, msecTimerJoystickFeedHrate)) // Toggle every 1 secs
			{
				OnCmdGetSensorsHRate();
			}

		break;
		//=========================================================================================
		// AUTONOMOUS
		//=========================================================================================
		case AUTONOMOUS:

 
			// Process incoming serial data, and perform callbacks
			cmdWiFi.feedinSerialData();
			//robot.SonarScanBatch( &servoSonar, &Sonar );
			// invia i dati 


			// SENSORI HR-----------------------------------------------
			if (hasExpired( TimerHB, 1000 )) // Toggle every msecs
			{
				kbSonarSendData();
				int alfa = 0;
				alfa =90 - robot.SonarFindMaxDistAngle( &servoSonar, &Sonar ) ;
				Serial.print( "1,Rot." ); Serial.print( alfa ); Serial.println( ";" );
				robot.rotateDeg( alfa );
				robot.moveCm(10);
				OnCmdGetSensorsHRate();
				OnCmdGetSensorsLRate();
				
				//	Serial.print( "1,RAM:" ); Serial.print( robot.freeRam() ); Serial.print( ";" );
			}

			// SENSORI -----------------------------------------------



			


		break;	
 	

	}














 

  // Toggle LED periodically. If the LED does not toggle every 2000 ms, 
  // this means that cmdMessenger are taking a longer time than this  

  
}


