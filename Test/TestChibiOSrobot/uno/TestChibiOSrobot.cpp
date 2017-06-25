// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle propriet� del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>

#if 1

	#include <TinyGPS++.h>
	#include "arduino.h"

static THD_FUNCTION(Thread1, arg);
static THD_FUNCTION(Thread2, arg);
static THD_FUNCTION(Thread3, arg);
static THD_FUNCTION(Thread4, arg);
static THD_FUNCTION(SerialHandler,arg);
//
//
static const int RXPin = 4, TXPin = 3;
	static const uint32_t GPSBaud = 4800;

	// The serial connection to the GPS device
	//SoftwareSerial SerialGps(RXPin, TXPin);
	#define SerialGps Serial1
	
	TinyGPSPlus gps;

#endif

#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	

//#include <FrequencyTimer2\FrequencyTimer2.h>	
//#include <TimerThree\TimerThree.h>
//#include <FlexiTimer2\FlexiTimer2.h>
//#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
//#include <avr/wdt.h>
#include <robot/robot.h>


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////

#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
 	Servo servoSonar;
	NewPing Sonar(Pin_SonarTrig,Pin_SonarEcho);
#endif

 struct robot_c robot;
// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger/CmdMessenger.h>
static CmdMessenger cmd = CmdMessenger(Serial);
#include <robot/RobotInterfaceCommands.h>



// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
// thread 1		- Esplora
// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {
	const int FWDIST = 10;
	int alfa = 0;
	int cmDone = 0;
	int stuckCount = 0;
	// Flash led every 200 ms.

	while (1) 
	{
		if (robot.operatingMode== AUTONOMOUS)
		{
			robot.parameter.sonarStartAngle = 0;
			robot.parameter.sonarEndAngle = 180;
			robot.parameter.sonarStepAngle = 30;
			robot.parameter.sonarScanSweeps = 1;
			robot.parameter.sonarMedianSamples = 2;
			robot.parameter.sonarScanSpeed = map(analogRead(Pin_AnaPot1),0,1023, 10, 500);  //was = 30 ms di attesa tra due posizioni
			 
			robot.SonarScanBatch(&servoSonar, &Sonar);
			alfa = 90 - robot.parameter.SonarMaxDistAngle;

			// invia i dati Sonar
			kbSonarSendData();

	//		printf("1,Rot. %d ;", alfa);

			robot.rotateDeg(alfa);
			cmDone =robot.moveCm(FWDIST);	// avanti 
			if (cmDone < (FWDIST -1))	//ostacolo ?
			{ 
				//Serial.print("1,Obst!;");
				robot.moveCm(-FWDIST);	// torna indietro
				stuckCount++;
				if (stuckCount>2)
				{
					robot.rotateDeg(180); //inverto la direzione
					stuckCount = 0;
				}
			}
			else //nessun ostacolo, azzero il contatore
			{
				stuckCount = 0;
			}
		
			chThdSleepMilliseconds(500);	// Sleep for 150 milliseconds.

		}
		else   // non autonomo
		{
				
			OnCmdGetSensorsHRate();

			chThdSleepMilliseconds(500);	// Sleep for 50 milliseconds.
		}
	}
//  return 0;
}


// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
// thread 2 - invio stato sensori
// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////

static THD_WORKING_AREA(waThread2, 64);
static THD_FUNCTION(Thread2, arg) {

	while (1) 
	{
		LEDTOP_ON
		OnCmdGetSensorsHRate();
		LEDTOP_OFF						// Turn LED off.	 
		//kbSonarSendData();

		chThdSleepMilliseconds(1000);

	}
//  return 0;
}

// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
// thread 3 - Invio sensori a bassa frequenza
// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////

static THD_WORKING_AREA(waThread3, 200);
static THD_FUNCTION(Thread3, arg) {
	Serial.begin(SERIAL_BAUD_RATE);
	cmd.printLfCr();   // Adds newline to every command 

	//loop ---------------------------------------------------------------
	while (1) {
		//Serial.print("*");
		OnCmdGetSensorsLRate();		
		if (robot.operatingMode == AUTONOMOUS) {

		}
		if (robot.sensors.switchTop) { robot.SetMode(AUTONOMOUS); }
		else { robot.SetMode(SLAVE); }
		chThdSleepMilliseconds(5000);// Sleep for n milliseconds.
	}
//	return 0;
}




// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
// thread 4 - gps
// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waThread4, 64);
static THD_FUNCTION(Thread4, arg) {
	// Setup ---------------------------------------------
	SerialGps.begin(GPSBaud);

	while (1) {
		
		while (SerialGps.available() > 0)	gps.encode(SerialGps.read());
		if (gps.location.isUpdated()) {
			//printf("1,SATS= %l ;", gps.satellites.value());
			//printf("1,LAT= %l ;", gps.location.lat());
			//printf("1,LONG== %l ;", gps.location.lng());
			//printf("1,ALT== %f ;", gps.altitude.meters());
			Serial.print("1,");
			Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
			Serial.print("LAT=");  Serial.print(gps.location.lat(), 6);
			Serial.print(", LONG="); Serial.print(gps.location.lng(), 6);
			Serial.print(", ALT=");  Serial.print(gps.altitude.meters());
			Serial.print(";");
		}
		// Sleep for n milliseconds.
		chThdSleepMilliseconds(2300);
	}
//  return 0;
}


// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
// thread   SerialHandler - ricezione comandi
// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////

static THD_WORKING_AREA(waSerialHandler, 256);
static THD_FUNCTION(SerialHandler,arg) {
//static msg_t SerialHandler(void *arg)


	// Setup CommandMessenger -----------------------------------------------------
	cmd.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks();// Attach my application's user-defined callback methods
	
	while (true)
	{
		//printf("1,>;");
		cmd.feedinSerialData();
		chThdSleepMilliseconds(200);
		//if (robot.operatingMode == SLAVE) 			{chThdSleepMilliseconds(200);}// Sleep for n milliseconds.
		//else if (robot.operatingMode == AUTONOMOUS)	{chThdSleepMilliseconds(500);}// Sleep for n milliseconds.
		//else										{chThdSleepMilliseconds(100);}// Sleep for n milliseconds.
	}
}

// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
	// put your setup code here, to run once:
	cli();
	halInit();
	chSysInit();

	Serial.begin(SERIAL_BAUD_RATE);
	#if OPT_SPEECH 
		SPEECH_SERIAL.begin(SPEECH_SERIAL_SPEED);
	#endif
	// inizializzazione ROBOT ---------------------
	robot.initServoAndSonar(&servoSonar, &Sonar);
	robot.runIBIT(1000, &servoSonar, &Sonar);
	WEBCAM_ON
	chThdCreateStatic(waSerialHandler,sizeof(waSerialHandler), NORMALPRIO + 300, SerialHandler, NULL);
	chThdCreateStatic(waThread1,sizeof(waThread1), NORMALPRIO + 100, Thread1, NULL);//-Esplora
	chThdCreateStatic(waThread2,sizeof(waThread2),	NORMALPRIO + 200, Thread2, NULL);
	chThdCreateStatic(waThread3,sizeof(waThread3),	NORMALPRIO + 50, Thread3, NULL);
	chThdCreateStatic(waThread4,sizeof(waThread4),	NORMALPRIO + 50, Thread4, NULL);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  chThdSleep(10);

}







