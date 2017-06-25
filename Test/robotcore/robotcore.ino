//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   


#include "dbg.h"

#include "systemConfig.h"
#include "hw_config.h"

#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIBRERIE

#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>
#include <TinyGPSplus/TinyGPS++.h>
#include <StackArray.h>

//// ROS
//#include <ros_lib/ros.h>
//#include <ros_lib/time.h> //non serve
//#include <ros_lib/sensor_msgs/Range.h>

//#include <FrequencyTimer2\FrequencyTimer2.h>	
//#include <TimerThree\TimerThree.h>
//#include <FlexiTimer2\FlexiTimer2.h>
//#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
//#include <avr/wdt.h>
//#include <robotmodel.h>
#include <robot.h>
#include <speak.h>


#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI

#if OPT_SERVOSONAR
// va messo prima dell'istanza del robot
Servo servoSonar;
NewPing Sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif

//struct robot_c robot;	//was  struct robot_c robot;

// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger2/CmdMessenger2.h>
static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
///static CmdMessenger2 cmdWiFi = CmdMessenger2(SERIAL_WIFI);
#include "RobotInterfaceCommands2.h"

//------------------------------------------------------------------------------
#pragma region DEFINIZIONE MAILBOX VOICE
// mailbox size and memory pool object count
const size_t MB_COUNT = 6;

// type for a memory pool object
struct PoolObject_t {
	char* name;
	char str[100];
	int size;
};
// array of memory pool objects
PoolObject_t PoolObject[MB_COUNT];

// memory pool structure
MEMORYPOOL_DECL(memPool, MB_COUNT, 0);

// slots for mailbox messages
msg_t letter[MB_COUNT];

// mailbox structure
MAILBOX_DECL(mailVoice, &letter, MB_COUNT);

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////

#pragma endregion



//#include "stringlib.h"



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS  /////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 1		- Esplora
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waThreadEsplora, 200);
static THD_FUNCTION(ThreadEsplora, arg) {
	const int FWDIST = 10;
	int alfa = 0;
	int cmDone = 0;
	int stuckCount = 0;
	// Flash led every 200 ms.

	while (true)
	{
		//if (robot.operatingMode == AUTONOMOUS) {
		if (robot.status.sensors.switchTop==1) {

			robot.status.parameters.sonarStartAngle = 0;
			robot.status.parameters.sonarEndAngle = 180;
			robot.status.parameters.sonarStepAngle = 30;
			robot.status.parameters.sonarScanSweeps = 1;
			robot.status.parameters.sonarMedianSamples = 2;
			robot.status.parameters.sonarScanSpeed = map(analogRead(Pin_AnaPot1), 0, 1023, 10, 500);  //was = 30 ms di attesa tra due posizioni

			robot.SonarScanBatch(&servoSonar, &Sonar);
			alfa = 90 - robot.status.parameters.SonarMaxDistAngle;
			
			// invia i dati Sonar
			kbSonarSendData(&cmdMMI);

			//		printf("1,Rot. %d ;", alfa);

			robot.rotateDeg(alfa);
			cmDone = robot.moveCm(robot.status.parameters.sonarMaxDistance);	// avanti 
			if (cmDone < (robot.status.parameters.sonarMaxDistance - 1))	//ostacolo ?
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

 	}
	//  return 0;
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 2 - lettura e invio stato sensori via WiFi
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waThreadReadSensors, 200);
static THD_FUNCTION(ThreadReadSensors, arg) {

	while (1)
	{

		robot.readSensors();	//IR proxy, Gyro, GPS
		// da tenere allineata con robot.cs >  OnkbGetSensorsHighRate(ReceivedCommand arguments)

/*
		cmdWiFi.sendCmdStart(kbGetSensorsHRate);

		cmdWiFi.sendCmdArg(robot.posCurrent.x);	//robot position X
		cmdWiFi.sendCmdArg(robot.posCurrent.y);	//robot position y
		cmdWiFi.sendCmdArg(robot.posCurrent.r);	//robot position alfa gradi

		cmdWiFi.sendCmdArg(robot.status.irproxy.fw);	// IR proxy
		cmdWiFi.sendCmdArg(robot.status.irproxy.fwHL);	// IR proxy
		cmdWiFi.sendCmdArg(robot.status.irproxy.bk);	// IR proxy
		cmdWiFi.sendCmdArg(robot.status.pirDome);		// movimento

		cmdWiFi.sendCmdArg(robot.status.analog[0]);	//pot
		cmdWiFi.sendCmdArg(robot.status.analog[1]);	//batteria
		cmdWiFi.sendCmdArg(robot.status.analog[2]);	//light

		cmdWiFi.sendCmdArg(robot.getReleStatus(0));		//rele 1
		cmdWiFi.sendCmdArg(robot.getReleStatus(1));		//rel2

		cmdWiFi.sendCmdArg(digitalReadFast(Pin_MotENR));	//status motori
		cmdWiFi.sendCmdArg(digitalReadFast(Pin_MotENL));

		cmdWiFi.sendCmdArg(robot.status.switchTop); // status switch modo Autonomo/slave
		cmdWiFi.sendCmdArg(robot.status.laserOn); // laser
		cmdWiFi.sendCmdArg(robot.readBattChargeLevel());


		cmdWiFi.sendCmdArg(robot.status.gps.sats);		//gps
		cmdWiFi.sendCmdArg(robot.status.gps.lat);
		cmdWiFi.sendCmdArg(robot.status.gps.lng);



		cmdWiFi.sendCmdEnd();
*/

		// Gestione cambio modalità-----------------------------------------		
		if (robot.status.sensors.switchTop != robot.statusOld.sensors.switchTop) {
			if (robot.status.sensors.switchTop)
			{
				// Autonomo
				robot.status.operatingMode = AUTONOMOUS;
				SPEAK("okei esploro")
					SERIAL_MSG.print("1, operatingMode = AUTONOMOUS;");
			}
			else
			{
				robot.status.operatingMode = SLAVE;
				SERIAL_MSG.print("1, operatingMode = SLAVE;");
				SPEAK("okei comanda")
			}
		}
		//-------------------------------------------------------------------
		// chiedo chi sei solo se è attivo il pir da meno di un secondo
		if ((robot.statusOld.sensors.pirDome != robot.status.sensors.pirDome) && (robot.status.ts - robot.statusOld.ts > 10000))
		{
			SPEAK_CIAOCHISEI
		}

		//rate d'invio in base alla modalità operativa
		if (robot.status.operatingMode= AUTONOMOUS)
		{
			chThdSleepMilliseconds(1000);// Sleep for n milliseconds.

		}
		else
		{
			chThdSleepMilliseconds(500);// Sleep for n milliseconds.

		}
	}
	//  return 0;
}
/*
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 3 - Invio sensori a bassa frequenza
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waThreadSendSensorsLR, 200);
static THD_FUNCTION(ThreadSendSensorsLR, arg) {
	//SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	cmdWiFi.printLfCr();   // Adds newline to every command 

						   //loop ---------------------------------------------------------------
	while (1) {
		robot.readGps();
		SERIAL_MSG.print("1, SATS=");
		SERIAL_MSG.print(robot.status.gps.sats); // Number of satellites in use (u32)
		SERIAL_MSG.print(" LAT=");  SERIAL_MSG.print(robot.status.gps.lat, 6);
		SERIAL_MSG.print(" LONG="); SERIAL_MSG.print(robot.status.gps.lng, 6);
		SERIAL_MSG.print(" ALT=");  SERIAL_MSG.print(robot.status.gps.alt);
		SERIAL_MSG.print(" Home dist=");  SERIAL_MSG.print(robot.status.gps.homeDistCm);
		SERIAL_MSG.print(" Home angle=");  SERIAL_MSG.print(robot.status.gps.homeAngleDeg);
		SERIAL_MSG.println(";");

		
		chThdSleepMilliseconds(5000);// Sleep for n milliseconds.
	}
	//	return 0;
}*/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 4 - ROS SERIAL
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*
	ros::Time time;

	ros::NodeHandle  nh;

	sensor_msgs::Range range_msg;
	ros::Publisher pub_range("/ultrasound", &range_msg);

	const int adc_pin = 0;

	unsigned char frameid[] = "/ultrasound";



	long range_time;

static THD_WORKING_AREA(waThreadROS, 200);
static THD_FUNCTION(ThreadROS, arg) {
	
	//* rosserial Ultrasound Example
	//*
	//* This example is for the Maxbotix Ultrasound rangers.
	
 
 
		nh.initNode();
		nh.advertise(pub_range);


		range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
		range_msg.header.frame_id = frameid;
		range_msg.field_of_view = 0.1;  // fake
		range_msg.min_range = 0.0;
		range_msg.max_range = 6.47;

		///pinMode(8, OUTPUT);
		///digitalWrite(8, LOW);

	//loop ---------------------------------------------------------------  
	while (1) {
 
		//publish the adc value every 50 milliseconds
		//since it takes that long for the sensor to stablize
		int r = 0;

		range_msg.range = (float)robot.SonarPing();
		range_msg.header.stamp = range_msg.header.stamp.now();
		pub_range.publish(&range_msg);
		range_time = millis() + 50;

		nh.spinOnce();

		chThdSleepMilliseconds(50);// Sleep for n milliseconds.
	}
	//	return 0;
}
*/
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   MMIcommands - ricezione comandi da MMI
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

static THD_WORKING_AREA(waMMIcommands, 400);
static THD_FUNCTION(MMIcommands, arg) {
	//static msg_t MMIcommands(void *arg)


	// Setup CommandMessenger -----------------------------------------------------
	cmdMMI.printLfCr();   // Adds newline to every command 
						  //	cmdWiFi.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

	while (true)
	{

//		robot.readSensors();	

		// Comandi da interfaccia MMI ?
		cmdMMI.feedinSerialData();

		cmdMMI.sendCmdStart(kbGetSensorsHRate);

		cmdMMI.sendCmdArg(robot.status.posCurrent.x);	//robot position X
		cmdMMI.sendCmdArg(robot.status.posCurrent.y);	//robot position y
		cmdMMI.sendCmdArg(robot.status.posCurrent.r);	//robot position alfa gradi

		cmdMMI.sendCmdArg(robot.status.sensors.irproxy.fw);	// IR proxy
		cmdMMI.sendCmdArg(robot.status.sensors.irproxy.fwHL);	// IR proxy
		cmdMMI.sendCmdArg(robot.status.sensors.irproxy.bk);	// IR proxy
		cmdMMI.sendCmdArg(robot.status.sensors.pirDome);		// movimento

		cmdMMI.sendCmdArg(robot.status.sensors.analog[0]);	//pot
		cmdMMI.sendCmdArg(robot.status.sensors.analog[1]);	//batteria
		cmdMMI.sendCmdArg(robot.status.sensors.analog[2]);	//light

		cmdMMI.sendCmdArg(robot.getReleStatus(0));		//rele 1
		cmdMMI.sendCmdArg(robot.getReleStatus(1));		//rel2

		cmdMMI.sendCmdArg(digitalReadFast(Pin_MotENR));	//status motori
		cmdMMI.sendCmdArg(digitalReadFast(Pin_MotENL));

		cmdMMI.sendCmdArg(robot.status.sensors.switchTop); // status switch modo Autonomo/slave
		cmdMMI.sendCmdArg(robot.status.act.laserOn); // laser

		cmdMMI.sendCmdArg(robot.readBattChargeLevel()); //0-100


		cmdMMI.sendCmdArg(robot.status.sensors.gps.sats);		//gps
		cmdMMI.sendCmdArg(robot.status.sensors.gps.lat);
		cmdMMI.sendCmdArg(robot.status.sensors.gps.lng);

		//cmdMMI.sendCmdArg(robot.status.gps.alt);
		//cmdMMI.sendCmdArg(robot.status.gps.homeDistCm);
		//cmdMMI.sendCmdArg(robot.status.gps.homeAngleDeg);

		cmdMMI.sendCmdEnd();

		chThdSleepMilliseconds(800);
		//if (robot.operatingMode == SLAVE) 			{chThdSleepMilliseconds(200);}// Sleep for n milliseconds.
		//else if (robot.operatingMode == AUTONOMOUS)	{chThdSleepMilliseconds(500);}// Sleep for n milliseconds.
		//else										{chThdSleepMilliseconds(100);}// Sleep for n milliseconds.
	}
}
//////////////////////////////////////////////////////////////////////////////////
//  blinking LED       ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region  Processo blinking LED  
#define PIN_LED  13
// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waFlashLed, 64);
static THD_FUNCTION(FlashLed, arg) {
	// Flash led every 200 ms.
	pinMode(PIN_LED, OUTPUT);		digitalWrite(PIN_LED, 0);	// led superiore

	while (1) {
		// Turn LED on.
		LEDTOP_ON
			//digitalWriteFast(PIN_LED, HIGH);

			// Sleep for 50 milliseconds.
			chThdSleepMilliseconds(50);

		// Turn LED off.
		LEDTOP_OFF
			//digitalWriteFast(PIN_LED, LOW);

			// Sleep for 150 milliseconds.
			chThdSleepMilliseconds(950);
	}
}
#pragma endregion 
//////////////////////////////////////////////////////////////////////////////////
// M O N I T O R	   ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo di MONITOR
volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

static THD_WORKING_AREA(waThreadMonitor, 64);
static THD_FUNCTION(ThreadMonitor, arg) {

	while (true)
	{
		Serial.print(F("    waMMIcommands unused stack: "));
		Serial.println(chUnusedStack(waMMIcommands, sizeof(waMMIcommands)));
		//Serial.print(F("    overrun errors: "));
		//Serial.println( OverrunErrorCount);

		count++;
		//FIFO_SPEAK.push(int2str(count));
		uint32_t t = micros();
		// yield so other threads can run
		chThdYield();
		t = micros() - t;
		if (t > maxDelay) maxDelay = t;

		chThdSleepMilliseconds(3000);//	chThdYield();//	
									 //		SPEAK_CIAO
	}

}

#pragma endregion 

// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region [CHIBIOS RTOS]

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
	uint8_t newVariable;
	// heap is empty, use bss as start memory address 
	if ((uint16_t)__brkval == 0)
		return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
	// use heap end as the start of the memory address 
	else
		return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

void chSetup() {
	Serial.println("robot chibios");
	// fill pool with PoolObject array
	for (size_t i = 0; i < MB_COUNT; i++) {
		chPoolFree(&memPool, &PoolObject[i]);
	}

	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 10, FlashLed, NULL);
	chThdCreateStatic(waMMIcommands, sizeof(waMMIcommands), NORMALPRIO + 1, MMIcommands, NULL);
//	chThdCreateStatic(waThreadEsplora, sizeof(waThreadEsplora), NORMALPRIO + 1, ThreadEsplora, NULL);//-Esplora
	chThdCreateStatic(waThreadReadSensors, sizeof(waThreadReadSensors), NORMALPRIO + 1, ThreadReadSensors, NULL);
//	chThdCreateStatic(waThreadSendSensorsLR, sizeof(waThreadSendSensorsLR), NORMALPRIO + 1, ThreadSendSensorsLR, NULL);
//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 1, ThreadMonitor, NULL);



	dbg("all chThd started..");



}

void setup()
{
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	SERIAL_WIFI.begin(SERIAL_WIFI_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);

	LEDTOP_ON

	// inizializzazione ROBOT ---------------------
	robot.initServoAndSonar(&servoSonar, &Sonar);
	robot.runIBIT(300, &servoSonar, &Sonar);

	WEBCAM_ON
	chBegin(chSetup);	while (1) {}
}


void loop() {
	// not used
}

#pragma endregion


