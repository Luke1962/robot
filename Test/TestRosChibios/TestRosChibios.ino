// COMANDI PER TEST: 
//AVANTI 20 E INDIETRO 20: 
//19,20;19,-20;


//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
//#define delay(ms) chThdSleepMilliseconds(ms) 

#include <MyRobotLibs\dbg.h>

#include <MyRobotLibs\systemConfig.h>
#include <MyRobotLibs\hw_config.h>

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
//#include <TinyGPSplus/TinyGPS++.h>
//#include <StackArray.h>
#include <Arduino.h>	//per AttachInterrupt



#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
#if OPT_COMPASS
	#include <Wire\Wire.h>
	#include <compass\compass.h>
	MyCompass_c compass;
	//#include <Adafruit_Sensor\Adafruit_Sensor.h> //richiesto dalla liberia compass Adafruit_HMC5883_U
	//#include <HMC5883L\HMC5883L.h>
	//HMC5883L compass;
#endif // COMPASS

#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	#include <Newping\NewPing.h>
	#include <Servo\src\Servo.h>
	#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)

	#include <PWM\PWM.h>
	Servo servoSonar;
	NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif

#if OPT_GPS
	#include <TinyGPSplus\TinyGPS++.h> //deve essere incluso anche nel main

	TinyGPSPlus Gps;
#endif

#pragma region VL53L0X distanceSensor

#if OPT_LDS
	#include <Wire\Wire.h>
	#include <VL53L0X\VL53L0X.h>
	VL53L0X LDS;
	// Uncomment this line to use long range mode. This
	// increases the sensitivity of the distanceSensor and extends its
	// potential range, but increases the likelihood of getting
	// an inaccurate reading because of reflections from objects
	// other than the intended target. It works best in dark
	// conditions.
	#define LONG_RANGE
	// Uncomment ONE of these two lines to get
	// - higher speed at the cost of lower accuracy OR
	// - higher accuracy at the cost of lower speed

	//#define HIGH_SPEED
	//#define HIGH_ACCURACY

#endif // OPT_LDS

#pragma endregion

#include <robot.h>
struct robot_c robot;	//was  struct robot_c robot;


// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#if OPT_MMI
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>
	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//------------------------------------------------------------------------------
#endif

#pragma region DEFINIZIONE MAILBOX VOICE
// mailbox size and memory pool object count
const size_t MBOX_COUNT = 6;

// type for a memory pool object
struct mboxObject_t {
	char* name;
	char str[100];
	int size;
};
// array of memory pool objects
mboxObject_t msgObjArray[MBOX_COUNT];

// memory pool structure
//MEMORYPOOL_DECL(memPool, MBOX_COUNT, 0);

// slots for mailbox messages
//msg_t letter[MBOX_COUNT];

// mailbox structure
//MAILBOX_DECL(mailVoice, &letter, MBOX_COUNT);





/// ///////////////////////////////////////////////////////////////////////////////
// M U T E X ///////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
// Mutex for atomic access to data.
MUTEX_DECL(mutexMotion); //condiviso dai comandi di movimento e sonar per non essere in movimento quando il sonar scansiona
MUTEX_DECL(mutexSerialMMI);// accesso alla seriale
MUTEX_DECL(mutexSerialPC);// accesso alla seriale
MUTEX_DECL(mutexSensors);// accesso ai sensori in lettura e scrittura

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  FUNZIONI E UTILITIES GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
//void playSingleNote(int pin, int freq,int noteDuration) {
//	tone(pin, freq, noteDuration);
//	noTone(pin);
//}
//Dichiarazione di funzione che punta all’indirizzo zero
void(*softReset)(void) = 0;

volatile bool interruptFlag = 0;
// Interrupt service routines for the right motor's quadrature encoder
void ISRencoder()
{
//	noInterrupts();
	robot.status.cmd.stepsDone+=25;  //8 tacche per 200 step al giro
	//digitalWriteFast(Pin_LED_TOP_G, interruptFlag);
	//digitalWriteFast(13, interruptFlag);
	//interruptFlag = !interruptFlag;
//	interrupts();

}
#pragma endregion

#pragma region helperFunctions
// ////////////////////////////////////////////////////////////////////////////////////////////

void lampeggiaLed(int pin, int freq, uint16_t nvolte) {
	int ms = 500 / freq;
	for (size_t i = 0; i < nvolte; i++)
	{
		digitalWriteFast(pin, 1);
		chThdSleepMilliseconds(ms);
		digitalWriteFast(pin, 0);
		chThdSleepMilliseconds(ms);

	}

}
// ////////////////////////////////////////////////////////////////////////////////////////////

void countDown(int seconds) {
	MSG3("CountDown in ", seconds, " sec...");
	for (size_t i = seconds; i > 0; i--)
	{
		MSG2("  -", i);
		delay(1000);
	}
}
// ////////////////////////////////////////////////////////////////////////////////////////////

#pragma endregion


/// ///////////////////////////////////////////////////////////////////////////////
// ROS
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region ROS
#include <ros.h>
#include <ros/duration.h>
#include <ros/time.h> //non serve
#include <sensor_msgs/Range.h>

	ros::Time time;

	ros::NodeHandle  nh;

  
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  ROS_INFO2F                      ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	#define ROS_INFO(s) nh.loginfo(s);

	void ROS_INFO2F(char * c, float f) {
		#define CHARFLOATSIZE 10
		#define CHARFLOATDECS 3
		char charVal[CHARFLOATSIZE];  //temporarily holds data from vals
									  //4 is mininum width, 3 is precision; float value is copied onto buff
		dtostrf(f, CHARFLOATSIZE, CHARFLOATDECS, charVal);
		nh.loginfo(c);
		nh.loginfo(charVal);
	}

	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  ultrasound                         ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	#include <sensor_msgs/Range.h>    // ultrasound
	sensor_msgs::Range rosmsg_range;
	ros::Publisher pub_range("ultrasound", &rosmsg_range);
	char frameid_ultrasound[] = "/ultrasound_link";
	unsigned long ultrasound_time;
	//-----------------------------------------------------------

	float getRange_Ultrasound(int pin_num) {
		int val = 0;
		for (int i = 0; i < 4; i++) val += analogRead(pin_num);
		float range = val;
		return range / 322.519685;  // (0.0124023437 /4) ; //cvt to meters
	}

	void setup_ultrasound() {
		rosmsg_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
		rosmsg_range.header.frame_id = frameid_ultrasound;
		rosmsg_range.field_of_view = 0.1;  // fake
		rosmsg_range.min_range = 0.0;
		rosmsg_range.max_range = 4;
		ROS_INFO("setup_ultrasound");


		nh.advertise(pub_range);
	}
	void publish_ultrasound() {
		//publish ULTRASOUND
		if (millis() >= ultrasound_time) {
			int r = 0;

			rosmsg_range.range = getRange_Ultrasound(5);
			rosmsg_range.header.stamp = nh.now();
			pub_range.publish(&rosmsg_range);

			nh.spinOnce();
			ROS_INFO("P. ULTRASOUND");
			ultrasound_time = millis() + 2500;
		}


	}






	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  twist command                       ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	#include <geometry_msgs/Twist.h>  // cmd_vel


	unsigned long twist_command_time = 0;


	//-----------------------------------------------------------------
	void cmdcb_cmd_vel(const geometry_msgs::Twist & cmd_vel);
	ros::Subscriber <geometry_msgs::Twist> command_cmd_vel("/cmd_vel", cmdcb_cmd_vel);

	void cmdcb_cmd_vel(const geometry_msgs::Twist & cmd_vel) {
		//callback function every time linear and angular speed is received from 'cmd_vel' topic
		//this callback function receives cmd_msg object where linear and angular speed are stored


		//robot.status.cmd.targetVelocityLinear = cmd_vel.linear.x;
		//robot.status.cmd.targetVelocityAngular = cmd_vel.angular.z;


		twist_command_time = millis();

		robot.goCmdVel(cmd_vel.linear.x, cmd_vel.angular.z, twist_command_time);

		ROS_INFO2F(" vel  x:", robot.status.cmd.targetVelocityLinear);
		ROS_INFO2F(" vel  z:", robot.status.cmd.targetVelocityAngular);


		//robot_time = millis() + (1000);


	}


	void setup_cmd_vel() {
		nh.subscribe(command_cmd_vel);
		ROS_INFO("setup_cmd_vel");
	}

#pragma endregion
// ////////////////////////////////////////////////////////////////////////////////////////////



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
/*
static THD_WORKING_AREA(waThreadEsplora, 400);
static THD_FUNCTION(ThreadEsplora, arg) {
const int FWDIST = 10;
int alfa = 0;
int cmDone = 0; // percorso eseguito a valle di un comando moveCm o RotateDeg
int stuckCount = 0;

while (robot.status.operatingMode == AUTONOMOUS)
{

TOGGLEPIN(Pin_LED_TOP_B);
robot.status.parameters.sonarStartAngle = 0;
robot.status.parameters.sonarEndAngle = 180;
robot.status.parameters.sonarStepAngle = 30;
robot.status.parameters.sonarScanSweeps = 1;
robot.status.parameters.sonarMedianSamples = 2;
robot.status.parameters.sonarScanSpeed = 30; // map(analogRead(Pin_AnaPot1), 0, 1023, 10, 500);  //was = 30 ms di attesa tra due posizioni

robot.SonarScanBatch(&servoSonar, &Sonar);
alfa = 90 - robot.status.parameters.SonarMaxDistAngle;
SERIAL_MSG.print("Max dist @alfa:"); SERIAL_MSG.println(alfa);
dbg2("Max dist cm:", robot.status.parameters.sonarMaxDistance)
TOGGLEPIN(Pin_LED_TOP_B);

// invia i dati Sonar
kbSonarSendData(&cmdMMI);
TOGGLEPIN(Pin_LED_TOP_B);


robot.rotateDeg(alfa);
cmDone = robot.moveCm(robot.status.parameters.sonarMaxDistance);	// avanti
if (cmDone < (robot.status.parameters.sonarMaxDistance - 1))	//ostacolo ?
{
SERIAL_MSG.println("1,Obst!;");
robot.moveCm(-FWDIST);	// torna indietro
TOGGLEPIN(Pin_LED_TOP_B);
stuckCount++;
if (stuckCount>2)
{
robot.rotateDeg(180); //inverto la direzione
TOGGLEPIN(Pin_LED_TOP_B);
stuckCount = 0;
}
}
else //nessun ostacolo, azzero il contatore
{
stuckCount = 0;
}
TOGGLEPIN(Pin_LED_TOP_B);


chThdSleepMilliseconds(1500);	// Sleep for 150 milliseconds.

}
//  return 0;
}
*/


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   - SAFETY
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waSafety, 100);
static THD_FUNCTION(thdSafety, arg) {
	//robot.status.sensors.ignoreIR = true;

	while (true)
	{
		if (robot.status.isMoving && !robot.isObstacleFree())
		{
			robot.stop();
			MSG("Obstacle-Stop motor")
		}
		chThdSleepMilliseconds(200);
	}
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   - lettura sensori HR in robot.status
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

static THD_WORKING_AREA(waReadSensorsHR, 400);
static THD_FUNCTION(thdReadSensorsHR, arg) {
//	chMtxLock(&mutexSensors);
	// Allo startup leggo tutti isensori
	robot.readAllSensors();	//IR proxy, Gyro, GPS
//	chMtxUnlock(&mutexSensors);

	while (1)
	{
		LEDTOP_B_ON

//		chMtxLock(&mutexSensors);
		MSG("S HR>")
		robot.readSensorsHR();	//IR proxy, Gyro, GPS
		robot.status.tictac = !robot.status.tictac;
//		chMtxUnlock(&mutexSensors);
		LEDTOP_B_OFF


		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_AUTONOMOUS) {
			chThdSleepMilliseconds(500);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(500);
		} 


	}
	//  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 2B - lettura sensori LR in robot.status
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waReadSensorsLR, 400);
static THD_FUNCTION(thdReadSensorsLR, arg) {
	chMtxLock(&mutexSensors);
	// Allo startup leggo tutti isensori
	robot.readAllSensors();	//IR proxy, Gyro, GPS
//	chMtxUnlock(&mutexSensors);

	while (1)
	{
		LEDTOP_B_ON

//		chMtxLock(&mutexSensors);
		MSG("S LR>")
		robot.readSensorsLR();	//IR proxy, Gyro, GPS
		robot.status.tictac = !robot.status.tictac;
//		chMtxUnlock(&mutexSensors);
 
		LEDTOP_B_OFF

		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_AUTONOMOUS) {
			chThdSleepMilliseconds(3000);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(5000);
		} 


	}
	//  return 0;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   MMIcommands - ricezione comandi da MMI
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#if OPT_MMI

static THD_WORKING_AREA(waMMIcommands, 200);
static THD_FUNCTION(thdMMIcommands, arg) {
	//static msg_t MMIcommands(void *arg)


	// Setup CommandMessenger -----------------------------------------------------
	cmdMMI.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

	while (true)
	{
		dbg("1,MMI>; ");

		LEDTOP_B_ON

		///osalSysDisable(); //disabilita Interupt
		cmdMMI.feedinSerialData(); // Comando da interfaccia MMI ?, Se sì lo esegue
		///osalSysEnable();//abilita Interupt
		LEDTOP_B_OFF

			chThdSleepMilliseconds(900);
		//if (robot.operatingMode == SLAVE) 			{chThdSleepMilliseconds(200);}// Sleep for n milliseconds.
		//else if (robot.operatingMode == AUTONOMOUS)	{chThdSleepMilliseconds(500);}// Sleep for n milliseconds.
		//else										{chThdSleepMilliseconds(100);}// Sleep for n milliseconds.
	}
}
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   PCcommands - ricezione comandi da PC (per Debug)
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#if OPT_MMI

static THD_WORKING_AREA(waPCcommands, 100);
static THD_FUNCTION(thdPCcommands, arg) {
	// Setup CommandMessenger -----------------------------------------------------
	cmdPC.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdPC);// Attach my application's user-defined callback methods
	MSG("1,thdPCcommands STARTED;");

	while (true)
	{
		dbg("1,PC>; ");


		//osalSysDisable(); //disabilita Interupt
//		chMtxLock(&mutexSerialPC);
		cmdPC.feedinSerialData();  // Comando da pc ?, Se sì lo esegue
//		chMtxUnlock(&mutexSerialPC);
		//osalSysEnable();//abilita Interupt


		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_SLAVE) {
			chThdSleepMilliseconds(500);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(1000);
		}// Sleep for n milliseconds.

	}
}

// invia un  messaggio con la descrizione dell'evento e lo resetta 
void msgEventHR() {
	if (robot.status.pendingEvents.bumperF) { MSG("bumperF  EVENT");robot.status.pendingEvents.bumperF = false; }
	if (robot.status.pendingEvents.bumperL) { MSG("bumperL  EVENT");robot.status.pendingEvents.bumperL = false; }
	if (robot.status.pendingEvents.bumperR) { MSG("bumperR  EVENT");robot.status.pendingEvents.bumperR = false; }
	if (robot.status.pendingEvents.EncL) { MSG("EncL  EVENT");robot.status.pendingEvents.EncL = false; }
	if (robot.status.pendingEvents.EncR) { MSG("EncR  EVENT");robot.status.pendingEvents.EncR = false; }
	if (robot.status.pendingEvents.irproxyB) { MSG("irproxyB  EVENT");robot.status.pendingEvents.irproxyB = false; }
	if (robot.status.pendingEvents.irproxyF) { MSG("irproxyF  EVENT");robot.status.pendingEvents.irproxyF = false; }
	if (robot.status.pendingEvents.irproxyFH) { MSG("irproxyFH  EVENT");robot.status.pendingEvents.irproxyFH = false; }
	if (robot.status.pendingEvents.irproxyL) { MSG("irproxyL  EVENT");robot.status.pendingEvents.irproxyL = false; }
	if (robot.status.pendingEvents.irproxyR) { MSG("irproxyR  EVENT");robot.status.pendingEvents.irproxyR = false; }

	if (robot.status.pendingEvents.pirDome) { MSG("pirDome  EVENT");robot.status.pendingEvents.pirDome = false; }
	//if (robot.status.pendingEvents.analog[0]) { MSG("POT EVENT"); robot.status.pendingEvents.analog[0]= false;	}
	//if (robot.status.pendingEvents.analog[2]) { MSG("LIGHT EVENT");robot.status.pendingEvents.analog[2]= false;	 }
	
	
	
	
	
	
	
	
	
	
	
	
	

}
// se pending invia messaggio di evento su Batteria,luca e GPS
void msgEventLR() {

	if (robot.status.pendingEvents.batCharge) { MSG("BATTERY EVENT");robot.status.pendingEvents.batCharge= false;	 }
	if (robot.status.pendingEvents.light) { MSG("light  EVENT"); robot.status.pendingEvents.light = false;}
	if (robot.status.pendingEvents.gps) { MSG("GPS  EVENT"); robot.status.pendingEvents.gps = false;}

}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   SendStatusHR - invia lo stato sui vari canali
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waSendStatusHR, 100);
static THD_FUNCTION(thdSendStatusHR, arg) {
	while (true)
	{
		if (robot.status.pendingEvents.EventFlag)
		{
			LEDTOP_B_ON;
			// wait to enter print region
			//chMtxLock(&mutexSerialMMI);
			msgEventHR();

			osalSysDisable(); //disabilita Interupt
			onCmdGetSensorsHRate(&cmdMMI);
			onCmdGetSensorsLRate(&cmdMMI);
			osalSysEnable();//abilita Interupt

			//chMtxUnlock(&mutexSerialMMI);


			//chMtxLock(&mutexSerialPC);
			onCmdGetSensorsHRate(&cmdPC);
			onCmdGetSensorsLRate(&cmdPC);
			//chMtxUnlock(&mutexSerialPC);



			LEDTOP_B_OFF

		}

		chThdSleepMilliseconds(800);// Sleep for n milliseconds.


	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   SendStatusLR - invia lo stato sui vari canali
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waSendStatusLR, 200);
static THD_FUNCTION(thdSendStatusLR, arg) {

	//chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
	//onCmdGetSensorsLRate(&cmdMMI);
	//chMtxUnlock(&mutexSerialMMI);

	//chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
	//onCmdGetSensorsLRate(&cmdPC);
	//chMtxUnlock(&mutexSerialPC);

	while (true)
	{
		LEDTOP_B_ON
			if ((robot.status.operatingMode == MODE_AUTONOMOUS) &&
		 ( robot.status.pendingEvents.light
			|| robot.status.pendingEvents.batCharge
			|| robot.status.pendingEvents.gps
			))
		{
				//osalSysDisable(); //disabilita Interupt
				// wait to enter print region
				//chMtxLock(&mutexSerialMMI);
				msgEventLR();


			robot.status.pendingEvents.gps = false;
			robot.status.pendingEvents.batCharge = false;
			robot.status.pendingEvents.light = false;

			//		chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
			onCmdGetSensorsLRate(&cmdMMI);
			//		chMtxUnlock(&mutexSerialMMI);

			//		chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
			onCmdGetSensorsLRate(&cmdPC);
			//		chMtxUnlock(&mutexSerialPC);

		}



		LEDTOP_B_OFF

		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_AUTONOMOUS) {
			#ifdef DEBUG_ON
				chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
			#else
				chThdSleepMilliseconds(5000);// Sleep for n milliseconds.
			#endif
		}
		else { //SLEEP TIME IN MODALITA' SLAVE O JOISTICK
			#ifdef DEBUG_ON
				chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
			#else
				chThdSleepMilliseconds(15000);// Sleep for n milliseconds.
			#endif
		}


	}
}

#endif

//////////////////////////////////////////////////////////////////////////////////
//  THREAD  B R A I N      ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*
#pragma region  Processo	B R A I N    
static THD_WORKING_AREA(waBrain, 100);
static THD_FUNCTION(thdBrain, arg)
{
	// Setup CommandMessenger -----------------------------------------------------
	cmdMMI.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

	int sleepTime = 500;
	const int FWDIST = 10; //distanza avanzamento
	int alfa = 0;
	int cmDone = 0; // percorso eseguito a valle di un comando moveCm o RotateDeg
	int stuckCount = 0;

	//1)inizializzo i sensori e le variabili
	int pingCm = 0;
	int ldsmm = 0;
	int estimatedDistCm = 0;
 
	MSG2("OPMODE..",(int)robot.status.operatingMode);

	robot.status.cmd.commandDir = commandDir_e::GOR;

	while (1)
	{
		LEDTOP_G_ON
		switch (robot.status.operatingMode)
		{
		case operatingMode_e::MODE_SLAVE:
			#pragma region SLAVE

				MSG("B s>")

///				osalSysDisable(); //disabilita Interupt
				cmdMMI.feedinSerialData(); // Comando da interfaccia MMI ?, Se sì lo esegue
//				osalSysEnable();//abilita Interupt


				//Invia i dati dei sensori se cambia qualcosa
				if (robot.raiseEvents()) {

					msgEventHR();
					msgEventLR();
					robot.resetEvents();
				}
				//MSG(" okei comanda");
				sleepTime = 1000;
				break;
			#pragma endregion

		case	operatingMode_e::MODE_AUTONOMOUS:

		#pragma region AUTONOMOUS
			sleepTime = 2000;
			MSG("B A>")


			onCmdGetSensorsHRate(&cmdMMI);
			onCmdGetSensorsHRate(&cmdPC);
			onCmdGetSensorsLRate(&cmdMMI);
			onCmdGetSensorsLRate(&cmdPC);
			onCmdGetProxy(&cmdMMI);

			// ////////////////////////////////////////////////////////////////////////////////
			/// ///////////////////////////////////////////////////////////////////////////////
			//  Esplora
			/// ///////////////////////////////////////////////////////////////////////////////
			// ////////////////////////////////////////////////////////////////////////////////
			#pragma region ESPLORA
			#if 0

						MSG("BE>")

							TOGGLEPIN(Pin_LED_TOP_B);
						robot.status.parameters.sonarStartAngle = 0;
						robot.status.parameters.sonarEndAngle = 180;
						robot.status.parameters.sonarStepAngle = 30;
						robot.status.parameters.sonarScanSweeps = 1;
						robot.status.parameters.sonarMedianSamples = 2;
						robot.status.parameters.sonarScanSpeed = 30; // map(analogRead(Pin_AnaPot1), 0, 1023, 10, 500);  //was = 30 ms di attesa tra due posizioni

						robot.LDSScanBatch();
						alfa = 90 - robot.status.parameters.SonarMaxDistAngle;
						alfa = 30;
						MSG2("Max dist @alfa:", alfa);
						MSG2("Max dist cm:", robot.status.parameters.sonarMaxDistance)
							TOGGLEPIN(Pin_LED_TOP_B);

						// invia i dati Sonar all'Host
						//onCmdSonarSendData(&cmdMMI);
						TOGGLEPIN(Pin_LED_TOP_B);

						///osalSysDisable();
						robot.rotateDeg(alfa);
						osalSysEnable();
						cmDone = robot.moveCm(robot.status.parameters.sonarMaxDistance);	// avanti
						if (cmDone < (robot.status.parameters.sonarMaxDistance - 1))	//ostacolo ?
						{
							MSG("Obstacle!");
							robot.moveCm(-FWDIST);	// torna indietro
							TOGGLEPIN(Pin_LED_TOP_B);
							stuckCount++;
							if (stuckCount > 2)
							{
								robot.rotateDeg(180); //inverto la direzione
								TOGGLEPIN(Pin_LED_TOP_B);
								stuckCount = 0;
							}
						}
						else //nessun ostacolo, azzero il contatore
						{
							stuckCount = 0;
						}
						TOGGLEPIN(Pin_LED_TOP_B);


						sleepTime = 1500;	// Sleep for 150 milliseconds.



			#endif // 0
					//  return 0;
			#pragma endregion
			#if 1  //test che funzionino i sensori e movimento in ambiente chibios

						LASER_ON


							robot.go(robot.status.cmd.commandDir, robotSpeed_e::SLOW);
						MSG("MOTORI ATTIVATI")
							////2)avvio la rotazione del robot
							//osalSysDisable();
							////loop
							//while (true)
							//{
							//	// leggo LDS,sonar,heading
								robot.status.posCurrent.r = compass.getBearing();
								pingCm = robot.getLDSDistanceCm();
								ldsmm = LDS.readRangeSingleMillimeters();
								if (!LDS.timeoutOccurred()) {
									estimatedDistCm = ldsmm / 10;
								}
								else //uso il sonar
								{
									estimatedDistCm = pingCm;
								}
								//invio i dati
								onCmdGetPose(&cmdMMI);
								MSG3("Dist: ", estimatedDistCm, "cm")
									//MSG3("Compass: ",robot.status.posCurrent.r,"°")
									//MSG3("lds: ",ldsmm, "mm")
									//	MSG3("Ping: ",pingCm, "cm")
							//}
							//osalSysEnable();

			#endif // TEST_ALL			#pragma region ESPLORA2



				break;

		#pragma endregion

		case operatingMode_e::MODE_JOYSTICK:
			//MSG(" okei comanda col gioistic");
			sleepTime = 5000;
			break;

		default:
			MSG(" modo sconosciuto");
			sleepTime = 2000;
			break;

		}

 



		LEDTOP_G_OFF

		chThdSleepMilliseconds(sleepTime);//	chThdYield();//	

	}
}
#pragma endregion 

*/

// ////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
// thread 4 - ROS SERIAL
/// ///////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////
#if 1


const int adc_pin = 0;

unsigned char frameid[] = "/ultrasound";



long range_time;

static THD_WORKING_AREA(waRos, 400);
static THD_FUNCTION(thdRos, arg) {

	// rosserial Ultrasound Example
	//
	// This example is for the Maxbotix Ultrasound rangers.



	nh.initNode();
	nh.advertise(pub_range);

	setup_ultrasound();
	setup_cmd_vel();

	
	///pinMode(8, OUTPUT);
	///digitalWrite(8, LOW);

	//loop ---------------------------------------------------------------
	while (1) {
		MSG("ros>")
		ROS_INFO("." );
		//publish the adc value every 50 milliseconds
		//since it takes that long for the sensor to stablize
		int r = 0;
		publish_ultrasound();

 
		nh.spinOnce();

		chThdSleepMilliseconds(1500);// Sleep for n milliseconds.
	}
	//	return 0;
}

#endif // ROSSERIAL



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  N O N  A T T I V I  									/////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region THREAD NON ATTIVI
#if 0
	//////////////////////////////////////////////////////////////////////////////////
	//  blinking LED       ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	/*
	#pragma region  Processo blinking LED
	// 64 byte stack beyond task switch and interrupt needs
	static THD_WORKING_AREA(waFlashLed, 64);
	static THD_FUNCTION(FlashLed, arg) {

	while (1) {
	// Turn LED on.
	LEDTOP_G_ON
	// Sleep for 50 milliseconds.
	chThdSleepMilliseconds(20);

	// Turn LED off.
	LEDTOP_G_OFF

	// Sleep for 150 milliseconds.
	chThdSleepMilliseconds(960);
	}
	}
	#pragma endregion
	*/


	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread   SCAN 
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waScan, 200);
static THD_FUNCTION(thdScan, arg) {

	while (!robot.status.operatingMode == MODE_AUTONOMOUS)
	{
		// Lock movements
		//chMtxLock(&mutexMotion);

		onCmdSonarScan(&cmdMMI);

		// Unlock data access.
		//chMtxUnlock(&mutexMotion);

		chThdSleepMilliseconds(1000);

	}
}




#endif // 0
#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region [CHIBIOS RTOS]

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

//uint16_t getFreeSram() {
//	uint8_t newVariable;
//	// heap is empty, use bss as start memory address 
//	if ((uint16_t)__brkval == 0)
//		return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
//	// use heap end as the start of the memory address 
//	else
//		return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
//};

void thdSetup() {
	// fill pool with msgObjArray array
	//for (size_t i = 0; i < MBOX_COUNT; i++) {
	//	chPoolFree(&memPool, &msgObjArray[i]);
	//}

	//chThdCreateStatic(waSafety, sizeof(waSafety), NORMALPRIO +10, thdSafety, NULL);
	chThdCreateStatic(waReadSensorsLR, sizeof(waReadSensorsLR), NORMALPRIO + 6, thdReadSensorsLR, NULL);
	chThdCreateStatic(waRos, sizeof(waRos), NORMALPRIO + 6, thdRos, NULL);
	chThdCreateStatic(waReadSensorsHR, sizeof(waReadSensorsHR), NORMALPRIO + 5, thdReadSensorsHR, NULL);

	//chThdCreateStatic(waSendStatusHR, sizeof(waSendStatusHR), NORMALPRIO +4, thdSendStatusHR, NULL);
	//chThdCreateStatic(waSendStatusLR, sizeof(waSendStatusLR), NORMALPRIO +4, thdSendStatusLR, NULL);
	//chThdCreateStatic(waPCcommands, sizeof(waPCcommands), NORMALPRIO + 3, thdPCcommands, NULL);
	//chThdCreateStatic(waMMIcommands, sizeof(waMMIcommands), NORMALPRIO + 3, thdMMIcommands, NULL);
	//chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 2, FlashLed, NULL);
	//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 1, ThreadMonitor, NULL);
	//	chThdCreateStatic(watestRotaryEncoder, sizeof(watestRotaryEncoder), NORMALPRIO + 2, testRotaryEncoder, NULL);
	MSG("Thread chibios avviati ..");
	while (1) {}

}
#pragma endregion

void setup_robot() {
	LEDTOP_R_ON	// Indica inizio SETUP Phase


	robot.initHW(); //disabilita i motori
	MSG("ROBOTCORE v0.2");
	WEBCAM_OFF
	WEBCAM_ON
	MSG3("Bat : ",robot.readBattChargeLevel(),"%");

	tone(Pin_LED_TOP_R, 2, 0);
	Wire.begin(); // default timeout 1000ms
	robot.initRobot();
	noTone(Pin_LED_TOP_R);
	LEDTOP_G_ON	// Indica inizio SETUP Phase


	MSG("ACCENDI I MOTORI");
	countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE
	LDS.init();
	//compass.begin();

	robot.initCompass(&compass);


}
// ########################################################################################
// ########################################################################################
//  S E T U P
// ########################################################################################
// ########################################################################################
void setup()
{
	SERIAL_ROS.begin(SERIAL_ROS_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);

	setup_robot();


	lampeggiaLed(Pin_LED_TOP_G, 2, 10);
	MSG("MOVE TO 330 DEG");
	// mi allineo parallelo al corridoio tra libreria e divano
	robot.goHeading(330, 5); //bloccante se non va il Compass

	LEDTOP_R_OFF
	MSG("avvio dei task...")
	chBegin(thdSetup);
}


void loop() {
	// not used
}



