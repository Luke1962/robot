// test di pubblicazione  dati LDS su topic /scandata
// derivato da TestRobotInOut
// verifica il funzionamento dei vari componenti HW
// no ROS
// usa MMI o Serial 


#define SIMULATION_ON 0
#define DEBUG 1


#pragma region ROBOT MACRO REGION

	//set  1 se viuoi fare i test senza l'oggetto Robot
	#define OPT_ROBOT 0

	//////////////////////////////////////////////////////////////////////////////////
	// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	#pragma region LIBRERIE DI BASE PER L'INTERFACCIAMENTO HW E CONFIGURAZIONE DEL SISTEMA   
	//#define delay(ms) chThdSleepMilliseconds(ms) 

	//#include <MyRobotLibs\dbg.h>
	#include <MyRobotLibs\dbgCore.h>


	#if OPT_ROBOT
		#include <robot.h>
		struct robot_c robot;	//was  struct robot_c robot;

		void setup_Robot() {
			robot.initRobot();

		}

	#else
		#include <MyRobotLibs\parameters.h>
		#include <MyRobotLibs\systemConfig.h>
		#include <MyRobotLibs\hw_config.h>
		#include <MyRobotModelSmall\MyRobotModelSmall.h>
		//using namespace MyRobotModelSmall;
		// MyRobotModelSmall::robotBaseSmallModel_c robot;	//was  struct robot_c robot;

		void setup_Robot() {
 
		}
	#endif // OPT_ROBOT



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
	#include <avr/interrupt.h>

	#include <Arduino.h>	//per AttachInterrupt


	#pragma endregion

	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CREAZIONE OGGETTI GLOBALI
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#pragma region CREAZIONE OGGETTI GLOBALI
	#if OPT_MPU6050
	// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
	// is used in I2Cdev.h
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include <Wire.h>
	#endif

	#include <MPU6050.h>
	//#include <MPU6050\MPU6050_6Axis_MotionApps20.h>
	//	#include <MPU6050\MPU6050.h> // not necessary if using MotionApps include file


	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
	// AD0 high = 0x69
	MPU6050 mpu;
	//MPU6050 mpu(0x69); // <-- use for AD0 high
	//// MPU control/status vars
	//bool dmpReady = false;  // set true if DMP init was successful
	//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	//uint16_t fifoCount;     // count of all bytes currently in FIFO
	//uint8_t fifoBuffer[64]; // FIFO storage buffer
	//						// orientation/motion vars
	//Quaternion q;           // [w, x, y, z]         quaternion container
	//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	//VectorFloat gravity;    // [x, y, z]            gravity vector
	//float euler[3];         // [psi, theta, phi]    Euler angle container
	//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	void setup_MPU() {
		while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
		{
			Serial.println(F("No MPU6050 sensor, check wiring!"));
			delay(500);
		}
		// If you have GY-86 or GY-87 module.
		// To access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode


		mpu.setI2CMasterModeEnabled(false);
		mpu.setI2CBypassEnabled(true);
		mpu.setSleepEnabled(false);
	}
	/*
	void setupMPU() {
	MSG("MPU Init ...");

	//mpu.initialize();
	mpu.begin();
	Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");
	// load and configure the DMP
	//		devStatus = mpu.dmpInitialize();
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setGyroOffsetY(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	}
	*/

	#endif // OPT_MPU6050


	#if OPT_COMPASS
	#include <Wire.h>
	//#include <compass\compass.h>
	//MyCompass_c myCompass;
	#include <HMC5883L.h>
	HMC5883L myCompass;

	#include <HMC5883L/HMC5883L.h>
	//HMC5883L compass;

	void setup_Compass() {
		// Initialize Initialize HMC5883L
		Serial.println("Initialize HMC5883L");
		//while (!myCompass.begin(2))
		while (!myCompass.begin())
		{
			Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
			delay(500);
		}

		// Set measurement range
		myCompass.setRange(HMC5883L_RANGE_1_3GA);

		// Set measurement mode
		myCompass.setMeasurementMode(HMC5883L_CONTINOUS);

		// Set data rate
		myCompass.setDataRate(HMC5883L_DATARATE_30HZ);

		// Set number of samples averaged
		myCompass.setSamples(HMC5883L_SAMPLES_8);

		// Set calibration offset. See HMC5883L_calibration.ino
		myCompass.setOffset(0, 0);

	}
	#endif // COMPASS

	#if OPT_SONAR
	#include <Newping\NewPing.h>
	NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
	#endif //

	#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	#include <Servo\src\Servo.h>
	#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)

	#include <PWM\PWM.h>
	Servo servoSonar;
	#endif

	#if OPT_GPS
	#include <TinyGPSplus\TinyGPS++.h> //deve essere incluso anche nel main

	TinyGPSPlus Gps;
	#endif

	#pragma region VL53L0X distanceSensor

	#if OPT_LDS
		//#include <Wire\Wire.h>
		#include <Wire.h>
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


		bool setup_LDS() {
			byte failCount = 0;
			bool initDone = false;
			while (!initDone && (failCount < 10))
			{
				MSG("LDS init...");
				if (LDS.init())
				{
					MSG("LDS OK;");
					initDone = true;
					return initDone;
				}
				else
				{
					MSG("LDS   ..FAIL;");
					delay(500);
					failCount++;
				}

			}
			return initDone;
		}

	#endif // OPT_LDS



	#if OPT_STEPPERLDS
		#include <Timer5/Timer5.h>
		#include <PinChangeInt\PinChangeInt.h>	//https://github.com/NicoHood/PinChangeInterrupt

		#include <MyStepper\myStepper.h>
		myStepper_c myLDSstepper(PIN_STEPPERLDS_CK, PIN_STEPPERLDS_ENABLE, PIN_STEPPERLDS_CW, PIN_STEPPERLDS_HOME, PIN_STEPPERLDS_END);

		byte ckState = 0;

		#define LDS_STEPPER_SPEED PI
		#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1
		#define LDSmicrostepsPerStep 2 // dato da combinazione di M1 e M2
		// Genera il clock per LDS Stepper
		ISR(timer5Event)
		{
			// Reset Timer1 (resetTimer1 should be the first operation for better timer precision)
			resetTimer5();
			// For a smaller and faster code, the line above could safely be replaced with a call
			// to the function resetTimer1Unsafe() as, despite its name, it IS safe to call
			// that function in here (interrupts are disabled)

			// Make sure to do your work as fast as possible, since interrupts are automatically
			// disabled when this event happens (refer to interrupts() and noInterrupts() for
			// more information on that)

			// Toggle led's state
			ckState ^= 1;
			//digitalWriteFast(13, ckState);

			digitalWriteFast(PIN_STEPPERLDS_CK, ckState);
		}

		// Commuta la direzione dello stepper
		void ISRstepperSwitchHome() {
			static unsigned long last_interrupt_timeHome = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				myLDSstepper.setHomePosition(true);
				myLDSstepper.setCW(false);  ///angle positive ccw
			//	myLDSstepper.goRadsPerSecond(2 * LDS_STEPPER_SPEED);
			}
			last_interrupt_timeHome = interrupt_time;
		}

		// Commuta la direzione dello stepper
		void ISRstepperSwitchEnd() {
			static unsigned long last_interrupt_timeEnd = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{

				myLDSstepper.setEndPosition(true);
				myLDSstepper.setCW(true);  ///.disable();
			//	myLDSstepper.goRadsPerSecond(LDS_STEPPER_SPEED);
			}
			last_interrupt_timeEnd = interrupt_time;
		}


		// versione pe Interrupt su Change
		void ISRldsH() {
			static unsigned long last_interrupt_timeHome = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				int x = digitalReadFast(PIN_STEPPERLDS_HOME);
				if (x == 0)
				{
					myLDSstepper.setHomePosition(true);
					myLDSstepper.setCW(true);  ///.disable();

				}
				else
				{
					myLDSstepper.setHomePosition(false);
					myLDSstepper.enable();

				}

			}
			last_interrupt_timeHome = interrupt_time;
		}
		void ISRldsE() {
			static unsigned long last_interrupt_timeEnd = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				int x = digitalReadFast(PIN_STEPPERLDS_END);
				if (x == 0)
				{
					//myLDSstepper.setHomePosition(true);
					myLDSstepper.setCW(false);  ///.disable();

				}
				else
				{
					//myLDSstepper.setHomePosition(false);
					myLDSstepper.enable();

				}

			}
			last_interrupt_timeEnd = interrupt_time;
		}


		void setup_StepperLDS(float speed = LDS_STEPPER_SPEED ) {
			pinMode(PIN_STEPPERLDS_HOME, INPUT_PULLUP);// open >+5 closed =gnd
			pinMode(PIN_STEPPERLDS_END, INPUT_PULLUP);// open >+5 closed =gnd
													  //attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, CHANGE);  // add more attachInterrupt code as required
													  //attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, CHANGE);  // add more attachInterrupt code as required

			attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, FALLING);  // add more attachInterrupt code as required
			attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, FALLING);  // add more attachInterrupt code as required
			sei();		
			
			
			myLDSstepper.goRadsPerSecond(speed);// start LDS stepper
		}

	#endif



	// per i test
	int targetAngle = 0;
	int freeDistCm = 0;
	char charVal[10];

	#pragma endregion


	// ////////////////////////////////////////////////////////////////////////////////////////////
	#if OPT_ENCODERS
	// dopo dichiarazione di robot
	// ########################################################################################
	// ########################################################################################
	// ENCODER ISR
	// opera solo se  targetEncoderThicks > 0 ed ogni ISR_MINMUM_INTERVAL_MSEC
	// ########################################################################################
	// ########################################################################################
	// Interrupt service routines for the right motor's quadrature encoder
	volatile bool interruptFlag = false;
	volatile uint32_t robotstatussensorsEncR = 0;
	volatile unsigned long isrCurrCallmSec = 0;
	volatile unsigned long isrLastCallmSec = 0;

	#define ISR_MINMUM_INTERVAL_MSEC 20  //deve essere < 30 = circa ROBOT_MOTOR_STEPS_PER_ENCODERTICK * ROBOT_MOTOR_CLOCK_microsecondMIN /1000 I= 12.5*2500/1000




	/*		void ISRencoder() //versione che usa targetEncoderThicks
	{
	if (robot.status.cmd.targetEncoderThicks > 0) //faccio lavorare l'ISR solo se targetEncoderThicks> 0
	{
	isrCurrCallmSec = millis();

	if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
	{
	//LEDTOP_B_ON
	isrLastCallmSec = isrCurrCallmSec;
	robot.status.sensors.EncR += 1;
	if (robot.status.sensors.EncR > robot.status.cmd.targetEncoderThicks)
	{
	//LEDTOP_G_ON
	robot.stop();
	//MOTORS_DISABLE
	robot.status.cmd.targetEncoderThicks = 0;

	}
	LEDTOP_B_OFF
	}

	}

	}
	*/

	// versione per comandi basati su cmd_vel
	// incrementa solo il contatore
	// chi legge deve poi  resettarlo e applicare le logiche per capire se
	// il movimento era linerare o rotatorio e  in quale direzione
	int32_t robotStatusSensorsencRcnt = 0;
	bool robotStatusSensorsEncR; //contatore
	bool robotStatusSensorsEncRprec;
	void ISRencoderRforTesting() {
		isrCurrCallmSec = millis();

		// trascorso tempo minimo?
		if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
		{
			//lettura encoder
			robotStatusSensorsEncR = digitalReadFast(Pin_EncRa);
			// cambiato lo stato ?
			if (robotStatusSensorsEncRprec != robotStatusSensorsEncR)
			{
				TOGGLEPIN(Pin_LED_TOP_B);
				isrLastCallmSec = isrCurrCallmSec;
				// incrementa
				robotStatusSensorsencRcnt += 1;
				// memorizzo lo stato corrente
				robotStatusSensorsEncRprec = robotStatusSensorsEncR;
			}
		}
	}
	void setup_EncodersForTesting() {
		pinMode(Pin_EncRa, INPUT);// open >+5 closed =gnd
								  // was 	attachInterrupt(digitalPinToInterrupt(Pin_EncRa), ISRencoderR, CHANGE);
		attachPinChangeInterrupt(Pin_EncRa, ISRencoderRforTesting, CHANGE);  // add more attachInterrupt code as required

	}

	#if OPT_ROBOT
	void readEncoders(float * dx, float * dth) { // rirorna la lettura degli encoder e li azzera
		MSG2("encRcnt ", robot.status.sensors.encRcnt);
		if (robot.status.cmd.commandDir == commandDir_e::GOF)
		{
			*dx = robot.status.sensors.encRcnt * ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_M2STEPS;
		}
		if (robot.status.cmd.commandDir == commandDir_e::GOB)
		{
			*dx = -robot.status.sensors.encRcnt* ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_M2STEPS;
			*dth = 0.0;
		}
		if (robot.status.cmd.commandDir == commandDir_e::GOR)
		{
			*dx = 0.0;
			*dth = -robot.status.sensors.encRcnt* ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_RAD2STEPS;
		}

		robot.status.sensors.encR = 0;
		robot.status.sensors.encL = 0;
	}
	void ISRencoderR() {
		isrCurrCallmSec = millis();

		// trascorso tempo minimo?
		if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
		{
			//lettura encoder
			robot.status.sensors.encR = digitalReadFast(Pin_EncRa);
			// cambiato il valore ?
			if (robot.status.sensors.encRprec != robot.status.sensors.encR)
			{
				TOGGLEPIN(Pin_LED_TOP_G);
				isrLastCallmSec = isrCurrCallmSec;
				// incrementa
				robot.status.sensors.encR += 1;
				robot.status.sensors.encRprec = robot.status.sensors.encR;
			}
		}
	}
	void setup_Encoders() {
		pinMode(Pin_EncRa, INPUT);// open >+5 closed =gnd
								  // was 	attachInterrupt(digitalPinToInterrupt(Pin_EncRa), ISRencoderR, CHANGE);
		attachPinChangeInterrupt(Pin_EncRa, ISRencoderR, CHANGE);  // add more attachInterrupt code as required

	}

	#else
	void readEncoders(float * dxBack, float * dth) { // rirorna la lettura degli encoder e li azzera
		MSG2("Enc. cnt", robotStatusSensorsencRcnt);
		//float ROBOT_MOTOR_STEPS_PER_ENCODERTICK = 12.5f;
		//float ROBOT_M2STEPS = 3536.7765f;
		*dxBack = robotStatusSensorsencRcnt* ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_M2STEPS;
	}
	#endif
	#endif

	#if OPT_ROBOT
	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CmdMessenger object to the default Serial port
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>
	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//------------------------------------------------------------------------------
	#endif // OPT_ROBOT






	#pragma endregion

	#pragma region Helper Functions: lampeggiaLed, countDown, ledSeq1
	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  FUNZIONI E UTILITIES GLOBALI
	// ////////////////////////////////////////////////////////////////////////////////////////////
	//void playSingleNote(int pin, int freq,int noteDuration) {
	//	tone(pin, freq, noteDuration);
	//	noTone(pin);
	//}




	// ////////////////////////////////////////////////////////////////////////////////////////////
	///  bloccante
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
	void ledSeq1(int t = 500) {
		LEDTOP_R_ON;	// Indica inizio SETUP Phase
		delay(t);
		LEDTOP_R_OFF;

		LEDTOP_G_ON;	// Indica inizio SETUP Phase
		delay(t);
		LEDTOP_G_OFF;

		LEDTOP_B_ON;	// Indica inizio SETUP Phase
		delay(t);
		LEDTOP_B_OFF;
		
		LEDTOP_ALL_OFF;
	}

	void MSG2F(char * c, float f) {
	#define CHARFLOATSIZE 7
	#define CHARFLOATDECS 4
		char charVal[CHARFLOATSIZE];  //temporarily holds data from vals
									  //4 is mininum width, 3 is precision; float value is copied onto buff
		dtostrf(f, CHARFLOATSIZE, CHARFLOATDECS, charVal);
		SERIAL_MMI.print("1,");
		SERIAL_MMI.print(c);
		SERIAL_MMI.print(charVal);
		SERIAL_MMI.println(F(";"));
	}
	#define sgn(x) ((x > 0) - (x < 0))

	// ////////////////////////////////////////////////////////////////////////////////////////////
	#pragma endregion

	// ########################################################################################
	// ########################################################################################
	//  S E T U P
	// ########################################################################################
	// ########################################################################################
	void setup_RobotHardware()
	{
		LEDTOP_R_ON	// Indica inizio SETUP Phase
		delay(1000);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE 5 ok

		#if OPT_ROBOT
			MSG("s.Robot")
			robot.initRobot(); //inizializza solo le variabili
			setup_Robot();
		#else
			initRobotBaseHW();
		#endif
		Wire.begin(); // default timeout 1000ms

		#if OPT_LDS

			#if OPT_ROBOT
				robot.initRobot(&LDS);
			#endif
			setup_LDS(); 	//robot.initLDS(&LDS);
			MSG("s.LDS")

		#endif
					/*
					InitTimersSafe(); //PWM.h initialize all timers except for 0, to save time keeping functions
					//sets the frequency for the specified pin
					bool success = SetPinFrequencySafe(Pin_LED_TOP_R, 2);

					//if the pin frequency was set successfully, turn pin 13 on
					if (success) {
					digitalWrite(Pin_LED_TOP_B, HIGH);
					pwmWrite(Pin_LED_TOP_R, 128); //set 128 to obtain 50%duty cyle
					SetPinFrequency(Pin_LED_TOP_R, 2); //setting the frequency to 10 Hz
					}
					*/


		#if OPT_ENCODERS
			MSG("s.ENC")
			#if OPT_ROBOT
				setup_Encoders();

			#else
				setup_EncodersForTesting();
			#endif
		#endif


		#if OPT_GPS
			MSG("s.GPS")
			#if OPT_ROBOT
				robot.initRobot(&Gps);
			#endif
		#endif

							//	noTone(Pin_LED_TOP_R);
		#if OPT_MPU6050
			MSG("s.MPU")
			setup_MPU();
		#endif
		#if OPT_COMPASS

			MSG("s.Compass")
			setup_Compass();
			//robot.initRobot(&compass);
			//robot.initCompass(&compass);//include compass.begin();
			//	robot.compassCalibrate();

		#endif


		#if OPT_STEPPERLDS
			MSG("s.StepperLDS")
				//robot.initRobot(&myLDSstepper);
				setup_StepperLDS(PI); //avvia lo stepper
		#endif




		#if OPT_SEROVOSONAR
			#if OPT_ROBOT
				robot.initRobot(&servoSonar);
			#endif
		#endif

		//LEDTOP_R_OFF
		//LEDTOP_G_ON


	}


 #pragma endregion




#pragma region ROS MACRO REGION

	/// ///////////////////////////////////////////////////////////////////////////////
	// ROS
	/// ///////////////////////////////////////////////////////////////////////////////

	#include <ros.h>
	#include <ros/time.h>
	#include <std_msgs/String.h>
	#include <sensor_msgs/Range.h> //ultrasound
	#include <tf/transform_broadcaster.h>
	#include <sensor_msgs/LaserScan.h>
	#include <geometry_msgs/Twist.h>  // cmd_vel
	ros::NodeHandle  nh;
	//--------------------------------
	#define ROS_INFO(s) nh.loginfo(s);

	//--------------------------------
	geometry_msgs::TransformStamped t;
	tf::TransformBroadcaster broadcaster;

	#pragma region ROS_CHATTER
		//--------------------------------
		std_msgs::String str_msg;
		ros::Publisher chatter("chatter", &str_msg);
		/// ///////////////////////////////////////////////////////////////////////////////
		void publish_chatter(char* charVal) {


			//str_msg.data = strcat("ultra sound:", charVal);
			str_msg.data = charVal;
			chatter.publish(&str_msg);
			nh.spinOnce();

		}

	#pragma endregion

#pragma region ROS_ULTRASOUND
		//--------------------------------
		sensor_msgs::Range rosmsg_range;
		char ultrasound_frameid[] = "ultrasound";

		ros::Publisher pub_range("ultrasound", &rosmsg_range);
		unsigned long range_time =0 ;



		void setup_ROS_Ultrasound() {
			rosmsg_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
			rosmsg_range.header.frame_id = ultrasound_frameid;
			rosmsg_range.field_of_view = 0.1;  // fake
			rosmsg_range.min_range = 0.0;
			rosmsg_range.max_range = 6.47;

		}

		void publish_range() {
			if (millis() >= range_time) {



				// ROS Pubblicazione messaggio -------------
				rosmsg_range.range = (float)LDS.readRangeSingleMillimeters()/1000;



				//4 is mininum width, 3 is precision; float value is copied onto buff
				dtostrf(rosmsg_range.range, 4, 3, charVal);
				publish_chatter(charVal);

				rosmsg_range.header.stamp = nh.now();
				pub_range.publish(&rosmsg_range);


				range_time = millis() + 500;
				publish_range();
				nh.spinOnce();

			}

		}

#pragma endregion

	//--------------------------------

	#pragma region ROS_CMD_VEL
		double x = 1.0;
		double y = 0.0;
		double theta = 1.57;
		double g_req_angular_vel_z = 0;
		double g_req_linear_vel_x = 0;
		unsigned long g_prev_command_time = 0;

		geometry_msgs::Twist msg;

		void commandCallback(const geometry_msgs::Twist& cmd_vel);
		ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
		void commandCallback(const geometry_msgs::Twist& cmd_vel)
		{
			//callback function every time linear and angular speed is received from 'cmd_vel' topic
			//this callback function receives cmd_msg object where linear and angular speed are stored
			g_req_linear_vel_x = cmd_vel.linear.x;
			g_req_angular_vel_z = cmd_vel.angular.z;

			g_prev_command_time = millis();

			theta += cmd_vel.angular.z;
			if (theta > 3.14)    theta = -3.14;
			x += cos(theta)*cmd_vel.linear.x *0.1;
			y += sin(theta)*cmd_vel.linear.x *0.1;

			if (cmd_vel.angular.z != 0.0)
			{

				int speed = (int)(cmd_vel.angular.z * ROBOT_MOTOR_STEPS_PER_RADIANT);
				if (cmd_vel.angular.z > 0.0)
				{
					//robot.go(commandDir_e::GOR, speed);
				}
				else
				{
					//robot.go(commandDir_e::GOL, speed);

				}
			}
			else // comando lineare
			{
				int speed = (int)(cmd_vel.angular.z * ROBOT_MOTOR_STEPS_PER_RADIANT);
				if (cmd_vel.linear.x > 0.0)
				{
					//robot.go(commandDir_e::GOF, speed);
				}
				else
				{
					//robot.go(commandDir_e::GOB, speed);

				}

			}

			ROS_INFO("cmd_vel");
		}

	#pragma endregion

	//--------------------------------
	ros::Publisher speech("/rp/state_externalization/vocal_message", &str_msg);


	// laser data---------------

	//#define LDSsamples 5
	//#define LDSspeed PI/LDSmicrostepsPerStep		// PI rad/sec = 180°/sec 
	//#define SCAN_ANGLE_MIN  -PI / 2;//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	//float f_angle_max = PI / 2;
	//float f_angle_increment = PI / LDSsamples;  // (f_angle_max - SCAN_ANGLE_MIN)/ LDSsamples 
	//float f_time_increment;
	//float f_scan_time = 2 * LDSmicrostepsPerStep; // LDSmicrostepsPerStep* 2* (f_angle_max-SCAN_ANGLE_MIN )/LDSspeed
	//float f_range_min = 0.02;
	//float f_range_max = 2.0;
	//float ranges[LDSsamples]; // max of 30 measurements
	//float f_intensities[LDSsamples];
	//------------------------



#pragma region ROS_SCAN
	#define LDSmicrostepsPerStep 2

	//--------------------------------
	sensor_msgs::LaserScan scan_msg;;
	ros::Publisher pub_Laser("scan", &scan_msg);

	#define LDSsamples 20
	#define LDSspeed PI		// PI rad/sec = 180°/sec 
	#define SCAN_ANGLE PI		//SCAN_ANGLE_MAX - SCAN_ANGLE_MIN
	#define SCAN_ANGLE_MIN  -SCAN_ANGLE/2	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	#define SCAN_ANGLE_MAX   SCAN_ANGLE/2

	#define SCAN_ANGLE_INCREMENT  SCAN_ANGLE / LDSsamples;  // (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN)/ LDSsamples 
	#define SCAN_TIME  16 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX-SCAN_ANGLE_MIN )/LDSspeed
#define SCAN_TIME_INCREMENT_MS  LDS_STEPPER_SPEED/SCAN_ANGLE/LDSsamples
#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667
	#define	SCAN_RANGE_MIN 0.2f
	#define	SCAN_RANGE_MAX 5.0f

//	float ranges[LDSsamples]; // max of 30 measurements
	float intensities[LDSsamples]; // buffer 
	//------------------------
	float *ranges = new float[LDSsamples];
	unsigned long scan_time;


	void setup_ROS_LaserScan() {
		nh.advertise(pub_Laser);


		scan_msg.ranges_length = LDSsamples;
		scan_msg.intensities_length = LDSsamples;

		// create the test data
		for (int z = 0; z < LDSsamples; z++)
		{
			ranges[z] = 1.0;
			intensities[z] = 1.0;
		}
	}

	void publish_laserscan() {


		//if (millis() > scan_time)
		if (true)
		{
			// imposta la velocità


			scan_msg.header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			scan_msg.angle_min = -SCAN_ANGLE / 2;
			scan_msg.angle_max =  SCAN_ANGLE / 2;
			scan_msg.angle_increment = (float) SCAN_ANGLE / LDSsamples;
			scan_msg.scan_time = 3.91 ;	// tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE / LDSspeed ;
			scan_msg.time_increment =  scan_msg.scan_time/ (float)(LDSsamples -1) ;	//(1 / laser_frequency) / (num_readings);
			scan_msg.range_min = SCAN_RANGE_MIN;
			scan_msg.range_max = SCAN_RANGE_MAX;

			MSG2F("t_inc s.", scan_msg.time_increment);
			// attende che arrivi in home
			MSG("wait home...");
			while (!myLDSstepper.isHomePosition()) { delay(10); }

 
			scan_msg.header.stamp = nh.now();

			// acquisisce le distanze
			MSG("acquiring...");
			ROS_INFO("acquiring...")
			LASER_ON
				uint32_t dt,t, acqTime;
			int i = 0;

			LEDTOP_G_ON
			dt = (uint32_t)(1000 * scan_msg.time_increment);
			MSG2("dt ", dt);

			while ( i < LDSsamples)
			{
				t = millis();

				//acquisisco (36 - 37 mSec richiesti)
				ranges[i] = (float)LDS.readRangeSingleMillimeters()/1000;
				if (ranges[i] > 1)
				{
					// uso il sonar
					ranges[i] = (float)sonar.ping_cm()/100;
				}
				//MSG2F("d.", ranges[i]);
				//MSG2("i:",i)
				acqTime = millis() -t;
				MSG2("acqTime:", acqTime )
				i++; //incremento il puntatore

				//attendo time_increment
				//while (millis() < (t + 1000*scan_msg.time_increment)) delay(1);

				delay(dt-50); 
				// prove empiriche----------------------------
				//con  scan_time = PI
				//18 trovato sperimentalmente ok 
				// con scantime  = 3.91
				// 25 è poco (finisce dopo EndPosition)
				// se metto  MSG2("acqTime:", acqTime ) , 50  è ok


			}
			LASER_OFF
				//LASER_OFF
			LEDTOP_G_OFF

			scan_msg.ranges = ranges;
			scan_msg.intensities = intensities;

			// pubblica
			MSG("publish /scan");
			pub_Laser.publish(&scan_msg);



			//-------------------------------------------------------------
			// scansione in senso opposto
			//-------------------------------------------------------------


			while (!myLDSstepper.isEndPosition()) { delay(1); }
			// scansione in senso CW
		
			i = LDSsamples - 1;

			LEDTOP_B_ON
				while (i >= 0)
				{
					//t = millis();

					//acquisisco (36 - 37 mSec richiesti)
					ranges[i] = (float)LDS.readRangeSingleMillimeters() / 1000;
					if (ranges[i] > 1)
					{
						// uso il sonar
						ranges[i] = (float)sonar.ping_cm() / 100;
					}
 					i--; //decremento il puntatore
 					delay(dt - 50);
 

				}

			LEDTOP_B_OFF


			MSG("end acquiring...");
			ROS_INFO("end acquiring...")

			//for (unsigned int i = 0; i < LDSsamples; ++i) {
			//	ranges[i] = (float)LDS.readRangeSingleMillimeters()/1000;
			//	delay(SCAN_TIME_INCREMENT_MS);
			//}


			scan_msg.ranges = ranges;
			scan_msg.intensities = intensities;

			// pubblica
			MSG("publish /scan");
			pub_Laser.publish(&scan_msg);

			//imposta la prossima scansione
			scan_time = millis() + 2000;
			MSG("-- x --");

		}

	}
 
#pragma endregion


	//temporarily holds data from vals
	//	char charVal[10];

	char hello[13] = "hello world!";

	char base_link[] = "/base_link";
	char odom[] = "/odom";

	unsigned long tf_time;


	float move1;
	float move2;

	const int adc_pin = 0;




	//-----------------------------------------------------------------

/*

	#ifdef SIMULATION_ON

		void publish_range() {
			if (millis() >= range_time) {



				// ROS Pubblicazione messaggio -------------
				rosmsg_range.range = getRange_Ultrasound_Simulated(5);



				//4 is mininum width, 3 is precision; float value is copied onto buff
				dtostrf(rosmsg_range.range, 4, 3, charVal);
				publish_chatter(charVal);

				rosmsg_range.header.stamp = nh.now();
				pub_range.publish(&rosmsg_range);


				range_time = millis() + 500;
				publish_range();
				nh.spinOnce();

			}

		}


		//-----------------------------------------------------------------
		// elabora il comando di velocità in ricezione 

		float getRange_Ultrasound_Simulated(int pin_num) {
			int val = 0;
			for (int i = 0; i < 4; i++) val += analogRead(pin_num);
			float range = val;
			return range / 322.519685;   // (0.0124023437 /4) ; //cvt to meters
		}


		void publish_tf(commandDir_e commandDir, uint32_t stepsDone) {  //versione simulata
			if (millis() >= tf_time) {
				// drive in a circle---------
				double dx = 0.02;
				double dtheta = 0.18;
				x += cos(theta)*dx*0.1;
				y += sin(theta)*dx*0.1;
				theta += dtheta*0.1;
				if (theta > 3.14)
					theta = -3.14;
				//---------------------------
				t.header.frame_id = odom;
				t.child_frame_id = base_link;

				t.transform.translation.x = 0.0;
				if (commandDir == commandDir_e::GOF)
				{
					t.transform.translation.x =  stepsDone / ROBOT_MOTOR_STEPS_PER_CM / 100;

				}
				if (commandDir == commandDir_e::GOB)
				{
					t.transform.translation.x = - stepsDone / ROBOT_MOTOR_STEPS_PER_CM / 100;

				}
				t.transform.rotation.x = 0.0;
				t.transform.rotation.y = 0.0;
				t.transform.rotation.z = 0.0;
				if (commandDir == commandDir_e::GOR)
				{
					t.transform.rotation.z = DEG_TO_RAD*  stepsDone / ROBOT_MOTOR_STEPS_PER_DEG;
				}
				if (commandDir == commandDir_e::GOL)
				{
					t.transform.rotation.z = -DEG_TO_RAD*  stepsDone / ROBOT_MOTOR_STEPS_PER_DEG;
				}

				t.transform.rotation.w = 1.0;
				t.header.stamp = nh.now();
				broadcaster.sendTransform(t);

				tf_time = millis() + 1000;
				nh.spinOnce();

				stepsDone = 0;
			}
		}

	#else

	//-----------------------------------------------------------------
	// elabora il comando di velocità in ricezione 
	void commandCallback(const geometry_msgs::Twist& cmd_vel)
	{
		//callback function every time linear and angular speed is received from 'cmd_vel' topic
		//this callback function receives cmd_msg object where linear and angular speed are stored

		if (cmd_vel.angular.z != 0.0)
		{

			int speed = (int)(cmd_vel.angular.z * ROBOT_MOTOR_STEPS_PER_RADIANT);
			if (cmd_vel.angular.z > 0.0)
			{
				robot.go(commandDir_e::GOR, speed);
			}
			else
			{
				robot.go(commandDir_e::GOL, speed);

			}
		}
		else // comando lineare
		{
			int speed = (int)(cmd_vel.angular.z * ROBOT_MOTOR_STEPS_PER_RADIANT);
			if (cmd_vel.linear.x > 0.0)
			{
				robot.go(commandDir_e::GOF, speed);
			}
			else
			{
				robot.go(commandDir_e::GOB, speed);

			}

		}

		ROS_INFO("cmd_vel");
	}

	void publish_range() {
		if (millis() >= range_time) {



			// ROS Pubblicazione messaggio -------------
			rosmsg_range.range = robot.getLDSDistance();



			//4 is mininum width, 3 is precision; float value is copied onto buff
			dtostrf(rosmsg_range.range, 4, 3, charVal);
			publish_chatter(charVal);

			rosmsg_range.header.stamp = nh.now();
			pub_range.publish(&rosmsg_range);


			range_time = millis() + 500;
			publish_range();
			nh.spinOnce();

		}

	}


	void publish_tf() {
		if (millis() >= tf_time) {
			t.header.frame_id = odom;
			t.child_frame_id = base_link;

			t.transform.translation.x = 0.0;
			if (robot.status.cmd.commandDir == commandDir_e::GOF)
			{
				t.transform.translation.x = robot.status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_CM / 100;

			}
			if (robot.status.cmd.commandDir == commandDir_e::GOB)
			{
				t.transform.translation.x = -robot.status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_CM / 100;

			}
			t.transform.rotation.x = 0.0;
			t.transform.rotation.y = 0.0;
			t.transform.rotation.z = 0.0;
			if (robot.status.cmd.commandDir == commandDir_e::GOR)
			{
				t.transform.rotation.z = DEG_TO_RAD* robot.status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_DEG;
			}
			if (robot.status.cmd.commandDir == commandDir_e::GOL)
			{
				t.transform.rotation.z = -DEG_TO_RAD* robot.status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_DEG;
			}

			t.transform.rotation.w = 1.0;
			t.header.stamp = nh.now();
			broadcaster.sendTransform(t);

			tf_time = millis() + 1000;
			nh.spinOnce();

			robot.status.cmd.stepsDone = 0;
		}
	}
	#endif

	*/
	

	#pragma region Setup_ROS

		void setup_ROS() {

			nh.initNode();
			//broadcaster.init(nh);

			nh.advertise(chatter);

			//setup_ROS_Ultrasound();
			//nh.advertise(pub_range);

			setup_ROS_LaserScan();
			
		}



	#pragma endregion	// Setup_ROS

#pragma endregion ROS 









void setup()
{

	LEDTOP_R_OFF
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);

	ledSeq1(500);
	MSG("=== TestRosLaserScan ===")


	MSG("s.ROBOT")
	setup_RobotHardware();


	MSG("s.ROS")
	setup_ROS();

	LEDTOP_R_OFF
 
}


void loop()
{

	//publish_range();
	//ROS_INFO("Scanning");

	publish_laserscan(); // acquisisce le distanze...


	nh.spinOnce();  //elabora eventuali callBack

//	TOGGLEPIN(Pin_LED_TOP_B)
	//delay(500);
 

}

