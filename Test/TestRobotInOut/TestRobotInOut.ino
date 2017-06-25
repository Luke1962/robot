// COMANDI PER TEST: 
//AVANTI 20 E INDIETRO 20: 
//19,20;19,-20;


//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
//#define delay(ms) chThdSleepMilliseconds(ms) 

//#include <MyRobotLibs\dbg.h>
#include <MyRobotLibs\dbgCore.h>

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
/// ///////////////////////////////////////////////////////////////////////////////
// ROS
/// ///////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <ros/duration.h>
#include <ros/time.h> //non serve
#include <ros/msg.h>
#include <ros_lib\sensor_msgs\Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>


#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>
//#include <TinyGPSplus/TinyGPS++.h>
//#include <StackArray.h>
#include <avr/interrupt.h>


#include <Arduino.h>	//per AttachInterrupt



#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
	#if OPT_MPU6050
		#include <I2Cdev\I2Cdev.h>

		#include <MPU6050\MPU6050_6Axis_MotionApps20.h>
		//	#include <MPU6050\MPU6050.h> // not necessary if using MotionApps include file

		// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
		// is used in I2Cdev.h
		#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
			#include "Wire.h"
		#endif

		// class default I2C address is 0x68
		// specific I2C addresses may be passed as a parameter here
		// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
		// AD0 high = 0x69
		MPU6050 mpu;
		//MPU6050 mpu(0x69); // <-- use for AD0 high


		// MPU control/status vars
		bool dmpReady = false;  // set true if DMP init was successful
		uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
		uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
		uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
		uint16_t fifoCount;     // count of all bytes currently in FIFO
		uint8_t fifoBuffer[64]; // FIFO storage buffer

								// orientation/motion vars
		Quaternion q;           // [w, x, y, z]         quaternion container
		VectorInt16 aa;         // [x, y, z]            accel sensor measurements
		VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
		VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
		VectorFloat gravity;    // [x, y, z]            gravity vector
		float euler[3];         // [psi, theta, phi]    Euler angle container
		float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

		void setupMPU() {
			MSG("MPU Init ...");

			mpu.initialize();
			Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");
			// load and configure the DMP
			//		devStatus = mpu.dmpInitialize();
			// supply your own gyro offsets here, scaled for min sensitivity
			mpu.setXGyroOffset(220);
			mpu.setYGyroOffset(76);
			mpu.setZGyroOffset(-85);
			mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

		}

	#endif // OPT_MPU6050


	#if OPT_COMPASS
		#include <Wire\Wire.h>
		#include <compass\compass.h>
		MyCompass_c compass;
		//#include <Adafruit_Sensor\Adafruit_Sensor.h> //richiesto dalla liberia compass Adafruit_HMC5883_U
		//#include <HMC5883L\HMC5883L.h>
		//HMC5883L compass;
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



	#if OPT_STEPPERLDS

		#include <MyStepper\myStepper.h>
		myStepper_c myLDSstepper(PIN_STEPPERLDS_CK, PIN_STEPPERLDS_ENABLE, PIN_STEPPERLDS_CW, PIN_STEPPERLDS_STOP);
	#endif


	// per i test
		int targetAngle = 0;
		int freeDistCm = 0;
		char charVal[10];

	#pragma endregion

	#include <robot.h>
	struct robot_c robot;	//was  struct robot_c robot;


	// ////////////////////////////////////////////////////////////////////////////////////////////


	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CmdMessenger object to the default Serial port
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>
	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//------------------------------------------------------------------------------




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

	#define ISR_MINMUM_INTERVAL_MSEC 25  // 30 = circa ROBOT_MOTOR_STEPS_PER_ENCODERTICK * ROBOT_MOTOR_STEPS_PER_ENCODERTICK
	void ISRencoder()
	{
		if (robot.status.cmd.targetEncoderThicks > 0) //faccio lavorare l'ISR solo se targetEncoderThicks> 0
		{
			isrCurrCallmSec = millis();

			if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
			{
				LEDTOP_B_ON
					isrLastCallmSec = isrCurrCallmSec;
				robot.status.sensors.EncR += 1;
				if (robot.status.sensors.EncR > robot.status.cmd.targetEncoderThicks)
				{
					LEDTOP_G_ON
						robot.stop();
					//MOTORS_DISABLE
					robot.status.cmd.targetEncoderThicks = 0;

				}
				LEDTOP_B_OFF
			}

		}

	}



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
	void ledSeq1() {
		LEDTOP_R_ON	// Indica inizio SETUP Phase
			delay(500);
		LEDTOP_R_OFF

		LEDTOP_G_ON	// Indica inizio SETUP Phase
			delay(500);
		LEDTOP_G_OFF

		LEDTOP_B_ON	// Indica inizio SETUP Phase
			delay(500);
		LEDTOP_B_OFF

	}
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma endregion






#pragma region Test Functions
	void testMPU(int iterations = 5) {
		int16_t ax, ay, az;
		int16_t gx, gy, gz;

		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		// these methods (and a few others) are also available
		//accelgyro.getAcceleration(&ax, &ay, &az);
		//accelgyro.getRotation(&gx, &gy, &gz);

		// display tab-separated accel/gyro x/y/z values
		MSG("Test Compass")
		for (size_t i = 0; i < iterations; i++)
		{
			MSG2("ax:", ax);
			MSG2("ay:", ay);
			MSG2("az:", az);
			MSG2("gx:", gx);
			MSG2("gy:", gy);
			MSG2("gz:", gz);


			Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
			Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
			Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
			Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
			Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
			Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));

		}

	}
	void testMPU2() {
		MSG("TST MPU")
			long tStart = millis();
		while (millis() - tStart < 10000) {
			if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
				// reset so we can continue cleanly
				mpu.resetFIFO();
				MSG("FIFO overflow!");

				// otherwise, check for DMP data ready interrupt (this should happen frequently)
			}
			else if (mpuIntStatus & 0x02) {
				// wait for correct available data length, should be a VERY short wait
				while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

				// read a packet from FIFO
				mpu.getFIFOBytes(fifoBuffer, packetSize);

				// track FIFO count here in case there is > 1 packet available
				// (this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize;

				mpu.dmpGetQuaternion(&q, fifoBuffer);
				dtostrf(q.z, 4, 3, charVal);	//float to char array
				MSG("MPU")
					Serial1.print("1,quat ");
				Serial1.print(q.w);
				Serial1.print(",");
				Serial1.print(q.x);
				Serial1.print(",");
				Serial1.print(q.y);
				Serial1.print(",");
				Serial1.print(q.z);
				Serial1.println(";");

			}

			robot.readBumpers();

		}

	}
	void testLDS(int iterations= 5) {
		MSG("Test Compass")
		for (size_t i = 0; i < iterations; i++)
		{
			MSG2("LDS cm: ", robot.getLDSDistanceCm());
			delay(500);
		}

	}
	void testCompass(int iterations= 5) {
		MSG("Test Compass")
		for (size_t i = 0; i < iterations; i++)
		{
			MSG2("   alfa: ", robot.readCompassDeg());
			delay(500);
		}

	}

	void testGPS() {
		MSG("Test GPS")
		
}

	void testGoScan() {
		// test LDS
		while (true) {
			int angle = robot.goCWAndScanDeg(360);
			delay(1000);
			robot.goHeading(angle, 3);



			delay(5000);

		}

	}
	void testMotors() {
		MSG("MOTION TEST");
		countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE

		MSG("MOVE FW 5cm");
		robot.moveCm(10);
		robot.moveCm(-10);
		robot.rotateDeg(45);
		robot.rotateDeg(-90);
		robot.rotateDeg(45);
		MSG("END Testing robot.moveCm()");
	}
	void testGoHeading(int Heading= 3) {

		MSG2("## goHeading :", Heading);
		robot.stop();
		//MOTORS_DISABLE
		robot.goHeading(Heading, 5);

}
#pragma endregion


// ########################################################################################
// ########################################################################################
//  S E T U P
// ########################################################################################
// ########################################################################################
void setupRobot()
{
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	pinMode(Pin_LED_TOP_G, OUTPUT);
	ledSeq1();
	//lampeggiaLed(Pin_LED_TOP_R, 3, 5);
	//lampeggiaLed(Pin_LED_TOP_G, 3, 5);
	//lampeggiaLed(Pin_LED_TOP_B, 3, 5);
	LEDTOP_R_ON	// Indica inizio SETUP Phase
	MOTORS_DISABLE
	attachInterrupt(digitalPinToInterrupt(3), ISRencoder, CHANGE);

	InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions
					  //sets the frequency for the specified pin
					  //bool success = SetPinFrequencySafe(Pin_LED_TOP_R, 2);

					  ////if the pin frequency was set successfully, turn pin 13 on
					  //if (success) {
					  //	digitalWrite(Pin_LED_TOP_B, HIGH);
					  //	pwmWrite(Pin_LED_TOP_R, 128); //set 128 to obtain 50%duty cyle
					  //	SetPinFrequency(Pin_LED_TOP_R, 2); //setting the frequency to 10 Hz
					  //}

	countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE
	Wire.begin(); // default timeout 1000ms


	#if OPT_MPU6050

		setupMPU();
	#endif

	#if OPT_GPS
		robot.initRobot(&Gps);
	#endif
	#if OPT_LDS
		robot.initRobot(&LDS);
		robot.initLDS(&LDS);
	#endif
	#if OPT_COMPASS
		robot.initRobot(&compass);
		robot.initCompass(&compass);//include compass.begin();
	#endif
	#if OPT_STEPPERLDS
		robot.initRobot(&myLDSstepper);
	#endif

	#if OPT_SEROVOSONAR
		robot.initRobot(&servoSonar);
	#endif


	robot.initRobot();

	//	noTone(Pin_LED_TOP_R);


}
	
void setup()
{
	setupRobot();
	//	robot.compassCalibrate();

	LEDTOP_R_ON
	//robot.initHW(); //disabilita i motori
					//	tone(Pin_LED_TOP_R, 2, 0);
	MSG("####[  Test Robot In Out v1  ]####");
	WEBCAM_OFF
	WEBCAM_ON
	MSG3("Bat : ", robot.readBattChargeLevel(), "%");

	MSG2("A0 : ", analogRead(A0));
	MSG2("A1 : ", analogRead(A1));
	MSG2("A1 : ", analogRead(A2));



	MSG("ACCENDI I MOTORI");
	countDown(5);


	// test di obstacleFree()		 
	testMotors();

	//testGoHeading(90);
#if 0
	//SERIAL_MSG.println(F("1,running IBIT...;"));
	//robot.runIBIT(300);


	// posiziona il servo al centro
			MSG("Posiziono Servo al centro...")
				//servoSonar.attach(Pin_ServoSonarPWM);
				robot.ServoAtPos(10); delay(1000);
			robot.ServoAtPos(90); delay(1000);
			robot.ServoAtPos(160); delay(1000);


#endif // 0

	// ########################################################################################
	// ########################################################################################
	//  TEST OPZIONALI
	// ########################################################################################
	// ########################################################################################
	#if 0
	#pragma region TEST OPZIONALI PRIMA DELL'AVVIO DELL'OS
	#define TEST_SONAR 1
	#if TEST_SONAR
		MSG("SONAR ONLY TEST..");
		for (size_t i = 0; i < 5; i++)
		//		while (true)
		{
			TOGGLEPIN(Pin_LaserOn)
				MSG3("Ping: ", robot.getLDSDistanceCm(), "cm");
			//MSG3("Ping: ",sonar.ping_cm(),"cm");
			delay(300);
		}
	#endif


	#define TEST_LDS 1

	#if TEST_LDS
		MSG("LDS init...");
		//Wire.begin();

		bool initDone = false;
		while (!initDone)
		{
			if (LDS.init())
			{
				MSG("LDS..OK;");
				initDone = true;
			}
			else
			{
				MSG("LDS..FAIL;");
				delay(300);
			}

		}
		delay(2000);
		LDS.setTimeout(500);

		#if defined LONG_RANGE
			// lower the return signal rate limit (default is 0.25 MCPS)
			LDS.setSignalRateLimit(0.1);
			// increase laser pulse periods (defaults are 14 and 10 PCLKs)
			LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
			LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
		#endif
		#if defined HIGH_SPEED
			// reduce timing budget to 20 ms (default is about 33 ms)
			LDS.setMeasurementTimingBudget(20000);
		#elif defined HIGH_ACCURACY
			// increase timing budget to 200 ms
			LDS.setMeasurementTimingBudget(200000);
		#endif

		//Eseguo le misure di prova------------------------------
		uint16_t d = 0;

		for (size_t i = 0; i < 5; i++)
		{
			LASER_ON
			d = LDS.readRangeSingleMillimeters();	//robot.getLDSDistanceCm()
			if (!LDS.timeoutOccurred()) {
				MSG3("LDS :", d, "mm");
			}
			else
			{
				MSG("LDS TIMEOUT ");
				Wire.begin();
				LDS.init();

			}
			LASER_OFF

			delay(500);
		}
			//---------------------------------------------------------

//		MSG("LDS  TEST END");

		#endif


	#pragma region test go()
		MSG("rotateUntilMinFreeDist..")
		robot.status.sensors.sonarEchos[90] = robot.goCWUntilMinFreeDist(200);
		MSG2("dist found cm:", robot.status.sensors.sonarEchos[90]);
	#if 0
		// RUOTA con velocità crescente 1 secondo ogni step
		// Attenzione !! Se presente un altro tone() non funziona
		robot.stop();
		MSG("TEST robot.go()..")
			MSG("PLEASE SWITCH MOTORS ON")
			MSG("DISCONNECT CABLES")
			MSG("Start test in 10 secondi...")
			countDown(10);
		int ck = ROBOT_MOTOR_CLOCK_microsecondMAX;
		bool cont = true;
		while (cont)
		{
			MSG2("Current ck:", ck)
				robot.go(commandDir_e::GOR, ck);
			delay(1000);
			ck -= 200;
			if (ck < ROBOT_MOTOR_CLOCK_microsecondMIN)
			{
				//torno indietro per 3 secondi a velocità media
				robot.stop();
				robot.go(commandDir_e::GOL, robotSpeed_e::MEDIUM);
				delay(3000);
				cont = false;
			}
		}

		MSG("Fine test......")
			robot.stop();
		//while (true) {}

	#endif // 0

	#pragma endregion


	#pragma region I2C Scanner
	#if 0


		byte error, address;
		int nDevices;

		Serial.println("Scanning...");

		nDevices = 0;
		for (address = 1; address < 127; address++)
		{
			// The i2c_scanner uses the return value of
			// the Write.endTransmisstion to see if
			// a device did acknowledge to the address.
			error = I2c.write(address, 0);


			if (error == 0)
			{
				Serial.print("I2C device found at address 0x");
				if (address < 16)
					Serial.print("0");
				Serial.print(address, HEX);
				Serial.println("  !");

				nDevices++;
			}
			else if (error == 4)
			{
				Serial.print("Unknow error at address 0x");
				if (address < 16)
					Serial.print("0");
				Serial.println(address, HEX);
			}
		}
		if (nDevices == 0)
			Serial.println("No I2C devices found\n");
		else
			Serial.println("done\n");

		delay(5000);           // wait 5 seconds for next scan

	#endif // 0
	#pragma endregion

	#define TEST_GPS 0
	#if TEST_GPS
	//CHIAVARI LAT	44.326953 LONG 9.289679

	#endif // TEST_SPEECH

	#define TEST_SPEECH 0
	#if TEST_SPEECH
		SERIAL_MSG.println(F("1,test Speak;"));
		SERIAL_MMI.println(F("28,CIAO CIAO;"));

	#endif // TEST_SPEECH

	#define TEST_IRPROXY 0
	#if TEST_IRPROXY	//test ixproxy
		SERIAL_MSG.print(F("1,testing ixproxy...;"));
		while (true)
		{
			robot.readSensors();
			if (robot.status.sensors.irproxy.fw != robot.statusOld.sensors.irproxy.fw)
			{
				SERIAL_MSG.println(F("1,irproxy.fw changed;"));
				TOGGLEPIN(Pin_LED_TOP_B);

			}
			else if (robot.status.sensors.irproxy.fwHL != robot.statusOld.sensors.irproxy.fwHL)
			{
				SERIAL_MSG.println(F("1,irproxy.fwHl changed;"));
				TOGGLEPIN(Pin_LED_TOP_G);

			}
		}

	#endif // 0//test ixproxy

	#define TEST_OBST 0
	#if TEST_OBST		// test di obstacleFree()
		SERIAL_MSG.print(F("1,testing robot.isObstacleFree()...;"));
		bool val = false;
		bool oldVal = false;
		robot.status.cmd.commandDir = GOF;
		while (val)
		{
			val = robot.obstacleFree();
			if (oldVal != val)
			{
				TOGGLEPIN(Pin_LED_TOP_B);
				SERIAL_MSG.println("1,robot.isObstacleFree() changed;");
				oldVal = val;
			}
		}
	#endif // 1

	#define TEST_MOTION 0
	#if TEST_MOTION		// test di obstacleFree()



		attachInterrupt(digitalPinToInterrupt(Pin_EncRa), ISRencoder, LOW);
		interrupts();

		while (true)
		{
			MSG2("stp:", robot.status.cmd.stepsDone);

			delay(1000);
		}

		MSG("MOTION TEST");
		countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE

		MSG("MOVE FW 5cm");
		robot.moveCm(10);
		MSG("MOVE BK 5cm");
		robot.moveCm(-10);




		MSG("Testing robot.moveCm()...Attiva motori");
		countDown(8);
		robot.moveCm(10);
		robot.moveCm(-10);
		robot.rotateDeg(10);
		robot.rotateDeg(-20);
		robot.rotateDeg(10);
		MSG("END Testing robot.moveCm()");

		//SERIAL_MSG.print("testing robot.moveCm()...");
		//int cmFatti = 0;
		//#define DIST 30
		//while (!Serial.available())
		//{
		//	cmFatti = robot.moveCm(DIST);
		//	if (cmFatti <= DIST)
		//	{
		//		TOGGLEPIN(Pin_LED_TOP_R);
		//		SERIAL_MSG.print("OSTACOLO DOPO CM:"); SERIAL_MSG.println(cmFatti);
		//		robot.moveCm(-cmFatti);
		//	}
		//	else
		//	{
		//		TOGGLEPIN(Pin_LED_TOP_G);

		//	}
		//}
	#endif // 1



	#define TEST_COMPASS 1
	#if TEST_COMPASS		//TEST_COMPASS  
		robot.readSensorsHR(); // legge sensori
		MSG("TEST_COMPASS. Accendere i motori...");
		// Initialise the sensor 

		robot.stop();
		compass.begin(2);
		int DestAngle = 0; // nord
		int currAngle = 0;
		int error = 0;

		//misuro
		currAngle = compass.getBearing();
		error = currAngle - DestAngle;

		MSG("Ora punto a nord SENZA CALIBRAZIONE...")
			robot.rotateDeg(-error);

		currAngle = compass.getBearing();
		MSG2(" Heading FINALE SENZA CALIBRAZIONE: ", currAngle);
		countDown(3);

	#pragma region Calibrazione compass
	#define CALIBRATE_COMPASS 1
	#if CALIBRATE_COMPASS

		MSG("Ora CALIBRO...")

		//metto in moto 
		MSG("In moto..");
		robot.compassCalibrate();

	#endif // 0
	#pragma endregion

		// al termine della calibrazione faccio puntare il robot ai 4 punti cardinali
		MSG("Ora punto a nord DOPO CALIBRAZIONE ...")
			countDown(15);
		currAngle = compass.getBearing();
		MSG3("Bearing di partenza: DOPO CALIBRAZIONE ", currAngle, "°");
		error = currAngle - DestAngle;
	#if 0  // non va se gli IR sono accecati dalla luce
		robot.rotateDeg(-error);
	#else
		while (true)
		{
			//misuro
			int currAngle = compass.getBearing();
			MSG2("current Bearing: ", currAngle);

			// calcolo l'errore----------------------
			int error = currAngle - DestAngle;
			if (abs(error) < 10)
			{
				robot.stop();
				break;
			}
			else
			{

				if (error < 180)
				{
					// sotto mezzo giro l'errore lo considero >0
				}
				else
				{
					// oltre mezzo giro 
					error = DestAngle + 360 - currAngle;
				}
				MSG2("Error  ", error);

				// regolo
				if (error > 0)
				{
					robot.go(commandDir_e::GOR, (motCk_t)robotSpeed_e::SLOW);
				}
				else if (error < 0)
				{
					robot.go(commandDir_e::GOL, (motCk_t)robotSpeed_e::SLOW);
				}



			}
		}

	#endif // 0

		//robot.stop();
		MSG("**********Arrivato********");
		MSG3("Bearing di arrivo: ", compass.getBearing(), "°");

		while (true) {
			TOGGLEPIN(Pin_LED_TOP_G);
			delay(300);
		}
	#endif // COMPASS



	// TEST LDS,COMPASS,SONAR.. INTANTO CHE GIRA
	#if 1

		MSG("TEST LDS,COMPASS,SONAR..");
		MSG("PLEASE SWITCH MOTORS ON");
		MSG("DISCONNECT CABLES");
		MSG("Start test in 10 secondi...");
		countDown(5);
		LASER_ON

		//1)inizializzo i sensori e le variabili
		int pingCm = 0;
		int ldsmm = 0;
		int estimatedDistCm = 0;
		MSG("COMPASS init");
		Wire.begin();
		compass.begin(2);

		initDone = true;
		while (!initDone)
		{
			MSG("LDS init...");
			if (LDS.init())
			{
				MSG("LDS OK;");
				initDone = true;
			}
			else
			{
				MSG("LDS   ..FAIL;");
				delay(500);
			}

		}
		delay(1000);
//		LDS.setTimeout(500);


		MSG("MOVE FW 5cm");
		robot.moveCm(5);
		MSG("MOVE BK 5cm");
		robot.moveCm(-5);
		//MSG("Target 180");
		//delay(1000);
		//robot.rotateHeading(180, 5);
		//MSG("Target 90");
		//delay(1000);
		//robot.rotateHeading(90, 5);
		//delay(1000);
		//MSG("Target 270");
		//robot.rotateHeading(270, 5);
		//delay(1000);
		//MSG("Target 0");
		//robot.rotateHeading(270, 5);
		//MSG("Target 180");







	#endif // TEST_ALL


	#pragma endregion //regione dei test

	#endif // 0


	// ########################################################################################
	// ########################################################################################
	//  TEST OPZIONALI
	// ########################################################################################
	// ########################################################################################

	#pragma region TEST OPZIONALI PRIMA DELL'AVVIO DELL'OS




		#if 0
			while (true) {
				MSG2("Angle with max FreeDist: ", robot.goCWAndScanDeg(360));
				delay(2000);
				MSG2("Angle with max FreeDist: ", robot.goCWAndScanDeg(-360));
				delay(2000);

			}

		#endif // 1


			// test ESPLORA
		#if 0
			int backDist = 10; // quanto va indietro quando trova ostacolo
			int movedmm = 0;
			int minFreeDistCm = 10;
			int nextRotCw = -1;
			robot.status.sensors.ignoreIR = true; //usato da isObstacleFreeIR()
			while (true)
			{
				LEDTOP_B_ON
					nextRotCw *= -1; //inverto il senso di rotazione
									 // cerco l'angolo con il maggior perccorso libero
				targetAngle = robot.goCWAndScanDeg(nextRotCw * 180);
				delay(1000); //per debug

				MSG("----------");
				MSG2("FreeDist : ", freeDistCm);
				MSG2("  AT Alfa:", targetAngle);
				MSG("----------");
				LEDTOP_B_OFF

					LEDTOP_G_ON
					// ruoto fino all'angolo
					robot.goHeading(targetAngle);//robot.rotateDeg(deltaAngle);
				MSG2("FreeDist: ", freeDistCm);
				delay(1000); //per debug
							 //// mi metto in moto
							 //robot.go(commandDir_e::GOF, robotSpeed_e::MEDIUM);
							 //freeDistCm = robot.status.sensors.ldsEchos_cm[targetAngle];
							 ////
							 //while (robot.isObstacleFreeBumpers() && (freeDistCm>5) )
							 //{
							 //	freeDistCm= robot.getLDSDistanceCm();
							 //	robot.readBumpers();

							 //}

				LEDTOP_G_OFF

					////avanti è abbastanza libero ?
					movedmm = robot.moveCm(freeDistCm);
				MSG3("Moved: ", movedmm, "mm");

				//ho incontrato ostacolo?
				if ((movedmm < 100) || (movedmm < 10 * (freeDistCm - backDist)))
				{
					//sì torno indietro

					robot.moveCm(-backDist);
					LEDTOP_R_ON

						if ((robot.status.sensors.bumper.left) || (robot.status.sensors.irproxy.fl)) //ostacolo a sinistra
						{

							robot.goCWUntilMinFreeDist(50);
							//angle = robot.goCWAndScan(10, true);
							//robot.goHeading(angle);

						}
						else if ((robot.status.sensors.bumper.right) || (robot.status.sensors.irproxy.fr))
						{
							robot.goCWUntilMinFreeDist(-50);
							//angle = robot.goCWAndScan(10, false);
							//robot.goHeading(angle);

						}
					//robot.rotateDeg(nextRotCw*20);
					robot.moveCm(robot.getLDSDistanceCm() - backDist);
					//minFreeDistCm -= max(minFreeDistCm-20,20);


					LEDTOP_R_OFF

				}
				else //no ostacolo
				{
					//minFreeDistCm = min(200, freeDistCm + 20);
					//LEDTOP_B_ON

				}
				//}
				//else 
				//{
				//	// no
				//	robot.moveCm(-backDist);
				//	//robot.goCWUntilMinFreeDist(minFreeDistCm);
				//	//robot.rotateDeg(nextRotCw*20);
				//	// path libero > incremento

				//}

			}
		#endif // 1


		//TEST_COMPASS_3   ### BLOCCANTE ###
		#if 0		//calibra  e poi ruota verso Nord
			// Initialise the sensor 
			robot.stop();
			robot.status.sensors.EncR = 0;


			MSG("Test GO 1 giro....")
			//robot.goCWAndScan(20);
			long targetRads = 2 * PI; //target in radianti
			robot.status.cmd.targetEncoderThicks = targetRads *  ROBOT_MOTOR_STEPS_PER_RADIANT / ROBOT_MOTOR_STEPS_PER_ENCODERTICK;
			MSG2("Target Enc.", robot.status.cmd.targetEncoderThicks);

			robot.go(commandDir_e::GOR, robotSpeed_e::MEDIUM);
			countDown(5);

			while (true)
			{
				MSG2("Enc R.", robot.status.sensors.EncR);
				MSG2("LDS.", robot.getLDSDistanceCm());
				delay(1000);

			}

			/*
			long targetRads = 2 * PI; //un giro
			robot.status.cmd.targetEncoderThicks = targetRads *  ROBOT_MOTOR_STEPS_PER_RADIANT /  ROBOT_MOTOR_STEPS_PER_ENCODERTICK;
			MSG2("Target Enc.", targetEncoderTicks);

			// per il tempo a disposizione...
			while (robot.status.sensors.EncR < targetEncoderTicks) //tempo sufficiente per fare almeno un giro
			{
			// rilevo distanza e angolo
			MSG2("Enc R.", robot.status.sensors.EncR);
			delay(500);
			}

			robot.stop();

			MSG("..End Scan");
			*/
		#endif // COMPASS



		//TEST_COMPASS_2   ### BLOCCANTE ###
		#if 0		//calibra  e poi ruota verso Nord
			// Initialise the sensor 
			robot.stop();
			MSG("CALIBRAZIONE COMPASS. Accendere i motori......")
				countDown(5);
			robot.compassCalibrate();

			// al termine della calibrazione faccio puntare il robot ai 4 punti cardinali
			MSG("Ora punto a nord DOPO CALIBRAZIONE ...")
				robot.goHeading(0, 3);
			MSG("**********Arrivato********");
			MSG3("Bearing di arrivo: ", compass.getBearing(), "°");

			while (true) {
				TOGGLEPIN(Pin_LED_TOP_G);
				delay(300);
			}
		#endif // COMPASS

		// TEST SONAR
		#if 0	
			MSG("SONAR ONLY TEST..");
			for (size_t i = 0; i < 5; i++)		//		while (true)
			{
				TOGGLEPIN(Pin_LaserOn);
				MSG3("Ping: ", robot.sonarPing(), "cm");
				delay(300);
			}
		#endif // 0

		// ruota finchè non trova davanti una distanza minima libera
		#if 0 		
			MSG("goCWUntilMinFreeDist..")
				robot.status.sensors.sonarEchos[90] = robot.goCWUntilMinFreeDist(200);
			MSG2("dist found cm:", robot.status.sensors.sonarEchos[90]);
		#endif

		// RUOTA con velocità crescente 1 secondo ogni step
		#if 0

			// Attenzione !! Se presente un altro tone() non funziona
			robot.stop();
			MSG("TEST robot.go()..")
			MSG("PLEASE SWITCH MOTORS ON")
			MSG("DISCONNECT CABLES")
			MSG("Start test in 10 secondi...")
			countDown(10);
			int ck = ROBOT_MOTOR_CLOCK_microsecondMAX;
			bool cont = true;
			while (robot.isObstacleFreeBumpers())
			{
				MSG2("Current ck:", ck)
					robot.go(commandDir_e::GOR, ck);
				delay(1000);
				ck -= 200;
				if (ck < ROBOT_MOTOR_CLOCK_microsecondMIN)
				{
					//torno indietro per 3 secondi a velocità media
					robot.stop();
					robot.go(commandDir_e::GOL, robotSpeed_e::MEDIUM);
					delay(3000);
					cont = false;
				}
				robot.readBumpers();
			}

			MSG("Fine test......");
			robot.stop();
			//while (true) {}

		#endif // 0


		// I2C Scanner	 
		#if 0		


			byte error, address;
			int nDevices;

			Serial.println("Scanning...");

			nDevices = 0;
			for (address = 1; address < 127; address++)
			{
				// The i2c_scanner uses the return value of
				// the Write.endTransmisstion to see if
				// a device did acknowledge to the address.
				error = I2c.write(address, 0);


				if (error == 0)
				{
					Serial.print("I2C device found at address 0x");
					if (address < 16)
						Serial.print("0");
					Serial.print(address, HEX);
					Serial.println("  !");

					nDevices++;
				}
				else if (error == 4)
				{
					Serial.print("Unknow error at address 0x");
					if (address < 16)
						Serial.print("0");
					Serial.println(address, HEX);
				}
			}
			if (nDevices == 0)
				Serial.println("No I2C devices found\n");
			else
				Serial.println("done\n");

			delay(5000);           // wait 5 seconds for next scan

		#endif 

		// TEST_GPS 
		#if 0
								   //CHIAVARI LAT	44.326953 LONG 9.289679

		#endif 


		// TEST_SPEECH
		#if 0		 
			SERIAL_MSG.println(F("1,test Speak;"));
			SERIAL_MMI.println(F("28,CIAO CIAO;"));
		#endif // TEST_SPEECH


		//test ixproxy
		#if 0	
			SERIAL_MSG.print(F("1,testing ixproxy...;"));
			while (true)
			{
				robot.readSensors();
				if (robot.status.sensors.irproxy.fw != robot.statusOld.sensors.irproxy.fw)
				{
					SERIAL_MSG.println(F("1,irproxy.fw changed;"));
					TOGGLEPIN(Pin_LED_TOP_B);

				}
				else if (robot.status.sensors.irproxy.fwHL != robot.statusOld.sensors.irproxy.fwHL)
				{
					SERIAL_MSG.println(F("1,irproxy.fwHl changed;"));
					TOGGLEPIN(Pin_LED_TOP_G);

				}
			}

		#endif // 0//test ixproxy




		// TEST_COMPASS_1 SENZA CALIBRAZIONE 			
		#if 0		///Ruota misurando in continuazione
			MSG("Ora punto a nord SENZA CALIBRAZIONE...")
				robot.goHeading(3);
		#endif // 1




		// RUOTA E MISURA LDS,COMPASS,SONAR.. 
		#if 0  

			MSG("TEST LDS,COMPASS,SONAR..");
			MSG("ACCENDI MOTORI E SCOLLEGA CAVI");

			countDown(5);
			LASER_ON

				//1)inizializzo i sensori e le variabili
				int pingCm = 0;
			int ldsmm = 0;
			int estimatedDistCm = 0;

			//MSG("Target 180");
			//delay(1000);
			//robot.rotateHeading(180, 5);
			//MSG("Target 90");
			//delay(1000);
			//robot.rotateHeading(90, 5);
			//delay(1000);
			//MSG("Target 270");
			//robot.rotateHeading(270, 5);
			//delay(1000);
			//MSG("Target 0");
			//robot.rotateHeading(270, 5);
			//MSG("Target 180");


		#endif // TEST_ALL

		//PROVA SERVO----------------------------------
		#if 0	

			MSG("Servo al centro...")
				//servoSonar.attach(Pin_ServoSonarPWM);
				robot.ServoAtPos(10); delay(1000);
			robot.ServoAtPos(160); delay(1000);
			robot.ServoAtPos(90); delay(1000);


		#endif // 0

	#pragma endregion //regione dei test


	LEDTOP_R_OFF

 }


void loop() {
	MSG("Start Test Seqence....");

	lampeggiaLed(Pin_LED_TOP_G, 5, 10);
	LEDTOP_G_ON;
	// wait to enter print region
	//chMtxLock(&mutexSerialMMI);
 
	onCmdGetSensorsHRate(&cmdMMI);
	onCmdGetSensorsLRate(&cmdMMI);
 
  
	MSG3("Bat : ", robot.readBattChargeLevel(), "%");

	MSG2("A0 : ", analogRead(A0));
	MSG2("A1 : ", analogRead(A1));
	MSG2("A1 : ", analogRead(A2));

	//testMPU2();
	testMPU(60);

	testLDS(10);


	testCompass(10);
	//testGPS();
}



