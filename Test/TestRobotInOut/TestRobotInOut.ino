// verifica il funzionamento dei vari componenti HW
// no ROS
// usa MMI 


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

#include <I2Cdev\I2Cdev.h>

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
	#if OPT_MPU6050

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


		bool setupLDS() {
			byte failCount = 0;
			bool initDone = false;
			while (!initDone && ( failCount < 10))
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
		myStepper_c myLDSstepper(PIN_STEPPERLDS_CK, PIN_STEPPERLDS_ENABLE, PIN_STEPPERLDS_CW, PIN_STEPPERLDS_END);

		byte ckState = 0;


		#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1

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


		void ISRstepperSwitchHome() {
			static unsigned long last_interrupt_timeHome = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
					myLDSstepper.setHomePosition(true);
					myLDSstepper.setCW(true);  ///.disable();
			}
			last_interrupt_timeHome = interrupt_time;
		}

		void ISRstepperSwitchEnd() {
			static unsigned long last_interrupt_timeEnd = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{

					myLDSstepper.setEndPosition(true);
					myLDSstepper.setCW(false);  ///.disable();
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


		void setupStepperLDS(float speed = 2*PI) {
			pinMode(PIN_STEPPERLDS_HOME, INPUT_PULLUP);// open >+5 closed =gnd
			pinMode(PIN_STEPPERLDS_END, INPUT_PULLUP);// open >+5 closed =gnd
			//attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, CHANGE);  // add more attachInterrupt code as required
			//attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, CHANGE);  // add more attachInterrupt code as required

			attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, FALLING);  // add more attachInterrupt code as required
			attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, FALLING);  // add more attachInterrupt code as required
																						// start stepper
			myLDSstepper.goRadsPerSecond(speed);
}

	#endif



	// per i test
		int targetAngle = 0;
		int freeDistCm = 0;
		char charVal[10];

	#pragma endregion

	#include <robot.h>
	struct robot_c robot;	//was  struct robot_c robot;


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

		void setupEncoders() {
			pinMode(Pin_EncRa, INPUT);// open >+5 closed =gnd
		// was 	attachInterrupt(digitalPinToInterrupt(Pin_EncRa), ISRencoderR, CHANGE);
			attachPinChangeInterrupt(Pin_EncRa, ISRencoderR, CHANGE);  // add more attachInterrupt code as required

		}

		void readEncoders(float * dx, float * dth) { // rirorna la lettura degli encoder e li azzera
			MSG2("encR ",robot.status.sensors.encR);
			if (robot.status.cmd.commandDir == commandDir_e::GOF)
			{
				*dx = robot.status.sensors.encR* ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_M2STEPS;
			}
			if (robot.status.cmd.commandDir == commandDir_e::GOB)
			{
				*dx = -robot.status.sensors.encR* ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_M2STEPS;
				*dth = 0.0;
			}
			if (robot.status.cmd.commandDir == commandDir_e::GOR)
			{
				*dx = 0.0;
				*dth = -robot.status.sensors.encR* ROBOT_MOTOR_STEPS_PER_ENCODERTICK / ROBOT_RAD2STEPS ;
			}

			robot.status.sensors.encR = 0;
			robot.status.sensors.encL = 0;
		}
	#endif


	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CmdMessenger object to the default Serial port
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>
	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//------------------------------------------------------------------------------





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

// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma endregion




// ########################################################################################
// ########################################################################################
//  TEST Functions
// ########################################################################################
// ########################################################################################
	int scan[100];
#pragma region Test Functions
	void testEncoders() {
		float dx = 0.0;
		float dth = 0.0;
		readEncoders(&dx, &dth);
		MSG2F("dx ", dx);
		MSG2F("dth ", dth);

	}

	void testIRproxy() {
		MSG("testing ixproxy...");
		while (true)
		{
			robot.readAllSensors();
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

	}
	void testObstacle() {
		SERIAL_MSG.print(F("1,testing robot.isObstacleFree()...;"));
		bool val = false;
		bool oldVal = false;
		robot.status.cmd.commandDir = GOF;
		while (val)
		{
			val = robot.isObstacleFree();
			if (oldVal != val)
			{
				TOGGLEPIN(Pin_LED_TOP_B);
				SERIAL_MSG.println("1,robot.isObstacleFree() changed;");
				oldVal = val;
			}
		}

	}
	void testSpeech() {
		MSG("1,test Speak;");
		SERIAL_MMI.println(F("28,CIAO CIAO;"));

	}
	void testLaser() {
	MSG("Laser ON OFF 5 secondi...");
	LASER_ON
	countDown(5);
	LASER_OFF

	}
	#if OPT_MPU
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
	#endif
	int i, measures = 0;
 
	void testLDS(int iterations) { //verifica che funzioni LDS a prescinder dalllo stepper
		for (size_t i = 0; i < iterations; i++)
		{
			MSG2("cm:",	robot.getLDSDistanceCm());
		}
	}
	void testScanLDS(int scansioni= 1) {
		MSG2("LDS loops ", scansioni);

		for (size_t j = 0; j < scansioni; j++) {
			i = 0;
			//Attende che sia in home
			while (!myLDSstepper.isHomePosition())	{delay(10);	}
			LASER_ON;
			LEDTOP_B_ON;
			while (!myLDSstepper.isEndPosition()) 
			{
				scan[i++] = robot.getLDSDistanceCm();
				delay(200);

			}
			LASER_OFF;
			LEDTOP_B_OFF;
			measures = i;
			MSG2("# ", measures);			
			//stampa le misure intanto che la scansione torna indietro
			for (size_t k = 0; k < measures; k++)
			{
				MSG2("# ", k);
				MSG2("LDS cm: ",scan[k] );

			}

		}

	}
	void testCompass(int iterations) {

		//MSG("Compass calibration...")
		//robot.compassCalibrate();
		MSG("Test Compass")
		Wire.begin();
		compass.begin(2);
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
	void testMotorsGoHeading(int finalHeading= 3) {

		MSG("## goHeading 0:");
		robot.stop();
		robot.goHeading(0, 5);
		delay(1000);

		MSG("## goHeading 90:");
		robot.stop();
		robot.goHeading(90, 5);
		delay(1000);

		MSG("## goHeading 180:");
		robot.stop();
		robot.goHeading(180, 5);
		delay(1000);

		MSG("## goHeading 270:");
		robot.stop();
		robot.goHeading(270, 5);
		delay(1000);
		MSG2("## goHeading :", robot.readCompassDeg());
		robot.stop();
		//MOTORS_DISABLE
		robot.goHeading(finalHeading, 5);
		delay(5000);

}
	void testMotorsCmdVel() {
		// RUOTA con velocità crescente 1 secondo ogni step
		// Attenzione !! Se presente un altro tone() non funziona
		robot.stop();
		MSG("TEST robot.go()..")
		MSG("PLEASE SWITCH MOTORS ON")
		MSG("DISCONNECT CABLES")
		MSG("Start test in 3 secondi...")
		countDown(3);

/*
		int ck = ROBOT_MOTOR_CLOCK_microsecondMAX;
		bool cont = true;
		while (cont)
		{
			MSG2("mot. ck:", ck)
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
*/
		// limiti velocità lineare
		// Vmin = Hzmin/CMDVEL_LINEAR_TO_MOTORHZ = 166.7/2000 = 0.083 m/s
		// Vmax =HzMax/CMDVEL_LINEAR_TO_MOTORHZ =  400 /2000 = 0.2 m/s
		// Vang_min =Hzmin/CMDVEL_ANGULAR_TO_MOTORHZ  =  166.7  /341,992 =0.487 rad/sec
		// Vang_Max =HzMax/CMDVEL_ANGULAR_TO_MOTORHZ  =   400 /341,992 =1.17 rad/sec

		#define T1 2000		// forward time
		#define V1 0.1		// limear speed

		#define T2 4000		// rotation time msec
		#define V2  PI/8	// angular speed rad/s

		double motTime = 0;

		//rotaz 360°
		robot.goCmdVel(0, V2);delay(T2*4); testEncoders();
		delay(2000);
		robot.goCmdVel(V1, 0); delay(T1); testEncoders();
		robot.goCmdVel(0, V2);delay(T2); testEncoders();
		robot.goCmdVel(V1/2, 0);delay(T1); testEncoders();
		robot.goCmdVel(0, V2);delay(T2); testEncoders();
		robot.goCmdVel(V1, 0);delay(T1); testEncoders();
		robot.goCmdVel(0, V2);delay(T2); testEncoders();
		robot.goCmdVel(V1/2, 0);delay(T1); testEncoders();
		robot.goCmdVel(0, V2);delay(T2); testEncoders();

		MSG("Fine test......")
			robot.stop();

	}
	void I2Cscanner() {
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

}

	void testEsplora(){
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
	}

	void testServo() {
		//PROVA SERVO----------------------------------
		#if 0	
			MSG("Servo al centro...")
			//servoSonar.attach(Pin_ServoSonarPWM);
			robot.ServoAtPos(10); delay(1000);
			robot.ServoAtPos(160); delay(1000);
			robot.ServoAtPos(90); delay(1000);
		#endif // 0
	}
	void testGoTargetEncoderTicks() {  // rotazione non bloccante con uso di targetEncoderThicks
		robot.stop();
		robot.status.sensors.encR = 0;
		MSG("Test GO 1 giro....")
		long targetRads = 2 * PI; //target in radianti
		robot.status.cmd.targetEncoderThicks = targetRads *  ROBOT_MOTOR_STEPS_PER_RADIANT / ROBOT_MOTOR_STEPS_PER_ENCODERTICK;
		MSG2("Target Enc.", robot.status.cmd.targetEncoderThicks);

		robot.go(commandDir_e::GOR, robotSpeed_e::MEDIUM);
		countDown(5);

		while (true)
		{
			MSG2("Enc R.", robot.status.sensors.encR);
			MSG2("LDS.", robot.getLDSDistanceCm());
			delay(1000);

		}

	}

#pragma endregion


// ########################################################################################
// ########################################################################################
//  S E T U P
// ########################################################################################
// ########################################################################################
void setupRobot()
{
	//lampeggiaLed(Pin_LED_TOP_R, 3, 5);
	//lampeggiaLed(Pin_LED_TOP_G, 3, 5);
	//lampeggiaLed(Pin_LED_TOP_B, 3, 5);
	LEDTOP_R_ON	// Indica inizio SETUP Phase
	MOTORS_DISABLE

	InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions
					  //sets the frequency for the specified pin
					  //bool success = SetPinFrequencySafe(Pin_LED_TOP_R, 2);

	////if the pin frequency was set successfully, turn pin 13 on
	//if (success) {
	//	digitalWrite(Pin_LED_TOP_B, HIGH);
	//	pwmWrite(Pin_LED_TOP_R, 128); //set 128 to obtain 50%duty cyle
	//	SetPinFrequency(Pin_LED_TOP_R, 2); //setting the frequency to 10 Hz
	//}

	delay(5000);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE 5 ok
	Wire.begin(); // default timeout 1000ms

	#if OPT_ENCODERS

		setupEncoders();
	#endif

	#if OPT_MPU6050

		setupMPU();
	#endif

	#if OPT_GPS
		robot.initRobot(&Gps);
	#endif
	#if OPT_LDS
		robot.initRobot(&LDS);
		//setupLDS(); 	//robot.initLDS(&LDS);
	#endif
	#if OPT_COMPASS
		robot.initRobot(&compass);
		robot.initCompass(&compass);//include compass.begin();
		//	robot.compassCalibrate();

	#endif
	#if OPT_STEPPERLDS
		robot.initRobot(&myLDSstepper);
		setupStepperLDS(); //avvia lo stepper
	#endif

	#if OPT_SEROVOSONAR
		robot.initRobot(&servoSonar);
	#endif


	robot.initRobot(); //inizializza solo le variabili

	//	noTone(Pin_LED_TOP_R);

}
	
void setup()
{
	LEDTOP_ALL_OFF;

	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
 
	MSG("===Test Robot In Out===")
 
	setupRobot();

	LEDTOP_ALL_OFF;
	LEDTOP_B_ON
	//robot.initHW(); //disabilita i motori
					//	tone(Pin_LED_TOP_R, 2, 0);
	//MSG("ACCENDI I MOTORI")
	//countDown(5);

	WEBCAM_OFF
	WEBCAM_ON

	LEDTOP_B_OFF
  }

int loopCounter = 1;
void loop() {
	MSG("====================")
	MSG2("Start Test Seq. #", loopCounter++);
	ledSeq1();

 
	// wait to enter print region
	//chMtxLock(&mutexSerialMMI);
 
  
  
	//MSG3("Bat : ", robot.readBattChargeLevel(), "%");

	//MSG2("A0 : ", analogRead(A0));
	//MSG2("A1 : ", analogRead(A1));
	//MSG2("A2 : ", analogRead(A2));
	//testMotorsGoHeading(3);


	while (true)
	{
		testMotorsCmdVel();
		//testEncoders();
		delay(2000);
	}
	testLDS(20);
	testScanLDS(10);


	//testMPU2();
	#if OPT_MPU
		testMPU(60);
	#endif // OPT_MPU


	testCompass(10);
	//testIRproxy();

	//testGPS();
	MSG("======== end ========")
	delay(5000);
}



