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

#pragma region LIBRERIE
// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	// LIBRERIE VARIE
	/// ///////////////////////////////////////////////////////////////////////////////
	#include <digitalWriteFast.h>
	//#include <ChibiOS_AVR.h>
	//#include <util/atomic.h>
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
		void ISRldsHome() {
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
		void ISRldsEnd() {
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
					robot.status.sensors.encRcnt += 1;
					robot.status.sensors.encRprec = robot.status.sensors.encR;
				}
			}
		}

		void setupEncoders() {
			pinMode(Pin_EncRa, INPUT);// open >+5 closed =gnd
		// was 	attachInterrupt(digitalPinToInterrupt(Pin_EncRa), ISRencoderR, CHANGE);
			attachPinChangeInterrupt(Pin_EncRa, ISRencoderR, CHANGE);  // add more attachInterrupt code as required

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
			delay(300);
		LEDTOP_R_OFF

		LEDTOP_G_ON	// Indica inizio SETUP Phase
			delay(300);
		LEDTOP_G_OFF

		LEDTOP_B_ON	// Indica inizio SETUP Phase
			delay(300);
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


/// ///////////////////////////////////////////////////////////////////////////////
// / ///////////////////////////////////////////////////////////////////////////////
///  																			///
//		R	O	S		S E G M E N T
///
// / ///////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region ROS Libraries and Variables
	#include <math.h>
	#include <ros.h>
	#include <ros/duration.h>
	#include <ros/time.h> //non serve
	#include <ros/msg.h>
	#include <std_msgs/String.h>
	#include <std_msgs/Empty.h>
	#include <std_msgs/Float64.h>
	#include <std_msgs/String.h>

	//#include <ros_lib\sensor_msgs\Range.h>
	#include <sensor_msgs/Range.h>
	#include <tf/transform_broadcaster.h>

 
	ros::NodeHandle  nh;

	#pragma region ROS HELPER FUNCTIONS
		/// ///////////////////////////////////////////////////////////////////////////////
		///  ROS HELPER FUNCTIONS /////////////////////////////////////////////////////////
		/// ///////////////////////////////////////////////////////////////////////////////

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


	#pragma endregion

	/// ///////////////////////////////////////////////////////////////////////////////
	//  ROS Frame_Id utilizzati ///////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	#pragma region ROS Frame_Id utilizzati
		char base_link_frame_id[] = "/base_link";
		char odom_frame_id[] = "/odom";

	#pragma endregion


	/// ///////////////////////////////////////////////////////////////////////////////
	//  PUBLISHERS  ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////


	#pragma region ROS chatter
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//  chatter                      ---------------------------------//
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
			std_msgs::String str_msg;
			ros::Publisher chatter("/chatter", &str_msg);

			void setup_chatter() {
				nh.advertise(chatter);
				ROS_INFO("setup chatter");
			}

			// Pubblica  chatter ----------------------------------
			void publish_chatter(char* charVal) {
				//str_msg.data = strcat("ultra sound:", charVal);
				str_msg.data = charVal;
				chatter.publish(&str_msg);
				ROS_INFO("publish_chatter");
			}

	#pragma endregion


	#pragma region ROS tf
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//  tf                                  ---------------------------------//
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		#include <tf/tf.h>
		#include <tf/transform_broadcaster.h>
		geometry_msgs::TransformStamped t;
		tf::TransformBroadcaster broadcaster;
		//const char* base_link[] = "base_link";
		//const char* odom[] = "odom";

		//char base_link_frame_id[] = "/base_link";
		//char odom[] = "/odom";

		unsigned long tf_time;

		// Pubblica  tf ----------------------------------
		void publish_tf() {

			if (millis() > tf_time) {

				// broadcast tf odom->base_link-------------
				t.header.stamp = nh.now();
				t.header.frame_id = odom_frame_id;
				t.child_frame_id = base_link_frame_id;

				t.transform.translation.x = robot.status.posCurrent.x / 1000;
				t.transform.translation.y = robot.status.posCurrent.y / 1000;
				t.transform.translation.z = 0.0;

				t.transform.rotation = tf::createQuaternionFromYaw((float)robot.status.posCurrent.r*DEG_TO_RAD);


				broadcaster.sendTransform(t);

				ROS_INFO("P. TF");
				//------------------------------------------
				tf_time = millis() + 200;
			}

		}



	#pragma endregion



	#pragma region ROS odom

		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//  odom                                ---------------------------------//
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		#include <tf/tf.h>		// richiesto da  createQuaternionFromYaw
		#include <tf/transform_broadcaster.h>
		#include <nav_msgs/Odometry.h>
		unsigned long odom_time;
		//double x = 0.0;
		//double y = 0.0;
		//double th = 0.0;

		//double vx = 0.1;
		//double vy = -0.1;
		//double vth = 0.1;

		tf::TransformBroadcaster odom_broadcaster;
		nav_msgs::Odometry rosmsg_odom;
		ros::Publisher pub_odom("/odom", &rosmsg_odom);

		// setup odometry da chiamare nel setup_ros() ----------------------------------
		void setup_odom() {
			nh.advertise(pub_odom);
			//ROS_INFO("setup_odom");
		}
		// Pubblica  odometry  ----------------------------------
		void publish_odom() {
			if (millis() > odom_time) {
 

				//first, we'll publish the transform over tf
//				publish_tf();
				//geometry_msgs::TransformStamped odom_trans;
				//odom_trans.header.stamp = nh.now();
				//odom_trans.header.frame_id = odom_frame_id;
				//odom_trans.child_frame_id = base_link_frame_id;
				//odom_trans.transform.translation.x = robot.status.posCurrent.x/1000;
				//odom_trans.transform.translation.y = robot.status.posCurrent.y/1000;
				//odom_trans.transform.translation.z = 0.0;
				////since all odometry is 6DOF we'll need a quaternion created from yaw
				//odom_trans.transform.rotation = tf::createQuaternionFromYaw((float)robot.status.posCurrent.r*DEG_TO_RAD);
				////send the transform
				//odom_broadcaster.sendTransform(odom_trans);

				//next, we'll publish the odometry message over ROS
				//nav_msgs::Odometry odom;
				rosmsg_odom.header.stamp = nh.now();;
				rosmsg_odom.header.frame_id = odom_frame_id;

				//set the position
				rosmsg_odom.pose.pose.position.x = (float)robot.status.posCurrent.x/1000;
				rosmsg_odom.pose.pose.position.y = (float)robot.status.posCurrent.y/1000;
				rosmsg_odom.pose.pose.position.z = 0.0;
				rosmsg_odom.pose.pose.orientation = tf::createQuaternionFromYaw((float)robot.status.posCurrent.r*DEG_TO_RAD);

				//set the velocity
				rosmsg_odom.child_frame_id = base_link_frame_id;
				rosmsg_odom.twist.twist.linear.x = robot.status.cmd.targetVelocityLinear;
				rosmsg_odom.twist.twist.linear.y = 0;
				rosmsg_odom.twist.twist.angular.z = robot.status.cmd.targetVelocityAngular;

				//publish the message
				pub_odom.publish(&rosmsg_odom);
				//ROS_INFO("pub odom");
				odom_time = millis() + 500;

				// debug

				//ROS_INFO2F("x ", rosmsg_odom.pose.pose.position.x);
				//ROS_INFO2F("y ", rosmsg_odom.pose.pose.position.y);
				//ROS_INFO2F("th ", rosmsg_odom.pose.pose.orientation.z);
			}
		}



	#pragma endregion


	/// ///////////////////////////////////////////////////////////////////////////////
	//  SUBSCRIBERS ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	/*
	1. includi qui i .h dei messaggi
	2. metti qui le funzioni di callback void rosCb_cmd_vel(const std_msgs::Float64& msg)
	3. metti qui ros::Subscriber<std_msgs::Float64> sub_my1("your_topic", &messageCb);
	4. in Setup() metti nh.subscribe(sub_my1);*/

   
	#pragma region ROS cmdVel

		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//  twist command                       ---------------------------------//
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
//		#include <ros_lib\turtle_actionlib\Velocity.h>
		#include <geometry_msgs/Twist.h>  // cmd_vel


		unsigned long twist_command_time = 0;



		//-----------------------------------------------------------------
		//void rosCb_cmd_vel(const geometry_msgs::Twist & cmd_vel);

		void rosCb_cmd_vel(const geometry_msgs::Twist & cmdVel)
		{

			//ROS_INFO2F("cmd_vel.x:", robot.status.cmd.targetVelocityLinear);
			//ROS_INFO2F("cmd_vel.z:", robot.status.cmd.targetVelocityAngular);
			if ((cmdVel.angular.z != 0.0) || (cmdVel.linear.x != 0.0))
			{
				ROS_INFO("rosCb_cmd_vel");
				//MSG2F("cmd_vel x", robot.status.cmd.targetVelocityLinear);
				//MSG2F("cmd_vel z", robot.status.cmd.targetVelocityAngular); 			
				robot.goCmdVel(cmdVel.linear.x, cmdVel.angular.z);
			}
			else
			{
				ROS_INFO("stop");
				robot.stop();

			}
		}
		ros::Subscriber <geometry_msgs::Twist> command_cmd_vel("cmd_vel", rosCb_cmd_vel);


		void setup_cmd_vel() {
			nh.subscribe(command_cmd_vel);
			ROS_INFO("setup cmd_vel");
		}

	#pragma endregion


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
void setupRos() {
	nh.initNode();
	nh.getHardware()->setBaud(57600);

//	broadcaster.init(nh);
//	ROS_INFO("ROBOT SETUP COMPLETE");

	setup_cmd_vel();
	setup_odom();
	setup_chatter();

	//setup_speech();
	//setup_ultrasound();
	//setup_laserscan();

	nh.spinOnce();
//	ROS_INFO("ROS SETUP COMPLETE");

}

void setup()
{
	LEDTOP_ALL_OFF;

	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
 
	nh.getHardware()->setBaud(57600);
	//MSG("===Test Robot In Out===")
	nh.initNode();
	nh.subscribe(command_cmd_vel);
//	nh.advertise(pub_odom);

	setupRobot();
	myLDSstepper.disable();

	//setupRos();
	//ROS_INFO("ROS SETUP COMPLETE");

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
	ledSeq1();
	nh.spinOnce();

	//MSG("====================")
	//MSG2("Start Test Seq. #", loopCounter++);



	//publish_tf();
	robot.updatePoseFromEncoders(); // aggiorna robot.status.posCurrent
//	publish_odom();//pubblica in odom robot.status.posCurrent
	//publish_chatter("hello");
	//publish_speech();
	//publish_ultrasound();
	//publish_laserscan(); // acquisisce le distanze...

	//MSG2("pose.x:", robot.status.posCurrent.x);
	//MSG2("pose.y:", robot.status.posCurrent.y);
	//MSG2("pose.r:", robot.status.posCurrent.r);
	

	//testGPS();
	//MSG("======== end ========")
	delay(500);
}



