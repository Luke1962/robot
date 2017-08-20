// verifica il funzionamento dei vari componenti HW
// no ROS
// usa MMI 

#pragma region ROBOT_SEGMENT
#define OPT_ROBOT 0
	/// ///////////////////////////////////////////////////////////////////////////////
	// LIBRERIE DI BASE
	/// ///////////////////////////////////////////////////////////////////////////////
	#include <Arduino.h>	//per AttachInterrupt
	#include <digitalWriteFast.h>
	#include <I2Cdev\I2Cdev.h>

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
	//#include <ChibiOS_AVR.h>
	//#include <util/atomic.h>
	//#include <TinyGPSplus/TinyGPS++.h>
	//#include <StackArray.h>
//	#include <avr/interrupt.h>



	#pragma endregion




	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CREAZIONE OGGETTI GLOBALI E RELATIVE LIBRERIE
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#pragma region CREAZIONE OGGETTI GLOBALI
	#if OPT_MPU6050
		#include <Wire.h>
		#include <MPU6050.h>
		MPU6050 mpu;
 
		void setup_MPU() {
			while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
			{
				Serial.println(F("No MPU6050 sensor, check wiring!"));
				delay(500);
			}
 
			mpu.setI2CMasterModeEnabled(false);
			mpu.setI2CBypassEnabled(true);
			mpu.setSleepEnabled(false);
		}

	#endif // OPT_MPU6050


	#if OPT_COMPASS
		#include <Wire.h>
		#include <compass\compass.h>
		MyCompass_c compass;
	#endif // COMPASS

	#if OPT_SONAR
		#include <Newping\NewPing.h>
		NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
	#endif //

	#if OPT_GPS
		#include <TinyGPSplus\TinyGPS++.h> //deve essere incluso anche nel main
		TinyGPSPlus Gps;
	#endif

	#pragma region VL53L0X distanceSensor

	#if OPT_LDS
		#include <Wire.h>
		#include <VL53L0X\VL53L0X.h>
		VL53L0X LDS;
		#define LONG_RANGE
		// Uncomment ONE of these two lines to get
		// - higher speed at the cost of lower accuracy OR
		// - higher accuracy at the cost of lower speed

		//#define HIGH_SPEED
		//#define HIGH_ACCURACY


		bool setupLDS() {
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


	void setupStepperLDS(float speed = 2 * PI) {
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
//	using namespace robSmall;
	#include <MyRobotModelSmall\MyRobotModelSmall.h>


	//struct robot_c rob;	//was  struct robot_c robot;
	MyRobotModelSmall::robotBaseSmallModel_c rob;	//was  struct robot_c robot;

	#include <MyStepperBase\MyStepperBase.h>
	StepperBase_c robotMotors;
	//volatile StepperBase_c robotMotors(Pin_MotCK,Pin_MotENL,Pin_MotENR,Pin_MotCWL,Pin_MotCWR;

	// ////////////////////////////////////////////////////////////////////////////////////////////
	#if OPT_ENCODERS
	// dopo dichiarazione di rob
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
			rob.status.sensors.encR = digitalReadFast(Pin_EncRa);
			// cambiato il valore ?
			if (rob.status.sensors.encRprec != rob.status.sensors.encR)
			{
				TOGGLEPIN(Pin_LED_TOP_G);
				isrLastCallmSec = isrCurrCallmSec;
				// incrementa
				rob.status.sensors.encRcnt += 1;
				rob.status.sensors.encRprec = rob.status.sensors.encR;
			}
		}
	}

	void setupEncoders() {
		pinMode(Pin_EncRa, INPUT);// open >+5 closed =gnd
	// was 	attachInterrupt(digitalPinToInterrupt(Pin_EncRa), ISRencoderR, CHANGE);
		attachPinChangeInterrupt(Pin_EncRa, ISRencoderR, CHANGE);  // add more attachInterrupt code as required

	}

	#endif

/*
	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CmdMessenger object to the default Serial port
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>
	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//------------------------------------------------------------------------------
*/




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
			delay(ms);
			digitalWriteFast(pin, 0);
			delay(ms);

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
	void ledSeq1(int ms) {
		LEDTOP_R_ON	// Indica inizio SETUP Phase
			delay(ms);
		LEDTOP_R_OFF

			LEDTOP_G_ON	// Indica inizio SETUP Phase
			delay(ms);
		LEDTOP_G_OFF

			LEDTOP_B_ON	// Indica inizio SETUP Phase
			delay(ms);
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


	void setup_Robot()
	{
		initRobotBaseHW();
		LEDTOP_R_ON	// Indica inizio SETUP Phase
		MOTORS_DISABLE

		//InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions
 
		delay(5000);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE 5 ok
		Wire.begin(); // default timeout 1000ms

		#if OPT_ENCODERS

				setupEncoders();
		#endif

		#if OPT_MPU6050

				setup_MPU();
		#endif

		#if OPT_GPS
				//rob.initRobot(&Gps);
		#endif
		#if OPT_LDS
				//rob.initRobot(&LDS);
				//setupLDS(); 	//rob.initLDS(&LDS);
		#endif
		#if OPT_COMPASS
				//rob.initRobot(&compass);
				//rob.initCompass(&compass);//include compass.begin();
											//	rob.compassCalibrate();

		#endif
		#if OPT_STEPPERLDS
				//rob.initRobot(&myLDSstepper);
				setupStepperLDS(); //avvia lo stepper
		#endif

		#if OPT_SEROVOSONAR
				//rob.initRobot(&servoSonar);
		#endif


		//rob.initRobot(); //inizializza solo le variabili
		//rob.initModel(); //inizializza solo le variabili
		STEPPERLDS_OFF

			//	noTone(Pin_LED_TOP_R);

	}



#pragma endregion //ROBOT SEGMENT









/// ///////////////////////////////////////////////////////////////////////////////
// / ///////////////////////////////////////////////////////////////////////////////
///  																			///
//		R	O	S		S E G M E N T
///
// / ///////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region ROS_SEGMENT
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
	#pragma endregion

	#pragma region ROS HELPER FUNCTIONS
		/// ///////////////////////////////////////////////////////////////////////////////
		///  ROS HELPER FUNCTIONS /////////////////////////////////////////////////////////
		/// ///////////////////////////////////////////////////////////////////////////////

		#define ROS_INFO(s) nh.loginfo(s);

		void ROS_INFO2F(char * c, float f) {
		#define CHRFLOATSIZE 7
		#define CHRFLOATDECS 3
			char charVal[CHRFLOATSIZE];  //temporarily holds data from vals
										  //4 is mininum width, 3 is precision; float value is copied onto buff
			dtostrf(f, CHRFLOATSIZE, CHRFLOATDECS, charVal);
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

				t.transform.translation.x = rob.status.posCurrent.x / 1000;
				t.transform.translation.y = rob.status.posCurrent.y / 1000;
				t.transform.translation.z = 0.0;

				t.transform.rotation = tf::createQuaternionFromYaw((float)rob.status.posCurrent.r*DEG_TO_RAD);


				broadcaster.sendTransform(t);

				ROS_INFO("P. TF");
				//------------------------------------------
				tf_time = millis() + 200;
			}

		}



	#pragma endregion
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//  velocity                                ---------------------------------//
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//



	#pragma region ROS odom

		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//  odom                                ---------------------------------//
		//-----------------------------------------------------------------------//
		//-----------------------------------------------------------------------//
		//#include <tf/tf.h>		// richiesto da  createQuaternionFromYaw
		//#include <tf/transform_broadcaster.h>
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
				//odom_trans.transform.translation.x = rob.status.posCurrent.x/1000;
				//odom_trans.transform.translation.y = rob.status.posCurrent.y/1000;
				//odom_trans.transform.translation.z = 0.0;
				////since all odometry is 6DOF we'll need a quaternion created from yaw
				//odom_trans.transform.rotation = tf::createQuaternionFromYaw((float)rob.status.posCurrent.r*DEG_TO_RAD);
				////send the transform
				//odom_broadcaster.sendTransform(odom_trans);

				//next, we'll publish the odometry message over ROS
				//nav_msgs::Odometry odom;
				rosmsg_odom.header.stamp = nh.now();;
				rosmsg_odom.header.frame_id = odom_frame_id;

				//set the position
				rosmsg_odom.pose.pose.position.x = (float)rob.status.posCurrent.x/1000;
				rosmsg_odom.pose.pose.position.y = (float)rob.status.posCurrent.y/1000;
				rosmsg_odom.pose.pose.position.z = 0.0;
				rosmsg_odom.pose.pose.orientation = tf::createQuaternionFromYaw((float)rob.status.posCurrent.r*DEG_TO_RAD);

				//set the velocity
				rosmsg_odom.child_frame_id = base_link_frame_id;
				rosmsg_odom.twist.twist.linear.x = rob.status.cmd.targetVelocityLinear;
				rosmsg_odom.twist.twist.linear.y = 0;
				rosmsg_odom.twist.twist.angular.z = rob.status.cmd.targetVelocityAngular;

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

			//ROS_INFO2F("cmd_vel.x:", rob.status.cmd.targetVelocityLinear);
			//ROS_INFO2F("cmd_vel.z:", rob.status.cmd.targetVelocityAngular);
			if ((cmdVel.angular.z != 0.0) || (cmdVel.linear.x != 0.0))
			{
				ROS_INFO("rosCb_cmd_vel");
				//MSG2F("cmd_vel x", rob.status.cmd.targetVelocityLinear);
				//MSG2F("cmd_vel z", rob.status.cmd.targetVelocityAngular); 			
				rob.updPoseFromEncoders();
				robotMotors.goCmdVel(cmdVel.linear.x, cmdVel.angular.z);
			}
			else
			{
				ROS_INFO("stop");
				robotMotors.stop();
				rob.updPoseFromEncoders();

			}
		}
		ros::Subscriber <geometry_msgs::Twist> command_cmd_vel("cmd_vel", rosCb_cmd_vel);


		void setup_cmd_vel() {
			nh.subscribe(command_cmd_vel);
			ROS_INFO("setup cmd_vel");
		}

	#pragma endregion



	void setup_ROS() {
		MSG("s.ROS")
		nh.getHardware()->setBaud(57600);
		nh.initNode();

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


#pragma endregion




 

// ########################################################################################
// ########################################################################################
//  S E T U P
// ########################################################################################
// ########################################################################################

void setup()
{
	LEDTOP_ALL_OFF;

	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
 
	//nh.getHardware()->setBaud(57600);
	//nh.initNode();


	//nh.subscribe(command_cmd_vel);
	//nh.advertise(pub_odom);
	setup_ROS();
	setup_Robot();


	//setupRos();
	//ROS_INFO("ROS SETUP COMPLETE");

	LEDTOP_ALL_OFF;
	LEDTOP_B_ON
	//rob.initHW(); //disabilita i motori
					//	tone(Pin_LED_TOP_R, 2, 0);
	//MSG("ACCENDI I MOTORI")
	//countDown(5);

	WEBCAM_OFF
	WEBCAM_ON

	LEDTOP_B_OFF
  }



int loopCounter = 1;
void loop() {
	ledSeq1(90);
	nh.spinOnce();

	robotMotors.goCmdVel(0, 0.5);
	delay(3000);
	robotMotors.stop();

	//MSG("====================")
	MSG2("ENC.cnt: ", rob.status.sensors.encRcnt);



	//publish_tf();
	rob.updPoseFromEncoders(); // aggiorna rob.status.posCurrent
	publish_odom();//pubblica in odom rob.status.posCurrent
	//publish_chatter("hello");
	//publish_speech();
	//publish_ultrasound();
	//publish_laserscan(); // acquisisce le distanze...

	//MSG2("pose.x:", rob.status.posCurrent.x);
	//MSG2("pose.y:", rob.status.posCurrent.y);
	//MSG2("pose.r:", rob.status.posCurrent.r);
	

	//testGPS();
	//MSG("======== end ========")
	delay(50);
}



