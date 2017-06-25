/*
* rosserial Time and TF Example
* Publishes a transform at current time
*/


//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
	//#define delay(ms) chThdSleepMilliseconds(ms) 

	#include <MyRobotLibs\dbg.h>
	#include <MyRobotLibs\systemConfig.h>
	#include <MyRobotLibs\hw_config.h>
#pragma endregion

#pragma region ROS Libraries and Variables

	/// ///////////////////////////////////////////////////////////////////////////////
	// ROS
	/// ///////////////////////////////////////////////////////////////////////////////
	#include <math.h>

	#include <ros.h>
	#include <ros/time.h>
	#include <std_msgs/String.h>
	#include <sensor_msgs/Range.h>
	#include <tf/transform_broadcaster.h>

	ros::NodeHandle  nh;
	//ros::NodeHandle_<ArduinoHardware, number of subscribers ,publishers,  input buffer in bytes, output buffer in bytes> nh;
	
	//ros::NodeHandle_<ArduinoHardware, 2, 5, 125, 125> nh;

	/// TF BROADCASTER  ///////////////////////////////////////////////////////////////////////////////
	geometry_msgs::TransformStamped tf_odom;
	tf::TransformBroadcaster broadcaster;


	/// PUBLISHERS  ///////////////////////////////////////////////////////////////////////////////
	std_msgs::String str_msg;
	ros::Publisher pub_chatter("/pub_chatter", &str_msg);



	char hello[13] = "hello world!";

	char frameid_LDS[] = "/ultrasound";
	char frameid_base_link[] = "/base_link";
	char frameid_odom[] = "/odom";	//id del frame odometry

	long range_time;
	long tf_time;

	const int adc_pin = 0;

	sensor_msgs::Range range_msg;
	ros::Publisher pub_range("/ultrasound", &range_msg);


	/// ///////////////////////////////////////////////////////////////////////////////
	/// SUBSCRIBERS ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	/*
	1. includi qui i .h dei messaggi
	2. metti qui le funzioni di callback void rosCb_cmd_vel(const std_msgs::Float64& msg)
	3. metti qui ros::Subscriber<std_msgs::Float64> sub_my1("your_topic", &messageCb);
	4. in Setup() metti nh.subscribe(sub_my1);*/
	
	///The first step in subscribing to a topic is including the header file and creating the call back function.
	#include <std_msgs/Float64.h>
	#include <ros_lib\turtle_actionlib\Velocity.h>
	#include <ros_lib\geometry_msgs\Twist.h>
	#include <Servo.h>

	// procedura di callback----------------------------
	void rosCb_cmd_vel(const geometry_msgs::Twist cmdVel)
	{

		if ( (cmdVel.angular.z != 0.0) || ( cmdVel.linear.x !=0.0))
		{
			digitalWrite(13, 1);
			//		robot.go(commandDir_e::GOF, cmdVel.x);
		}
		else
		{
			digitalWrite(13, 0);

		}


	}

	Servo servo;
	void servo_cb(const geometry_msgs::Twist &cmd_msg) {
		servo.write(cmd_msg.linear.x); //set servo angle, should be from 0-180  
		digitalWrite(13, HIGH - digitalRead(13));  //toggle led  
	}

	//Subscriber(const char * topic_name, CallbackT cb, int endpoint = rosserial_msgs::TopicInfo::ID_SUBSCRIBER) :

	//ok ? ros::Subscriber<geometry_msgs::Twist> subs_cmd_vel("servo", servo_cb,0);
	const char subscribedTopicName[]   = "serial_node";
	ros::Subscriber<geometry_msgs::Twist> subs_cmd_vel("servo", servo_cb);
	//ros::Subscriber<geometry_msgs::Twist> subs_cmd_vel(subscribedTopicName, rosCb_cmd_vel);

	//ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

	/// ///////////////////////////////////////////////////////////////////////////////

#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
	#include <digitalWriteFast.h>
	#include <ChibiOS_AVR.h>
	#include <util/atomic.h>
		//#include <TinyGPSplus/TinyGPS++.h>
		//#include <StackArray.h>
	#include <Arduino.h>	//per AttachInterrupt


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


	//// ////////////////////////////////////////////////////////////////////////////////////////////
	////  CmdMessenger object to the default Serial port
	//// ////////////////////////////////////////////////////////////////////////////////////////////
	//#include <CmdMessenger2/CmdMessenger2.h>
	//	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	//	static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
	//#include <MyRobotLibs\RobotInterfaceCommands2.h>
	//	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//	//------------------------------------------------------------------------------





		/// ///////////////////////////////////////////////////////////////////////////////
		// M U T E X ///////////////////////////////////
		/// ///////////////////////////////////////////////////////////////////////////////
		// Mutex for atomic access to data.
		MUTEX_DECL(mutexMotion); //condiviso dai comandi di movimento e sonar per non essere in movimento quando il sonar scansiona
		MUTEX_DECL(mutexSerialMMI);// accesso alla seriale
		MUTEX_DECL(mutexSerialPC);// accesso alla seriale
		MUTEX_DECL(mutexSensors);// accesso ai sensori in lettura e scrittura



								 // ////////////////////////////////////////////////////////////////////////////////////////////
		volatile bool interruptFlag = 0;
		// Interrupt service routines for the right motor's quadrature encoder
		void ISRencoder()
		{
			//	noInterrupts();
			robot.status.cmd.stepsDone += 25;  //8 tacche per 200 step al giro
											   //digitalWriteFast(Pin_LED_TOP_G, interruptFlag);
											   //digitalWriteFast(13, interruptFlag);
											   //interruptFlag = !interruptFlag;
											   //	interrupts();

		}



		double theta;

#pragma endregion

#pragma region HelperFunctions

	void countDown(int seconds) {
		MSG3("CountDown in ", seconds, " sec...");
		for (size_t i = seconds; i > 0; i--)
		{
			MSG2("  -", i);
			delay(1000);
		}
	}



#pragma endregion


void setup()
{

	#pragma region ROS initialization
		nh.initNode();
		broadcaster.init(nh);

		// messaggi ai quali sottoscrivere
		//nh.subscribe(subs_cmd_vel);

		// messaggi da pubblicare ------
		nh.advertise(pub_chatter);
		nh.advertise(pub_range);
 



		range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
		range_msg.header.frame_id = frameid_LDS;
		range_msg.field_of_view = 0.1;  // fake
		range_msg.min_range = 0.0;
		range_msg.max_range = 6.47;

	#pragma endregion

	LEDTOP_R_ON	// Indica inizio SETUP Phase


	robot.initHW(); //disabilita i motori
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	ROSMSG("ROBOTCORE v0.2");

	WEBCAM_ON
//	MSG3("Bat : ", robot.readBattChargeLevel(), "%");
	tone(Pin_LED_TOP_R, 2, 0);
	countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE
	Wire.begin(); // default timeout 1000ms
	robot.initRobot();
	noTone(Pin_LED_TOP_R);
	LEDTOP_G_ON	// Indica inizio SETUP Phase


	ROSMSG("ACCENDI I MOTORI");
	LDS.init();
	//compass.begin();

	robot.initCompass(&compass);

	pinMode(8, OUTPUT);
	digitalWrite(8, LOW);
}

//temporarily holds data from vals
char charVal[10];



void loop()
{
	//publish the adc value every 50 milliseconds
	//since it takes that long for the sensor to stablize
	if (millis() >= range_time) {
		int r = 0;

		///-------------------------------------------------------------
		// ROS Pubblicazione range_msg 
		///-------------------------------------------------------------

		range_msg.range = (float)robot.getLDSDistanceCm() / 100;
		range_msg.header.stamp = nh.now();
		pub_range.publish(&range_msg);

		range_time = millis() + 500;

		///-------------------------------------------------------------
		// ROS Pubblicazione chatter 
		///-------------------------------------------------------------
		//4 is mininum width, 3 is precision; float value is copied onto buff
		dtostrf(range_msg.range, 4, 3, charVal);
		str_msg.data = charVal;
		pub_chatter.publish(&str_msg);
		//-------------------------------------------------------------


		#pragma region tfRobot
		//if (millis() >= tf_time) {

		///-------------------------------------------------------------
		// ROS Pubblicazione chatter 
		///-------------------------------------------------------------

		// drive in a circle
		double dx = 0.2;
		double dtheta = 0.18;
		robot.status.posCurrent.x += cos(theta)*dx*0.1;
		robot.status.posCurrent.y += sin(theta)*dx*0.1;
		robot.status.posCurrent.r = robot.readCompassDeg();
		theta += dtheta*0.1;
		if (theta > 3.14)
			theta = -3.14;

		// tf odom->base_link
		tf_odom.header.frame_id = frameid_odom;
		tf_odom.child_frame_id = frameid_base_link;

		tf_odom.transform.translation.x = (float)robot.status.posCurrent.x / 1000;
		tf_odom.transform.translation.y = (float)robot.status.posCurrent.y / 1000;

		//t.transform.rotation = tf::TransformBroadcaster::createQuaternionFromYaw();
		tf_odom.transform.rotation.x = 0.0;
		tf_odom.transform.rotation.y = 0.0;
		tf_odom.transform.rotation.z = DEG_TO_RAD *robot.status.posCurrent.r;
		tf_odom.transform.rotation.w = 1.0;
		tf_odom.header.stamp = nh.now();

		
		broadcaster.sendTransform(tf_odom);





		tf_time = millis() + 1000;


		//	}
		#pragma endregion



		nh.spinOnce();
		digitalWrite(13, 0);
		delay(500);
	}

 }
