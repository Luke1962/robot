// test di pubblicazione  dati LDS su topic /scandata



//////////////////////////////////////////////////////////////////////
// 
// se definito non inizializza i sensori e usa le funzioni di acquisizione simulate
//#define SIMULATION_ON


//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
	//#define delay(ms) chThdSleepMilliseconds(ms) 

	#include <Arduino.h>	//per AttachInterrupt
	#include <MyRobotLibs\dbg.h>
	#include <MyRobotLibs\systemConfig.h>
	#include <MyRobotLibs\hw_config.h>

	#include <Timer5/Timer5.h>
	#include <PinChangeInt\PinChangeInt.h>	//https://github.com/NicoHood/PinChangeInterrupt

	//#include <NewTone\NewTone.h>

 
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

	#include <MyStepper\myStepper.h>
	myStepper_c myLDSstepper(PIN_STEPPERLDS_CK, PIN_STEPPERLDS_ENABLE, PIN_STEPPERLDS_CW, PIN_STEPPERLDS_HOME);
	String inString = "";    // string to hold input
	float speed = PI;
#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1

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
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	//static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>
	// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
	//------------------------------------------------------------------------------
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





		/// ///////////////////////////////////////////////////////////////////////////////
		// M U T E X ///////////////////////////////////
		/// ///////////////////////////////////////////////////////////////////////////////
		// Mutex for atomic access to data.
		MUTEX_DECL(mutexMotion); //condiviso dai comandi di movimento e sonar per non essere in movimento quando il sonar scansiona
		MUTEX_DECL(mutexSerialMMI);// accesso alla seriale
		MUTEX_DECL(mutexSerialPC);// accesso alla seriale
		MUTEX_DECL(mutexSensors);// accesso ai sensori in lettura e scrittura


	#pragma endregion DEFINIZIONE MAILBOX VOICE




	// ////////////////////////////////////////////////////////////////////////////////////////////
	// INTERRUPT SERVICE ROUTINES
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



	byte ckState = 0;
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
		digitalWriteFast(13, ckState);

		digitalWriteFast(PIN_STEPPERLDS_CK, ckState);
	}


	void ISRstepperSwitchHome() {
		static unsigned long last_interrupt_time = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_time > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{
			int x = digitalReadFast(PIN_STEPPERLDS_HOME);
			if (x == 0)
			{
				myLDSstepper.setHomePosition(true);
				LEDTOP_B_ON;
				myLDSstepper.setCW(true);  ///.disable();

			}
			else
			{
				myLDSstepper.setHomePosition(false);
				LEDTOP_B_OFF;
				myLDSstepper.enable();

			}

		}
		last_interrupt_time = interrupt_time;
	}

	void ISRstepperSwitchEnd() {
		static unsigned long last_interrupt_time = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_time > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{
			int x = digitalReadFast(PIN_STEPPERLDS_END);
			if (x == 0)
			{
				//myLDSstepper.setHomePosition(true);
				LEDTOP_G_ON;
				myLDSstepper.setCW(false);  ///.disable();

			}
			else
			{
				//myLDSstepper.setHomePosition(false);
				LEDTOP_G_OFF;
				myLDSstepper.enable();

			}

		}
		last_interrupt_time = interrupt_time;
	}



#pragma endregion Creazione oggetti Globali
 

#pragma region ROS Libraries and Variables

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
	geometry_msgs::TransformStamped t;
	tf::TransformBroadcaster broadcaster;

	//--------------------------------
	std_msgs::String str_msg;
	ros::Publisher chatter("chatter", &str_msg);

	//--------------------------------
	sensor_msgs::Range rosmsg_range;
	ros::Publisher pub_range("ultrasound", &rosmsg_range);

	//--------------------------------
	sensor_msgs::LaserScan CDLaser_msg;;
	ros::Publisher pub_Laser("scan", &CDLaser_msg);

	//--------------------------------
	geometry_msgs::Twist msg;
	void commandCallback(const geometry_msgs::Twist& cmd_vel);
	ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

	//--------------------------------
	ros::Publisher speech("/rp/state_externalization/vocal_message", &str_msg);

	//--------------------------------
	#define ROS_INFO(s) nh.loginfo(s);

	// laser data---------------
	#define LDSmicrostepsPerStep 2

	#define LDSsamples 5
	#define LDSspeed PI/LDSmicrostepsPerStep		// PI rad/sec = 180°/sec 
	float f_angle_min = - PI/2 ;//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	float f_angle_max = PI/2 ;
	float f_angle_increment= PI/LDSsamples;  // (f_angle_max - f_angle_min)/ LDSsamples 
	float f_time_increment;
	float f_scan_time= 2 * LDSmicrostepsPerStep; // LDSmicrostepsPerStep* 2* (f_angle_max-f_angle_min )/LDSspeed
	float f_range_min =0.02;
	float f_range_max =2.0;
	float f_ranges[LDSsamples]; // max of 30 measurements
	float f_intensities[LDSsamples];
	//------------------------

	//temporarily holds data from vals
	char charVal[10];

	char hello[13] = "hello world!";

	char base_link[] = "/base_link";
	char odom[] = "/odom";
	char frameid[] = "/ultrasound";

	unsigned long publisher_timer;
	unsigned long range_time;
	unsigned long tf_time;

 
	float move1;
	float move2;

	const int adc_pin = 0;
	double x = 1.0;
	double y = 0.0;
	double theta = 1.57;
	double g_req_angular_vel_z = 0;
	double g_req_linear_vel_x = 0;
	unsigned long g_prev_command_time = 0;




	//-----------------------------------------------------------------

	/// ///////////////////////////////////////////////////////////////////////////////
	void publish_chatter(char* charVal) {


		//str_msg.data = strcat("ultra sound:", charVal);
		str_msg.data = charVal;
		chatter.publish(&str_msg);
		nh.spinOnce();

	}


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

				int speed =(int)( cmd_vel.angular.z * ROBOT_MOTOR_STEPS_PER_RADIANT);
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

		float getRange_Ultrasound_Simulated(int pin_num) {
			int val = 0;
			for (int i = 0; i < 4; i++) val += analogRead(pin_num);
			float range = val;
			return range / 322.519685;   // (0.0124023437 /4) ; //cvt to meters
		}


		void publish_tf() {  //versione simulata
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
				*/
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
					t.transform.translation.x = robot.status.cmd.stepsDone/ROBOT_MOTOR_STEPS_PER_CM/100;

				}
				if (robot.status.cmd.commandDir == commandDir_e::GOB)
				{
					t.transform.translation.x = -robot.status.cmd.stepsDone/ROBOT_MOTOR_STEPS_PER_CM/100;

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
					t.transform.rotation.z =- DEG_TO_RAD* robot.status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_DEG;
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

	void publish_laserscan() {
		/*
		min_height_(0.10),
		max_height_(0.15),
		angle_min_(-M_PI/2),
		angle_max_(M_PI/2),
		angle_increment_(M_PI/180.0/2.0),
		scan_time_(1.0/30.0),
		range_min_(0.45),
		range_max_(10.0),
		output_frame_id_("/kinect_depth_frame")

		//Got data in.
		//printf("[%s]\n", msg->data.c_str());


		//pub_laserscan->header.stamp = ros::Time::now();
		//pub_laserscan->header.seq = id++;
		//pub_laserscan->header.frame_id = "laser"; //associated with laser
		//pub_laserscan->angle_min = -1.04719755; //-60 degrees
		//pub_laserscan->angle_max = 1.04719755; //+60 degrees
		//pub_laserscan->angle_increment = 0.0872664626; //5 Degrees
		//pub_laserscan->time_increment = 0.0; //tbc
		//pub_laserscan->scan_time = (2.0); //tbc
		//pub_laserscan->range_min = 0.01; //1cm
		//pub_laserscan->range_max = 3.0; //3m


		//float32[] ranges;

		////Prove we can split the data
		//std::istringstream oss(std::string(msg->data.c_str()));
		//std::string word;
		//int index = 0;
		//while (getline(oss, word, ',')) {
		//	pub_laserscan->ranges[index] = atof(word.c_str()) / 100.0;
		//	//printf("%s\n", word.c_str());
		//	index++;
		//}
		//


		*/

		if (millis() > publisher_timer)
		{
			// imposta la velocità
			myLDSstepper.goRadsPerSecond(2 * PI);

			// attende che arrivi in home
			while (!myLDSstepper.isHomePosition()) { delay(200); }

			CDLaser_msg.header.stamp = nh.now();
			CDLaser_msg.header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			CDLaser_msg.angle_min = f_angle_min;
			CDLaser_msg.angle_max = f_angle_max;
			CDLaser_msg.angle_increment = f_angle_increment;
			CDLaser_msg.time_increment = f_time_increment;
			CDLaser_msg.scan_time = f_scan_time;
			CDLaser_msg.range_min = f_range_min;
			CDLaser_msg.range_max = f_range_max;

			for (int z = 0; z<5; z++)
			{
				//CDLaser_msg.ranges[z] = f_ranges[z];
				#ifdef SIMULATION_ON
					CDLaser_msg.ranges[z] = z;

				#else
					CDLaser_msg.ranges[z] = robot.getLDSDistance();

				#endif // SIMULTION_ON

			}

			//for (int z = 0; z<5; z++)
			//{
			//	CDLaser_msg.intensities[z] = f_intensities[z];//If your device does not provide intensities, please leave the array empty.
			//}

			publisher_timer = millis() + 5000;
			pub_Laser.publish(&CDLaser_msg);
			nh.spinOnce();

		}

	}

#pragma endregion ROS


#pragma region Helper Utilities
	void countDown(int seconds) {
		MSG3("CountDown in ", seconds, " sec...");
		for (size_t i = seconds; i > 0; i--)
		{
			MSG2("  -", i);
			delay(1000);
		}
	}
	#define sgn(x) ((x > 0) - (x < 0))
	void ledSeq1() {
		LEDTOP_R_ON	// Indica inizio SETUP Phase
			delay(1000);
		LEDTOP_G_ON	// Indica inizio SETUP Phase
			delay(1000);

		LEDTOP_B_ON	// Indica inizio SETUP Phase
			delay(1000);

		LEDTOP_R_OFF
		LEDTOP_G_OFF

	}
#pragma endregion


	void setupRobot() {
		SERIAL_ROS.begin(SERIAL_ROS_BAUD_RATE);
		SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
		SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
		ledSeq1();

		robot.initHW(); //disabilita i motori
		MSG("TESTROSLASERSCAN v0.1");

		MSG3("A0 (Vbat) : ", analogRead(A0), "/1024");
		tone(Pin_LED_TOP_R, 2, 0);
		countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE
		Wire.begin(); // default timeout 1000ms
		robot.initRobot();
		noTone(Pin_LED_TOP_R);
		LEDTOP_G_ON	// Indica inizio SETUP Phase


		MSG("ACCENDI I MOTORI");
		LDS.init();
		//compass.begin();

		robot.initCompass(&compass);



		pinMode(PIN_STEPPERLDS_HOME, INPUT_PULLUP);// open >+5 closed =gnd
		pinMode(PIN_STEPPERLDS_END, INPUT_PULLUP);// open >+5 closed =gnd
		attachPinChangeInterrupt(PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, CHANGE);  // add more attachInterrupt code as required
		attachPinChangeInterrupt(PIN_STEPPERLDS_END, ISRstepperSwitchEnd, CHANGE);  // add more attachInterrupt code as required
		
		// start stepper
		myLDSstepper.goRadsPerSecond(speed);




 

	}

#pragma region Setup_ROS


void setupUltrasound() {
	rosmsg_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
	rosmsg_range.header.frame_id = frameid;
	rosmsg_range.field_of_view = 0.1;  // fake
	rosmsg_range.min_range = 0.0;
	rosmsg_range.max_range = 6.47;

}
void setupLaserScan() {
	nh.advertise(pub_Laser);

	f_angle_min = -PI / 2; // -1.57;
	f_angle_max = PI / 2; // 1.57;
	f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
	f_time_increment = 10;
	f_scan_time = 4;
	f_range_min = 0.1;
	f_range_max = 2;

	CDLaser_msg.ranges_length = 5;
	CDLaser_msg.intensities_length = 5;

	// create the test data
	for (int z = 0; z < 5; z++)
	{
		f_ranges[z] = z;
		f_intensities[z] = z*z;
	}
}


#pragma endregion

void setup()
{
	setupRobot();

	#pragma region ROS initialization
		nh.initNode();
		broadcaster.init(nh);

		nh.advertise(chatter);

		setupUltrasound();
		nh.advertise(pub_range);

		setupLaserScan();
		nh.advertise(pub_Laser);

	#pragma endregion

}


void loop()
{

	publish_range();
	publish_chatter("Hello");

	publish_laserscan(); // acquisisce le distanze...



	digitalWrite(13, 1);
	delay(500);
	digitalWrite(13, 0);


}

