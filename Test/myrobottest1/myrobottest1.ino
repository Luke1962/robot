/*
   rosserial PubSub Example
   Prints "hello world!" and toggles led
*/

#define SIMULATION_OFF
//#define SIMULATION_ON


#ifndef SIMULATION_ON
	// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	#pragma region CONFIGURAZIONE DEL SISTEMA   
	//#define delay(ms) chThdSleepMilliseconds(ms) 

	#include <MyRobotLibs\dbg.h>

	#include <MyRobotLibs\systemConfig.h>
	#include <MyRobotLibs\hw_config.h>
	//#include <NewTone\NewTone.h>
	#include <Mystepper\myStepper.h>


  #endif





	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  robot                          ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//



	double robot_time = 0;
	unsigned long lastRobot_time;

	long laserscan_time;
	#define LDSmicrostepsPerStep 2
	char scan_frame_id[] = "laser_link";
	#define LDSsamples 5
	#define LDSspeed PI/LDSmicrostepsPerStep        // PI rad/sec = 180deg/sec 
		float f_ranges[LDSsamples]; // max of 30 measurements
		float f_intensities[LDSsamples];

#ifdef SIMULATION_ON
		//--------------------------------
		float robotPose_x = 1.0;
		float robotPose_y = 0.0;
		float robotPose_theta = 1.57;
		float robotPose_linear_vel_x = 0.5;
		float robotPose_angular_vel_x = 0.1;
		//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		float f_angle_min = -PI / 2;//robot.status.parameters.sonarStartAngle
		float f_angle_max = PI / 2;//robot.status.parameters.sonarEndAngle

		// 3.14/4   - 5 measurement points
		float f_angle_increment = PI / LDSsamples;  // (f_angle_max - f_angle_min)/ LDSsamples 
		float f_time_increment= 10;
		float f_scan_time = 2 * LDSmicrostepsPerStep; // LDSmicrostepsPerStep* 2* (f_angle_max-f_angle_min )/LDSspeed
		float f_range_min = 0.02;
		float f_range_max = 2.0;
  
 
 
	void robotMove() {
		if (millis()> robot_time) {
			//ROS_INFO("Robot Postion:(%.2f, %.2f. %.2f)" ,robotPose_x, robotPose_y, robotPose_z);
			//compute odometry in a typical way given the velocities of the robot
			double dt = millis() - lastRobot_time;
			double delta_x = (robot.status.cmd.targetVelocityLinear * cos(robotPose_theta) - 0 * sin(robotPose_theta)) * dt;
			double delta_y = (robot.status.cmd.targetVelocityLinear * sin(robotPose_theta) + 0 * cos(robotPose_theta)) * dt;
			double delta_th = robot.status.cmd.targetVelocityAngular * dt;

			robot.status.posCurrent.x += delta_x;
			robot.status.posCurrent.y += delta_y;
			robot.status.posCurrent.r += delta_th;
 
 
			robotPose_theta += robot.status.cmd.targetVelocityAngular;
			if (robotPose_theta > 3.14)    robotPose_theta = -3.14;

			robotPose_x += cos(robotPose_theta) * robot.status.cmd.targetVelocityLinear * 0.1;
			robotPose_y += sin(robotPose_theta) * robot.status.cmd.targetVelocityLinear * 0.1;

			ROS_INFO2F(" vel  x:", robot.status.cmd.targetVelocityLinear);
			ROS_INFO2F(" vel  z:", robot.status.cmd.targetVelocityAngular);

			robot_time = millis() + (5000);
		}
	}


	#else

		#include <Timer5/Timer5.h>

		myStepper_c myLDSstepper(PIN_STEPPERLDS_CK, PIN_STEPPERLDS_ENABLE, PIN_STEPPERLDS_CW, PIN_STEPPERLDS_HOME);
		String inString = "";    // string to hold input
		float speed = PI;
		#define MINIMUM_INTERRUPT_INTERVAL_MSEC 1

		#include <Wire\Wire.h>
		#include <VL53L0X\VL53L0X.h>
		VL53L0X LDS;
		#include <robot.h>
		struct robot_c robot;	//was  struct robot_c robot;

	void robotMove() {
	}

	#endif
								// laser scan global data-------------------------------------------------------


	void setupRobot() {
		pinMode(13, OUTPUT);
		pinMode(8, OUTPUT);   digitalWrite(8, LOW);
		Serial2.begin(57600);

		while (!Serial2.available()) {
			delay(100);

		}
		robot.status.parameters.sonarStartAngle = -1.57;
		robot.status.parameters.sonarEndAngle =1.57;
		robot.status.parameters.sonarStepAngle=  0.785;  // 3.14/4   = 5 measurement points



		// create the test data
		for (int z = 0; z < 5; z++)
		{
			f_ranges[z] = z;
			f_intensities[z] = z * z;
		}
		//-----------------------------

	}



#pragma region ROS ===============================================

	#include "math.h"
	#include <ros.h>
	#include <std_msgs/String.h>
	#include <std_msgs/Empty.h>
	#include <ros/time.h>
	ros::NodeHandle  nh;





	//-------------------------------------------------
	//per conversione double to char 


	void ROS_INFO2F(char * c, float f) {
	#define CHARFLOATSIZE 10
	#define CHARFLOATDECS 3
		char charVal[CHARFLOATSIZE];  //temporarily holds data from vals
		//4 is mininum width, 3 is precision; float value is copied onto buff
		dtostrf(f, CHARFLOATSIZE, CHARFLOATDECS, charVal);
		nh.loginfo(c);
		nh.loginfo(charVal);
	}


	/*
	String stringVal = "";

	stringVal+=String(int(floatVal))+ "."+String(getDecimal(floatVal)); //combining both whole and decimal part in string with a fullstop between them
	Serial.print("stringVal: ");Serial.println(stringVal);              //display string value

	  char charVal[stringVal.length()+1];                      //initialise character array to store the values
	  stringVal.toCharArray(charVal,stringVal.length()+1);     //passing the value of the string to the character array

	  Serial.print("charVal: ");
	  for(uint8_t i=0; i<sizeof(charVal);i++) Serial.print(charVal[i]); //display character array

	}

	//function to extract decimal part of float
	long getDecimal(float val)
	{
	  int intPart = int(val);
	  long decPart = 1000*(val-intPart); //I am multiplying by 1000 assuming that the foat values will have a maximum of 3 decimal places.
										//Change to match the number of decimal places you need
	  if(decPart>0)return(decPart);           //return the decimal part of float number if it is available
	  else if(decPart<0)return((-1)*decPart); //if negative, multiply by -1
	  else if(decPart=0)return(00);           //return 0 if decimal part of float number is not available
	}
	* */

	//-------------------------------------------------



	#define ROS_INFO(s) nh.loginfo(s);
	//#define ROS_INFO2(s,v) nh.loginfo(s);nh.loginfo(dtoa(v,'10','3',charVal));
	//--------------------------------



	//temporarily holds data from vals

	char ros_info[30];

	unsigned long publisher_timer;



	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  other  command                       --------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//

	//-----------------------------------------------------------------
	void cmdcb_toggleLed(const std_msgs::Empty& toggle_msg) {
		digitalWrite(13, HIGH - digitalRead(13)); // blink the led
	}

	ros::Subscriber<std_msgs::Empty> subMsg("toggle_led", cmdcb_toggleLed);


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




	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  speech		                        ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	char hello[11] = "ciao bello";
	std_msgs::String str_msg_speech;
	ros::Publisher speech("/rp/state_externalization/vocal_message", &str_msg_speech);
	unsigned long speech_time;

	void setup_speech() {
		nh.advertise(speech);
		ROS_INFO("setup_speech");
	}



	void publish_speech() {
		if (millis() >= speech_time) {
			// pubblicazione su chatter e speech-------
			str_msg.data = hello;
			chatter.publish(&str_msg);
			speech.publish(&str_msg);
			ROS_INFO("speech");

			speech_time = millis() + 5000;
			nh.spinOnce();
		}

		//  str_msg.data = itoa(i,str_msg,5);
		//  speech.publish(&str_msg );


	}





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

	char base_link_frame_id[] = "/base_link";
	//char odom[] = "/odom";
	char odom_frame_id[] = "odom";

	unsigned long tf_time;
#ifdef SIMULATION_ON

	// Pubblica  tf ----------------------------------
	void publish_tf() {

		if (millis() > tf_time) {

			// broadcast tf odom->base_link-------------
			t.header.stamp = nh.now();
			t.header.frame_id = odom_frame_id;
			t.child_frame_id = base_link_frame_id;

			t.transform.translation.x = robotPose_x;
			t.transform.translation.y = robotPose_y;
			t.transform.translation.z = 0.0;

			t.transform.rotation = tf::createQuaternionFromYaw(robotPose_theta);


			broadcaster.sendTransform(t);

			ROS_INFO("P. TF");
			//------------------------------------------
			tf_time = millis() + 200;
		}

	}
#else
	// Pubblica  tf ----------------------------------
	void publish_tf() {

		if (millis() > tf_time) {

			// broadcast tf odom->base_link-------------
			t.header.stamp = nh.now();
			t.header.frame_id = odom_frame_id;
			t.child_frame_id = base_link_frame_id;

			t.transform.translation.x = robot.status.posCurrent.x/1000;
			t.transform.translation.y = robot.status.posCurrent.y/1000;
			t.transform.translation.z = 0.0;

			t.transform.rotation = tf::createQuaternionFromYaw(robot.status.posCurrent.r*DEG_TO_RAD);


			broadcaster.sendTransform(t);

			ROS_INFO("P. TF");
			//------------------------------------------
			tf_time = millis() + 200;
		}

	}

#endif




	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	//  odom                                ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//
	unsigned long odom_time;
	#include <tf/transform_broadcaster.h>
	#include <nav_msgs/Odometry.h>
	// solo per simulazione.
	// nel caso di robot vero
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;
	tf::TransformBroadcaster odom_broadcaster;
	nav_msgs::Odometry rosmsg_odom;
	ros::Publisher pub_odom("/odom", &rosmsg_odom);

	void setup_odom() {
		nh.advertise(pub_odom);
		ROS_INFO("setup_odom");
	}
#ifdef SIMULATION_ON


	// Pubblica  odometry  ----------------------------------
	void publish_odom() {
		if (millis() > odom_time) {

			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = nh.now();
			odom_trans.header.frame_id = odom_frame_id;
			odom_trans.child_frame_id = base_link_frame_id;

			odom_trans.transform.translation.x = robotPose_x;
			odom_trans.transform.translation.y = robotPose_y;
			odom_trans.transform.translation.z = 0.0;
			//since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_trans.transform.rotation = tf::createQuaternionFromYaw(robotPose_theta);

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			//nav_msgs::Odometry odom;
			rosmsg_odom.header.stamp = nh.now();;
			rosmsg_odom.header.frame_id = odom_frame_id;

			//set the position
			rosmsg_odom.pose.pose.position.x = robotPose_x;
			rosmsg_odom.pose.pose.position.y = robotPose_y;
			rosmsg_odom.pose.pose.position.z = 0.0;
			rosmsg_odom.pose.pose.orientation = odom_trans.transform.rotation;

			//set the velocity
			rosmsg_odom.child_frame_id = base_link_frame_id;
			rosmsg_odom.twist.twist.linear.x = robot.status.cmd.targetVelocityLinear;
			rosmsg_odom.twist.twist.linear.y = 0;
			rosmsg_odom.twist.twist.angular.z = robot.status.cmd.targetVelocityAngular;

			//publish the message
			pub_odom.publish(&rosmsg_odom);
			ROS_INFO("pub odom");
			odom_time = millis() + 500;
		}
	}

#else


	// Pubblica  odometry  ----------------------------------
	void publish_odom() {
		if (millis() > odom_time) {

			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = nh.now();
			odom_trans.header.frame_id = odom_frame_id;
			odom_trans.child_frame_id = base_link_frame_id;

			odom_trans.transform.translation.x = robot.status.posCurrent.x/1000;
			odom_trans.transform.translation.y = robot.status.posCurrent.y/1000;
			odom_trans.transform.translation.z = 0.0;
			//since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_trans.transform.rotation = tf::createQuaternionFromYaw(robot.status.posCurrent.r*DEG_TO_RAD);

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			//nav_msgs::Odometry odom;
			rosmsg_odom.header.stamp = nh.now();;
			rosmsg_odom.header.frame_id = odom_frame_id;

			//set the position
			rosmsg_odom.pose.pose.position.x = robot.status.posCurrent.x / 1000;
			rosmsg_odom.pose.pose.position.y = robot.status.posCurrent.y / 1000;
			rosmsg_odom.pose.pose.position.z = 0.0;
			rosmsg_odom.pose.pose.orientation = odom_trans.transform.rotation;

			//set the velocity
			rosmsg_odom.child_frame_id = base_link_frame_id;
			rosmsg_odom.twist.twist.linear.x = robot.status.cmd.targetVelocityLinear;
			rosmsg_odom.twist.twist.linear.y = 0;
			rosmsg_odom.twist.twist.angular.z = robot.status.cmd.targetVelocityAngular;

			//publish the message
			pub_odom.publish(&rosmsg_odom);
			ROS_INFO("pub odom");
			odom_time = millis() + 500;
		}
	}


#endif // SIMULATION_ON








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
	//  laser scan                          ---------------------------------//
	//-----------------------------------------------------------------------//
	//-----------------------------------------------------------------------//

	#include <sensor_msgs/LaserScan.h>  // LDS
	sensor_msgs::LaserScan rosmsg_laserscan;
	ros::Publisher pub_Laser("scan", &rosmsg_laserscan);




#ifndef SIMULATION_ON
	long f_time_increment = 10;
	long f_scan_time = 4;

	void setup_laserscan() {
		nh.advertise(pub_Laser);
		// UdM in deg
		robot.status.parameters.sonarStartAngle = -90;
		robot.status.parameters.sonarEndAngle = 90;
		robot.status.parameters.sonarStepAngle = 5;	//f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
		robot.status.parameters.sonarMinDistance = 0.1;
		robot.status.parameters.sonarMaxDistance = 2;
		f_time_increment = 10;
		f_scan_time = 4;
 
 



		rosmsg_laserscan.ranges_length = 5;
		rosmsg_laserscan.intensities_length = 5;
		rosmsg_laserscan.header.frame_id = scan_frame_id;
		// create the test data
		for (int z = 0; z < 5; z++)
		{
			f_ranges[z] = z;
			f_intensities[z] = z*z;
		}
		nh.advertise(pub_Laser);
	}

	void publish_laserscan() {

		if (millis() > laserscan_time)
		{
			// imposta la velocità
			myLDSstepper.goRadsPerSecond(2 * PI);
			// attende che arrivi in home
			while (!myLDSstepper.isHomePosition()) { delay(200); }


			rosmsg_laserscan.header.stamp = nh.now();
			rosmsg_laserscan.header.frame_id = scan_frame_id; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			rosmsg_laserscan.angle_min = robot.status.parameters.sonarStartAngle*DEG_TO_RAD;
			rosmsg_laserscan.angle_max = robot.status.parameters.sonarEndAngle*DEG_TO_RAD;
			rosmsg_laserscan.angle_increment = robot.status.parameters.sonarStepAngle*DEG_TO_RAD;
			rosmsg_laserscan.time_increment = 10;
			rosmsg_laserscan.scan_time = 10;
			rosmsg_laserscan.range_min = robot.status.parameters.sonarMaxDistance/100;
			rosmsg_laserscan.range_max = 2.0;

			for (int z = 0; z < 5; z++)
			{
				//rosmsg_laserscan.ranges[z] = f_ranges[z];
				rosmsg_laserscan.ranges[z] = robot.getLDSDistance();
 
			}


			pub_Laser.publish(&rosmsg_laserscan);

			//           nh.spinOnce();
			ROS_INFO("P. laser");
			laserscan_time = millis() + 5000;

		}

	}

#else

	void setup_laserscan() {
		nh.advertise(pub_Laser);

		f_angle_min = -PI / 2; // -1.57;
		f_angle_max = PI / 2; // 1.57;
		f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
		f_time_increment = 10;
		f_scan_time = 4;
		f_range_min = 0.1;
		f_range_max = 2;



		rosmsg_laserscan.ranges_length = 5;
		rosmsg_laserscan.intensities_length = 5;
		rosmsg_laserscan.header.frame_id = scan_frame_id;
		// create the test data
		for (int z = 0; z < 5; z++)
		{
			f_ranges[z] = z;
			f_intensities[z] = z*z;
		}
		nh.advertise(pub_Laser);
	}


	void publish_laserscan() {

		if (millis() > laserscan_time)
		{
 

			rosmsg_laserscan.header.stamp = nh.now();
			rosmsg_laserscan.header.frame_id = scan_frame_id; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			rosmsg_laserscan.angle_min = f_angle_min;
			rosmsg_laserscan.angle_max = f_angle_max;
			rosmsg_laserscan.angle_increment = f_angle_increment;
			rosmsg_laserscan.time_increment = f_time_increment;
			rosmsg_laserscan.scan_time = f_scan_time;
			rosmsg_laserscan.range_min = f_range_min;
			rosmsg_laserscan.range_max = f_range_max;

			for (int z = 0; z < 5; z++)
			{
				//rosmsg_laserscan.ranges[z] = f_ranges[z];
				CDLaser_msg.ranges[z] = z;
  
			}

			//for (int z = 0; z<5; z++)
			//{
			//  rosmsg_laserscan.intensities[z] = f_intensities[z];//If your device does not provide intensities, please leave the array empty.
			//}

			pub_Laser.publish(&rosmsg_laserscan);

			//           nh.spinOnce();
			ROS_INFO("P. laser");
			laserscan_time = millis() + 5000;

		}

	}

#endif



#pragma endregion










// ---------------------------------------------
// main		 ----------------------------------
// ---------------------------------------------





void setup()
{
	setupRobot();

	#pragma region ROS initialization
		nh.initNode();
		broadcaster.init(nh);        
		ROS_INFO( "ROBOT SETUP COMPLETE");

		setup_odom();


		
		setup_chatter();
		
		setup_speech();  
			 
		setup_ultrasound();
		
		setup_laserscan();
		
		
		setup_cmd_vel();
		
		nh.spinOnce();

		ROS_INFO( "ROS SETUP COMPLETE");
  
	#pragma endregion
	
}


void loop()
{
	publish_tf();
	publish_odom();
	publish_chatter("hello");
	publish_speech();
	publish_ultrasound();
	publish_laserscan(); // acquisisce le distanze...

	digitalWrite(13, 1);
	delay(500);
	digitalWrite(13, 0);
	nh.spinOnce();

	robotMove();
	
	
	delay(500);
}
