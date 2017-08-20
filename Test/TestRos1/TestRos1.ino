/*
rosserial PubSub Example
Prints "hello world!" and toggles led
*/


#define SIMULATION_ON


#pragma region ROBOT REGION

//-----------------------------------------------------------------------//
//-----------------------------------------------------------------------//
//  robot                          ---------------------------------//
//-----------------------------------------------------------------------//
//-----------------------------------------------------------------------//
	#include <Wire.h>
	#pragma region Compass
		#include <HMC5883L/HMC5883L.h>

		HMC5883L compass;
		void setup_Compass() {
			// Initialize Initialize HMC5883L
			Serial.println("Initialize HMC5883L");
			while (!compass.begin())
			{
				Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
				delay(500);
			}

			// Set measurement range
			compass.setRange(HMC5883L_RANGE_1_3GA);

			// Set measurement mode
			compass.setMeasurementMode(HMC5883L_CONTINOUS);

			// Set data rate
			compass.setDataRate(HMC5883L_DATARATE_30HZ);

			// Set number of samples averaged
			compass.setSamples(HMC5883L_SAMPLES_8);

			// Set calibration offset. See HMC5883L_calibration.ino
			compass.setOffset(0, 0);

		}
	#pragma endregion


	#pragma region MPU
		#include <MPU6050/MPU6050.h>
		MPU6050 mpu;

		void setup_MPU() {
			while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
			{
				Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
				delay(500);
			}
			// If you have GY-86 or GY-87 module.
			// To access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode


			mpu.setI2CMasterModeEnabled(false);
			mpu.setI2CBypassEnabled(true);
			mpu.setSleepEnabled(false);
		}


		void readMPUCompass(float *accX, float *accY, float *gyrZ, float * mField) {
			// Read normalized values 
			//Vector normAccel = mpu.readNormalizeAccel();
			Vector rawAccel = mpu.readRawAccel();
			Vector rawGyro = mpu.readRawGyro();

			*accX = rawAccel.XAxis;
			*accY = rawAccel.YAxis;
			*gyrZ = rawGyro.ZAxis;

			// Calculate heading
			Vector norm = compass.readNormalize();
			float heading = atan2(norm.YAxis, norm.XAxis);
			float declinationAngle = (2.0 + (22.0 / 60.0)) / (180 / M_PI); //ok per milano
			heading += declinationAngle;
			// Correct for heading < 0deg and heading > 360deg
			if (heading < 0)
			{
				heading += 2 * PI;
			}

			if (heading > 2 * PI)
			{
				heading -= 2 * PI;
			}


			*mField =heading;
		}
	#pragma endregion

	float f_angle_min = -1.57;
	float f_angle_max = 1.57;
	float f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
	float f_time_increment = 10;
	float f_scan_time = 4;
	float f_range_min = 0.1;
	float f_range_max = 30;

	void setupRobot() {
		pinMode(13, OUTPUT);
		pinMode(8, OUTPUT);   digitalWrite(8, LOW);

		setup_MPU();
		setup_Compass();


		f_angle_min = -1.57;
		f_angle_max = 1.57;
		f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
		f_time_increment = 10;
		f_scan_time = 4;
		f_range_min = 0.1;
		f_range_max = 30;

		CDLaser_msg.ranges_length = 5;
		CDLaser_msg.intensities_length = 5;

		// create the test data
		for (int z = 0; z < 5; z++)
		{
			f_ranges[z] = z;
			f_intensities[z] = z * z;
		}
		//-----------------------------

	}
	double robot_time = 0;
	unsigned long lastRobot_time;
	void robotMove() {
		if (millis() > robot_time) {
			//ROS_INFO("Robot Postion:(%.2f, %.2f. %.2f)" ,robotPose_x, robotPose_y, robotPose_z);
			//compute odometry in a typical way given the velocities of the robot
			double dt = millis() - lastRobot_time;
			double delta_x = (robotPose_linear_vel_x * cos(robotPose_theta) - 0 * sin(th)) * dt;
			double delta_y = (robotPose_linear_vel_x * sin(robotPose_theta) + 0 * cos(th)) * dt;
			double delta_th = robotPose_angular_vel_z * dt;

			x += delta_x;
			y += delta_y;
			th += delta_th;



			robotPose_theta += robotPose_angular_vel_z;
			if (robotPose_theta > 3.14)    robotPose_theta = -3.14;

			robotPose_x += cos(robotPose_theta) * robotPose_linear_vel_x * 0.1;
			robotPose_y += sin(robotPose_theta) * robotPose_linear_vel_x * 0.1;

			ROS_INFO2F(" vel  x:", robotPose_linear_vel_x);
			ROS_INFO2F(" vel  z:", robotPose_angular_vel_z);

			robot_time = millis() + (5000);
		}
	}
 



#pragma endregion



#pragma region ROS REGION


#include "math.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
ros::NodeHandle  nh;


float robotPose_x = 1.0;
float robotPose_y = 0.0;
float robotPose_theta = 1.57;
float robotPose_linear_vel_x = 0.5;
float robotPose_angular_vel_z = 0.1;



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


char base_link_frame_id[] = "/base_link";
char ultrasound_frame_id[] = "/ultrasound";
char odom_frame_id[] = "/odom";

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



#pragma region ROS cmdVel

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
	robotPose_linear_vel_x = cmd_vel.linear.x;
	robotPose_angular_vel_z = cmd_vel.angular.z;

	twist_command_time = millis();


	ROS_INFO2F("cmd_vel  x:", robotPose_linear_vel_x);
	ROS_INFO2F("cmd_vel  z:", robotPose_angular_vel_z);
}


void setup_cmd_vel() {
	nh.subscribe(command_cmd_vel);
	ROS_INFO("setup_cmd_vel");
}

#pragma endregion


#pragma region ROS speech


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



#pragma endregion

#define TF 0
#if TF

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

char base_link_frame_id[] = "/base_link";
char odom_frame_id[] = "/odom";

unsigned long tf_time;

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



#pragma endregion


#endif // 0

#pragma region ROS odom

//-----------------------------------------------------------------------//
//-----------------------------------------------------------------------//
//  odom                                ---------------------------------//
//-----------------------------------------------------------------------//
//-----------------------------------------------------------------------//
unsigned long odom_time;
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
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
// Pubblica  odometry  ----------------------------------
void publish_odom() {
	if (millis() > odom_time) {

#if TF
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


#endif // 0

		//next, we'll publish the odometry message over ROS
		//nav_msgs::Odometry odom;
		rosmsg_odom.header.stamp = nh.now();;
		rosmsg_odom.header.frame_id = odom_frame_id;

		//set the position
		rosmsg_odom.pose.pose.position.x = robotPose_x;
		rosmsg_odom.pose.pose.position.y = robotPose_y;
		rosmsg_odom.pose.pose.position.z = 0.0;
		//rosmsg_odom.pose.pose.orientation = odom_trans.transform.rotation;
		rosmsg_odom.pose.pose.orientation.w = 1.0;
		rosmsg_odom.pose.pose.orientation.x = 0.0;
		rosmsg_odom.pose.pose.orientation.y = 0.0;
		rosmsg_odom.pose.pose.orientation.z = robotPose_theta;

		//set the velocity
		rosmsg_odom.child_frame_id = base_link_frame_id;
		rosmsg_odom.twist.twist.linear.x = robotPose_linear_vel_x;
		rosmsg_odom.twist.twist.linear.y = 0;
		rosmsg_odom.twist.twist.angular.z = robotPose_angular_vel_z;

		//publish the message
		pub_odom.publish(&rosmsg_odom);
		ROS_INFO("pub odom");
		odom_time = millis() + 500;
	}
}



#pragma endregion


#pragma region ROS ultrasound



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
	rosmsg_range.max_range = 6.47;
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

#pragma endregion

#pragma region ROS laserscan

//-----------------------------------------------------------------------//
//-----------------------------------------------------------------------//
//  laser scan                          ---------------------------------//
//-----------------------------------------------------------------------//
//-----------------------------------------------------------------------//

#include <sensor_msgs/LaserScan.h>  // LDS
sensor_msgs::LaserScan CDLaser_msg;
ros::Publisher pub_Laser("scan", &CDLaser_msg);

// laser scan global data-------------------------------------------------------
long laserscan_time;
#define LDSmicrostepsPerStep 2
char scan_frame_id[] = "laser_link";
#define LDSsamples 5
#define LDSspeed PI/LDSmicrostepsPerStep        // PI rad/sec = 180deg/sec 
//float f_angle_min = -PI / 2;//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
//float f_angle_max = PI / 2;
//float f_angle_increment = PI / LDSsamples;  // (f_angle_max - f_angle_min)/ LDSsamples 
//float f_time_increment;
//float f_scan_time = 2 * LDSmicrostepsPerStep; // LDSmicrostepsPerStep* 2* (f_angle_max-f_angle_min )/LDSspeed
//float f_range_min = 0.02;
//float f_range_max = 2.0;
//float f_ranges[LDSsamples]; // max of 30 measurements
//float f_intensities[LDSsamples];
//--------------------------------



void setup_laserscan() {
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
	CDLaser_msg.header.frame_id = scan_frame_id;
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
#ifndef SIMULATION_ON
		// imposta la velocità
		myLDSstepper.goRadsPerSecond(2 * PI);
		// attende che arrivi in home
		while (!myLDSstepper.isHomePosition()) { delay(200); }
#endif

		CDLaser_msg.header.stamp = nh.now();
		CDLaser_msg.header.frame_id = scan_frame_id; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		CDLaser_msg.angle_min = f_angle_min;
		CDLaser_msg.angle_max = f_angle_max;
		CDLaser_msg.angle_increment = f_angle_increment;
		CDLaser_msg.time_increment = f_time_increment;
		CDLaser_msg.scan_time = f_scan_time;
		CDLaser_msg.range_min = f_range_min;
		CDLaser_msg.range_max = f_range_max;

		for (int z = 0; z < 5; z++)
		{
			//CDLaser_msg.ranges[z] = f_ranges[z];
#ifdef SIMULATION_ON
			CDLaser_msg.ranges[z] = z;

#else
			CDLaser_msg.ranges[z] = robot.getLDSDistance();

#endif // SIMULATION_ON

		}

		//for (int z = 0; z<5; z++)
		//{
		//  CDLaser_msg.intensities[z] = f_intensities[z];//If your device does not provide intensities, please leave the array empty.
		//}

		pub_Laser.publish(&CDLaser_msg);

		//           nh.spinOnce();
		ROS_INFO("P. laser");
		laserscan_time = millis() + 5000;

}

}

#pragma endregion

#pragma region ROS IMU publisher
//header files for imu
#include <ros_lib\ros_arduino_msgs/RawImu.h>
#include <ros_lib/geometry_msgs/Vector3.h>

ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

void publishIMU()
{
	if (raw_imu_msg.accelerometer && raw_imu_msg.gyroscope && raw_imu_msg.magnetometer)
	{
		//this function publishes raw IMU reading
		raw_imu_msg.header.stamp = nh.now();
		raw_imu_msg.header.frame_id = "imu_link";
		readMPUCompass(&raw_imu_msg.raw_linear_acceleration.x, &raw_imu_msg.raw_linear_acceleration.y, &raw_imu_msg.raw_angular_velocity.z, &raw_imu_msg.raw_magnetic_field)
		////measure accelerometer
		//if (raw_imu_msg.accelerometer)
		//{
		//	measureAcceleration();
		//	raw_imu_msg.raw_linear_acceleration = raw_acceleration;
		//}

		////measure gyroscope
		//if (raw_imu_msg.gyroscope)
		//{
		//	measureGyroscope();
		//	raw_imu_msg.raw_angular_velocity = raw_rotation;
		//}

		////measure magnetometer
		//if (raw_imu_msg.magnetometer)
		//{
		//	measureMagnetometer();
		//	raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
		//}
		
		//publish raw_imu_msg object to ROS
		raw_imu_pub.publish(&raw_imu_msg);
	}
}

#pragma endregion

#pragma endregion



void setup()
{
	setupRobot();

#pragma region ROS initialization
	nh.initNode();
	#if TF		
		broadcaster.init(nh);
	#endif

	ROS_INFO("ROBOT SETUP COMPLETE");

	setup_odom();



	setup_chatter();

	setup_speech();

	setup_ultrasound();

	setup_laserscan();


	setup_cmd_vel();

	nh.spinOnce();


#pragma endregion

}


void loop()
{
#if 0
	publish_tf();

#endif // 0
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
