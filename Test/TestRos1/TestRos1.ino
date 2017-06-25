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

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

sensor_msgs::Range rosmsg_range;
ros::Publisher pub_range("/ultrasound", &rosmsg_range);


char hello[13] = "hello world!";

char base_link[] = "/base_link";
char odom[] = "/odom";

long range_time;
long tf_time;

const int adc_pin = 0;

char frameid[] = "/ultrasound";

float getRange_Ultrasound(int pin_num) {
	int val = 0;
	for (int i = 0; i < 4; i++) val += analogRead(pin_num);
	float range = val;
	return range / 322.519685;   // (0.0124023437 /4) ; //cvt to meters
}
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


						// ////////////////////////////////////////////////////////////////////////////////////////////
						//  CmdMessenger object to the default Serial port
						// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger2/CmdMessenger2.h>
static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
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

#pragma endregion

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
#pragma endregion


void countDown(int seconds) {
	MSG3("CountDown in ", seconds, " sec...");
	for (size_t i = seconds; i > 0; i--)
	{
		MSG2("  -", i);
		delay(1000);
	}
}




void setup()
{

#pragma region ROS initialization
	nh.initNode();
	broadcaster.init(nh);


	nh.advertise(chatter);


	nh.advertise(pub_range);


	rosmsg_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
	rosmsg_range.header.frame_id = frameid;
	rosmsg_range.field_of_view = 0.1;  // fake
	rosmsg_range.min_range = 0.0;
	rosmsg_range.max_range = 6.47;
#pragma endregion

#pragma region Robot init
	LEDTOP_R_ON	// Indica inizio SETUP Phase


	robot.initHW(); //disabilita i motori
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	MSG("ROBOTCORE v0.2");

	WEBCAM_ON
		MSG3("Bat : ", robot.readBattChargeLevel(), "%");
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

	pinMode(8, OUTPUT);
	digitalWrite(8, LOW);
#pragma endregion

}

//temporarily holds data from vals
char charVal[10];



void loop()
{
	//publish the adc value every 50 milliseconds
	//since it takes that long for the sensor to stablize
	if (millis() >= range_time) {
		int r = 0;


		// ROS Pubblicazione messaggio -------------
		rosmsg_range.range = getRange_Ultrasound(5);
		rosmsg_range.header.stamp = nh.now();
		pub_range.publish(&rosmsg_range);


		range_time = millis() + 500;

	}
	digitalWrite(13, 1);

	//4 is mininum width, 3 is precision; float value is copied onto buff
	dtostrf(getRange_Ultrasound(5), 4, 3, charVal);
	//str_msg.data = strcat("ultra sound:", charVal);
	str_msg.data = charVal;
	chatter.publish(&str_msg);


	if (millis() >= tf_time) {
		t.header.frame_id = odom;
		t.child_frame_id = base_link;
		t.transform.translation.x = 1.0;
		t.transform.rotation.x = 0.0;
		t.transform.rotation.y = 0.0;
		t.transform.rotation.z = 0.0;
		t.transform.rotation.w = 1.0;
		t.header.stamp = nh.now();
		broadcaster.sendTransform(t);

		tf_time = millis() + 1000;

	}



	nh.spinOnce();
	digitalWrite(13, 0);
	delay(500);


}
