/*
   rosserial PubSub Example
   Prints "hello world!" and toggles led
*/
#define SIMULATION_OFF  #define SIMULATION_ON

#ifdef SIMULATION_ON
void setupRobot() {
	pinMode(13, OUTPUT);
	pinMode(8, OUTPUT);   digitalWrite(8, LOW);



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

#endif
#ifdef SIMULATION_OFF
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
	setupStepperLDS(0); //avvia lo stepper
#endif

#if OPT_SEROVOSONAR
	robot.initRobot(&servoSonar);
#endif


	robot.initRobot(); //inizializza solo le variabili

					   //	noTone(Pin_LED_TOP_R);

}

#endif // SIMULATION_OFF





#include "math.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>    // ultrasound
#include <geometry_msgs/Twist.h>  // cmd_vel



ros::NodeHandle  nh;

#define ROS_INFO(s) nh.loginfo(s);

//--------------------------------

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//--------------------------------

//--------------------------------
sensor_msgs::Range rosmsg_range;
ros::Publisher pub_range("ultrasound", &rosmsg_range);

//--------------------------------
geometry_msgs::Twist msg;

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//  twist                               ---------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------


//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//  ultrasound                          ---------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------



  #include <sensor_msgs/LaserScan.h>  // LDS
  // laser scan global data-------------------------------------------------------
sensor_msgs::LaserScan CDLaser_msg;
long lds_publisher_timer;
ros:: Publisher pub_Laser("scan", &CDLaser_msg);
	#define LDSmicrostepsPerStep 2

	#define LDSsamples 5
	#define LDSspeed PI/LDSmicrostepsPerStep        // PI rad/sec = 180�/sec 
	float f_angle_min = - PI/2 ;//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	float f_angle_max = PI/2 ;
	float f_angle_increment= PI/LDSsamples;  // (f_angle_max - f_angle_min)/ LDSsamples 
	float f_time_increment;
	float f_scan_time= 2 * LDSmicrostepsPerStep; // LDSmicrostepsPerStep* 2* (f_angle_max-f_angle_min )/LDSspeed
	float f_range_min =0.02;
	float f_range_max =2.0;
	float f_ranges[LDSsamples]; // max of 30 measurements
	float f_intensities[LDSsamples];
//--------------------------------



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
		//  pub_laserscan->ranges[index] = atof(word.c_str()) / 100.0;
		//  //printf("%s\n", word.c_str());
		//  index++;
		//}
		//


		*/

		if (millis() > lds_publisher_timer)
		{
			#ifndef SIMULATION_ON
				// imposta la velocità
				myLDSstepper.goRadsPerSecond(2 * PI);
				// attende che arrivi in home
				while (!myLDSstepper.isHomePosition()) { delay(200); }
			#endif

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
			//  CDLaser_msg.intensities[z] = f_intensities[z];//If your device does not provide intensities, please leave the array empty.
			//}

			lds_publisher_timer = millis() + 5000;
			pub_Laser.publish(&CDLaser_msg);
			nh.spinOnce();

		}

	}
/*
void publish_scan() {
  if (millis() > lds_publisher_timer)
  {
	CDLaser_msg.header.stamp = nh.now();
	CDLaser_msg.header.frame_id = "laser_frame";
	CDLaser_msg.angle_min = f_angle_min;
	CDLaser_msg.angle_max = f_angle_max;
	CDLaser_msg.angle_increment = f_angle_increment;
	CDLaser_msg.time_increment = f_time_increment;
	CDLaser_msg.scan_time = f_scan_time;
	CDLaser_msg.range_min = f_range_min;
	CDLaser_msg.range_max = f_range_max;

	for (int z = 0 ; z < 5; z++)
	{
	  CDLaser_msg.ranges[z] = f_ranges[z];
	}

	for (int z = 0 ; z < 5; z++)
	{
	  CDLaser_msg.intensities[z] = f_intensities[z];
	}

	lds_publisher_timer = millis() + 3000;
	pub_Laser.publish(&CDLaser_msg);
	nh.spinOnce();
  }
}
*/



//-----------------------------------------------------------------------
//-----------------------------------------------------------------------














	double x = 1.0;
	double y = 0.0;
	double theta = 1.57;
	double g_req_angular_vel_z = 0;
	double g_req_linear_vel_x = 0;
	unsigned long g_prev_command_time = 0;

float move1;
float move2;

const int adc_pin = 0;



char base_link[] = "/base_link";
char odom[] = "/odom";
char frameid[] = "/ultrasound";

//temporarily holds data from vals
//char charVal[10];
char hello[11] = "ciao bello";
char ros_info[30] ;
 


unsigned long publisher_timer;
unsigned long range_time;
unsigned long tf_time;
//-----------------------------------------------------------


//-----------------------------------------------------------

 
 

 



//-----------------------------------------------------------

float getRange_Ultrasound(int pin_num) {
  int val = 0;
  for (int i = 0; i < 4; i++) val += analogRead(pin_num);
  float range =  val;
  return range / 322.519685;  // (0.0124023437 /4) ; //cvt to meters
}

//-----------------------------------------------------------------
void messageCb( const std_msgs::Empty& toggle_msg) {
  digitalWrite(13, HIGH - digitalRead(13)); // blink the led
}

ros::Subscriber<std_msgs::Empty> subMsg("toggle_led", messageCb );


//-----------------------------------------------------------------
void cmdvelCallBack(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x ;
  move2 = cmd_vel.angular.z ;

  x += cos(theta) * cmd_vel.linear.x * 0.1;
  y += sin(cmd_vel.angular.z) * cmd_vel.linear.x * 0.1;
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel",  cmdvelCallBack);

//-----------------------------------------------------------------

 
 
void commandCallback(const geometry_msgs::Twist& cmd_vel)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_vel.linear.x;
  g_req_angular_vel_z = cmd_vel.angular.z;

  g_prev_command_time = millis();

  theta += cmd_vel.angular.z;
  if (theta > 3.14)    theta = -3.14;
  x += cos(theta) * cmd_vel.linear.x * 0.1;
  y += sin(theta) * cmd_vel.linear.x * 0.1;


  ROS_INFO("cmd_vel");
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

//-----------------------------------------------------------------

std_msgs::String str_msg;
ros::Publisher chatter("/chatter", &str_msg);
ros::Publisher speech("/rp/state_externalization/vocal_message", &str_msg);

long i = 0;
 
long speech_time;


#pragma region Setup_ROS


void setupUltrasound() {
	rosmsg_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
	rosmsg_range.header.frame_id = frameid;
	rosmsg_range.field_of_view = 0.1;  // fake
	rosmsg_range.min_range = 0.0;
	rosmsg_range.max_range = 6.47;

}


#pragma endregion

// ---------------------------------------------
// Pubblica   ----------------------------------
// ---------------------------------------------

// Pubblica  chatter ----------------------------------
void publish_chatter(char* charVal) {
	//str_msg.data = strcat("ultra sound:", charVal);
	str_msg.data = charVal;
	chatter.publish(&str_msg);
	nh.spinOnce();
}


// Pubblica  tf ----------------------------------
void publish_tf(){
  /*
	// drive in a circle---------
	double dx = 0.02;
	double dtheta = 0.18;
	x += cos(theta)*dx*0.1;
	y += sin(theta)*dx*0.1;
	theta += dtheta*0.1;
	if(theta > 3.14)
	  theta=-3.14;
	//---------------------------
  */

  // broadcast tf odom->base_link-------------
  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.translation.x = x;
  t.transform.translation.y = y;

  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
  nh.spinOnce();
  //------------------------------------------


}

// Pubblica  odometry  ----------------------------------
void publish_odom(){
	
}

// Pubblica  speech  ----------------------------------
void publish_speech(){
  if ( millis() >= speech_time ) {
	// pubblicazione su chatter e speech-------
	str_msg.data = hello;
	chatter.publish(&str_msg );
	speech.publish(&str_msg );
	ROS_INFO( "speech");

	speech_time =  millis() + 5000;
	nh.spinOnce();
  }

  //  str_msg.data = itoa(i,str_msg,5);
  //  speech.publish(&str_msg );

  
}

// Pubblica  ultrasound  ----------------------------------
void publish_ultrasound() {
  //publish ULTRASOUND
  if ( millis() >= range_time ) {
	int r = 0;

	rosmsg_range.range = getRange_Ultrasound(5);
	rosmsg_range.header.stamp = nh.now();
	pub_range.publish(&rosmsg_range);
	range_time =  millis() + 100;
	nh.spinOnce();
  }

}



void setup()
{
	setupRobot();

	#pragma region ROS initialization
		nh.initNode();
		nh.subscribe(subMsg);
		nh.subscribe(subCmdVel);
		nh.subscribe(cmd_sub);
		
		nh.spinOnce();
		broadcaster.init(nh);

		//nh.advertise(chatter);
		//nh.advertise(speech);
		//setupUltrasound();
		//nh.advertise(pub_range);
		//setupLaserScan();
		//nh.advertise(pub_Laser);
		
  
	#pragma endregion
		ROS_INFO( "ROBOT SETUP COMPLETE");
	
}




void loop()
{
	publish_tf();

	publish_speech();


	publish_ultrasound();

	publish_laserscan(); // acquisisce le distanze...
	nh.spinOnce();

	digitalWrite(13, 1);
	delay(500);
	digitalWrite(13, 0);
}
