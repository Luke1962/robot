// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
#define SIMULATION_ON 1
#define DEBUG 1

#if SIMULATION_ON
	#define dbg(s) if(DEBUG){	Serial.println(s); }
	#define dbg2(s,v) if(DEBUG){	Serial.print(s);Serial.println(v); }

#else
	#define dbg(s) 
	#define dbg2(s,v)
#endif
//#include <Servo.h>
#pragma region WIFI

	//////////////////////
	// WiFi Definitions //
	//////////////////////

	#include <ESP8266WiFi.h>
	const char* ssid = "FASTWEB-CSRLCU";
	const char* password = "cesarini";
	#define ROS_TCP_CONNECTION_PORT 11411

	IPAddress server(192, 168, 0, 51); // ip of your ROS server
	//IPAddress ip_address;
	int status = WL_IDLE_STATUS;

	WiFiClient client;
	
	class WiFiHardware {

	public:
		WiFiHardware() {};

		void init() {
			// do your initialization here. this probably includes TCP server/client setup
			client.connect(server, ROS_TCP_CONNECTION_PORT);
			dbg("[client.connect]");
		}

		// read a byte from the serial port. -1 = failure
		int read() {
			// implement this method so that it reads a byte from the TCP connection and returns it
			//  you may return -1 is there is an error; for example if the TCP connection is not open
			return client.read();         //will return -1 when it will works
		}

		// write data to the connection to ROS
		void write(uint8_t* data, int length) {
			// implement this so that it takes the arguments and writes or prints them to the TCP connection
			for (int i = 0; i < length; i++)
				client.write(data[i]);
		}

		// returns milliseconds since start of program
		unsigned long time() {
			return millis(); // easy; did this one for you
		}
	};

	void setup_WiFi()
	{
		//WiFi.enableSTA(true);		//	WiFi.setmode(WiFi.STATION);
		WiFi.begin(ssid, password);
		
		if (DEBUG) {
			String mac = WiFi.macAddress();
			Serial.print("\nConnecting to ");
			Serial.println(ssid);
			Serial.print("MAC ADDR:"); Serial.println(mac);
		}

		//  Serial.print("\nConnecting to "); Serial.println(ssid);
		uint8_t i = 0;
		while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
		if(i == 21){
			if (DEBUG) {
				Serial.print("Could not connect to: ");
				Serial.println(ssid);
			}
			while (1) {
				// LED flash
				digitalWrite(LED_BUILTIN, 1);		delay(200);
				digitalWrite(LED_BUILTIN, 0); 		delay(200);
			}
		}
		else //ok connesso
		{
			if (DEBUG) {
				Serial.print("Ready to use ");
				Serial.println(WiFi.localIP());
			}

			// led acceso per 2 secondi
			digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
			digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
			digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
			digitalWrite(LED_BUILTIN, 1);

			//debug only
			//Serial.println("WiFi Connected"); Serial.println(ssid);
			//Serial.print("Ready! Use ");
			//Serial.print(WiFi.localIP());
			//Serial.println(" to access client");
			//Serial.print(1); //ok connected
		}
	}

#pragma endregion


#pragma region STEPPER_LDS
	#define ESP_PIN_STEPPERLDS_CK LED_BUILTIN		//LED =D16
	#define ESP_PIN_STEPPERLDS_HOME	D1 
	#define ESP_PIN_STEPPERLDS_END	D2 
	#define ESP_PIN_STEPPERLDS_CW	D3
	#define ESP_PIN_STEPPERLDS_ENABLE	D4

	#define STEPPERLDS_CW digitalWrite(ESP_PIN_STEPPERLDS_CW,0);
	#define STEPPERLDS_CCW digitalWrite(ESP_PIN_STEPPERLDS_CW,1);
	#define STEPPERLDS_INVERTDIR digitalWrite(ESP_PIN_STEPPERLDS_CW,!digitalRead(ESP_PIN_STEPPERLDS_CW));

	#define MINIMUM_INTERRUPT_INTERVAL_MSEC 50
	bool isHomePosition;
	bool StepperEnd;
	bool SepperIsMovingCW;




	// Commuta la direzione dello stepper
	void ISRstepperSwitchHome() {
		static unsigned long last_interrupt_timeHome = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{
			isHomePosition =true;
			STEPPERLDS_CW;   
			SepperIsMovingCW = true;
		}
		last_interrupt_timeHome = interrupt_time;
	}
	// Commuta la direzione dello stepper
	void ISRstepperSwitchEnd() {
		static unsigned long last_interrupt_timeEnd = 0;
		unsigned long interrupt_time = millis();
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
		{

			StepperEnd = true;
			STEPPERLDS_CCW;  
			SepperIsMovingCW = false;
		}
		last_interrupt_timeEnd = interrupt_time;
	}


	// versione pe Interrupt su Change
	#if 0
		void ISRldsHome() {
			static unsigned long last_interrupt_timeHome = 0;
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				int x = digitalRead(ESP_PIN_STEPPERLDS_HOME);
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
				int x = digitalRead(ESP_PIN_STEPPERLDS_END);
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

	#endif // 0


	//// Esegue lo step 
	//volatile byte ckState = 0;
	//void ISRstepperMakeStep() { 
	//	ckState = !ckState;
	//	digitalWrite(ESP_PIN_STEPPERLDS_CK, ckState);
	//}

	void setup_stepperLDS(float speed = 2 * PI) {
		pinMode(ESP_PIN_STEPPERLDS_HOME, INPUT_PULLUP);// open >+5 closed =gnd
		pinMode(ESP_PIN_STEPPERLDS_END, INPUT_PULLUP);// open >+5 closed =gnd
												  //attachPinChangeInterrupt(ESP_PIN_STEPPERLDS_CK_HOME, ISRstepperSwitchHome, CHANGE);  // add more attachInterrupt code as required
												  //attachPinChangeInterrupt(ESP_PIN_STEPPERLDS_CK_END, ISRstepperSwitchEnd, CHANGE);  // add more attachInterrupt code as required

		//attachPinChangeInterrupt(ESP_PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, FALLING);  // add more attachInterrupt code as required
		//attachPinChangeInterrupt(ESP_PIN_STEPPERLDS_END, ISRstepperSwitchEnd, FALLING);  // add more attachInterrupt code as required
		attachInterrupt(ESP_PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, FALLING);  // add more attachInterrupt code as required
		attachInterrupt(ESP_PIN_STEPPERLDS_END, ISRstepperSwitchEnd, FALLING);  // add more attachInterrupt code as required
																					 // start stepper
   


		pinMode(ESP_PIN_STEPPERLDS_HOME, INPUT_PULLUP);
		pinMode(ESP_PIN_STEPPERLDS_END, INPUT_PULLUP);
		pinMode(ESP_PIN_STEPPERLDS_CK, OUTPUT);
		pinMode(ESP_PIN_STEPPERLDS_CW, OUTPUT);


		// Imposta la frequenza del clock motori
		analogWriteFreq(3);
		analogWrite(ESP_PIN_STEPPERLDS_CK, 500);
	}

#pragma endregion





#pragma region LDS
	#include <arduino.h>
	#include "Wire.h"
	#include <VL53L0X.h>

	VL53L0X LDS;
	#define LONG_RANGE



	#if SIMULATION_ON

		float getLDSDistance() {
			#define	SIM_SCAN_RANGE_MIN 0.2f
			#define	SIM_SCAN_RANGE_MAX 2.0f


			float d;
			// random genera long tra min e max-1
			d = SIM_SCAN_RANGE_MIN + (float)random(1,101) *(SIM_SCAN_RANGE_MAX - SIM_SCAN_RANGE_MIN )/100;
			dbg(d);
			return d;
		}
		void setup_LDS() {
 			randomSeed(analogRead(A0));
 
 		}

	#else
		float getLDSDistance() {
			return (float)(LDS.readRangeSingleMillimeters() / 1000);
		}

		void setup_LDS() {
			Wire.begin(4, 5);
			LDS.init();
		}
	#endif // SIMULATION_ON

#pragma endregion


#pragma region ROS MACRO REGION

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
	//ros::NodeHandle  nh;
	ros::NodeHandle_<WiFiHardware> nh;

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
	geometry_msgs::Twist msg;


	//--------------------------------


	//--------------------------------
	#define ROS_INFO(s) nh.loginfo(s);

	// laser data---------------
	#define LDSmicrostepsPerStep 2

#pragma region ROS_SCAN
	//--------------------------------
	sensor_msgs::LaserScan scan_msg;;
	ros::Publisher pub_Laser("scan", &scan_msg);

	#define LDSsamples 15
	#define LDSspeed PI/LDSmicrostepsPerStep		// PI rad/sec = 180°/sec 
	#define SCAN_ANGLE_MIN  -PI / 2;//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	#define SCAN_ANGLE_MAX  PI / 2

	#define SCAN_ANGLE_INCREMENT  PI / LDSsamples;  // (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN)/ LDSsamples 
	#define	SCAN_TIME_INCREMENT (1 / 100) / (LDSsamples)
	#define SCAN_TIME  2*LDSmicrostepsPerStep // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX-SCAN_ANGLE_MIN )/LDSspeed
	#define	SCAN_RANGE_MIN 0.2f
	#define	SCAN_RANGE_MAX 2.0f


	float intensities[LDSsamples]; // buffer 
	//------------------------
	float *ranges = new float[LDSsamples];
	unsigned long scan_time;


	void setup_ROS_LaserScan() {
		nh.advertise(pub_Laser);

 
		scan_msg.ranges_length = LDSsamples;
		scan_msg.intensities_length = LDSsamples;

		// create the test data
		for (int z = 0; z < LDSsamples; z++)
		{
			ranges[z] = 1.0;
			intensities[z] = 1.0;
		}
	}
	void publish_laserscan() {
 

		if (millis() > scan_time)
		{
			// imposta la velocità
 

			scan_msg.header.frame_id = "ultrasound_link"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			scan_msg.angle_min = SCAN_ANGLE_MIN;
			scan_msg.angle_max = SCAN_ANGLE_MAX;
			scan_msg.angle_increment =  SCAN_ANGLE_INCREMENT;
			scan_msg.time_increment = SCAN_TIME_INCREMENT;	//(1 / laser_frequency) / (num_readings);
			scan_msg.scan_time = SCAN_TIME;
			scan_msg.range_min = SCAN_RANGE_MIN;
			scan_msg.range_max = SCAN_RANGE_MAX;


 
			#if not SIMULATION_ON
				// attende che arrivi in home
				dbg("wait home...");
				while (!isHomePosition) { delay(200); }

			#endif // SIMULATION_ON

			scan_msg.header.stamp = nh.now();

			// acquisisce le distanze
			dbg("acquiring...");
			for (unsigned int i = 0; i < LDSsamples; ++i) {
  
				ranges[i] = getLDSDistance();
 
			}
 

 			scan_msg.ranges = ranges;
			scan_msg.intensities = intensities;
			 
			// pubblica
			dbg("publish /scan");
			pub_Laser.publish(&scan_msg);

			//imposta la prossima scansione
			scan_time = millis() + 1000;
			dbg("-- x --");

		}

	}

#pragma endregion

 
	char hello[13] = "hello world!";

	char base_link[] = "/base_link";
	char odom[] = "/odom";
	char frameid[] = "/ultrasound";
	char charVal[10];

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
		//nh.spinOnce();

	}


 


 


#pragma region Setup_ROS



	void setup_ROS() {
		//client.connect(server, 11411);
		
		nh.initNode();
		//broadcaster.init(nh);

		nh.advertise(chatter);
		nh.advertise(pub_Laser);

		//setupUltrasound();
		//nh.advertise(pub_range);

		setup_ROS_LaserScan();
		if (DEBUG)
		{
			Serial.println("End setup_Ros");

		}
	}



#pragma endregion	// Setup_ROS

#pragma endregion ROS 




//--mia aggiunta ------------------------------
/*
#include <std_msgs/String.h>

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void publish_chatter(char* charVal) {


	//str_msg.data = strcat("ultra sound:", charVal);
	str_msg.data = charVal;
	chatter.publish(&str_msg);
	nh.spinOnce();

}
//-------------------------------------------------
 */


void ledSeq1(int ms) {
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, 0); delay(ms);
	digitalWrite(LED_BUILTIN, 1); delay(ms);
	digitalWrite(LED_BUILTIN, 0); delay(ms);
	digitalWrite(LED_BUILTIN, 1); delay(ms);
	digitalWrite(LED_BUILTIN, 0); delay(ms);
	digitalWrite(LED_BUILTIN, 1); delay(ms);

}



void setup() {
	Serial.begin(57600);
	setup_WiFi();  delay(2000);

	setup_ROS();
	ledSeq1(300);
	dbg("\n\#######EspRosScan test##########");
	setup_stepperLDS();
	//setup_LDS();

 }

char strVal[10];
int cnt = 0;



//uint8_t buf1[1024];
//uint8_t i1 = 0;
//
//uint8_t buf2[1024];
//uint8_t i2 = 0;

int ad0 = 3;
int f = 3;
long ms = millis();

void loop() {
 
		publish_chatter("ESP8266");
		publish_laserscan(); // acquisisce le distanze...

		if (millis() > ms+1000)
		{

			nh.spinOnce();  //elabora eventuali callBack

		}

 
 }
