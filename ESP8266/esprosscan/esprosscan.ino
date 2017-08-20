// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
#define SIMULATION_ON 0
#define DEBUG_ON 1

#pragma region NODEMCU_HW

	#pragma region NODEMCU_PIN

		//LED
		#define  Pin_LED_TOP_G D0
		#define LEDTOP_G_ON digitalWrite(Pin_LED_TOP_G, 1);
		#define LEDTOP_G_OFF digitalWrite(Pin_LED_TOP_G, 0);

		//LASER
		#define Pin_LaserOn D8	// Accensione Laser
		#define LASER_ON digitalWrite(Pin_LaserOn,1);
		#define LASER_OFF digitalWrite(Pin_LaserOn,0);



		#define writeFast1(gpIO)	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 4,  (1<<gpIO));
		#define writeFast0(gpIO) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 8,  (1<<gpIO));


//#include <TickerScheduler\TickerScheduler.h>
//		#define TASK_STEPPER_CK 1
//		TickerScheduler sched(5);
//
//		void setup_scheduler() {
//			boolean add(TASK_STEPPER_CK, 5, &stepper_ck , true);
//		}
//	#pragma endregion

	#pragma region Helper Functions
		//void MSG2F(char * c, float f) {
		//	#define CHARFLOATSIZE 7
		//	#define CHARFLOATDECS 4
		//	char charVal[CHARFLOATSIZE];  //temporarily holds data from vals
		//								  //4 is mininum width, 3 is precision; float value is copied onto buff
		//	dtostrf(f, CHARFLOATSIZE, CHARFLOATDECS, charVal);
		//	Serial.print("1,");
		//	Serial.print(c);
		//	Serial.print(charVal);
		//	Serial.println(F(";"));
		//}
		#if DEBUG_ON
			#define dbg(s) 	Serial.println(s); 
			#define dbg2(s,v)  	Serial.print(s);Serial.println(v);  


		#else
			#define dbg(s) 
			#define dbg2(s,v)
		#endif

		void ledSeq1(int ms, int times =3) {
			pinMode(Pin_LED_TOP_G, OUTPUT);
			for (size_t i = 0; i < times; i++)
			{
			digitalWrite(Pin_LED_TOP_G, 0); delay(ms);
			digitalWrite(Pin_LED_TOP_G, 1); delay(ms);

			}
 
		}
	#pragma endregion

	#pragma region STEPPER_LDS
 
		#define ESP_PIN_STEPPERLDS_CK D7	/*LED_BUILTIN   =D16 */
		#define ESP_PIN_STEPPERLDS_HOME	D1 
		#define ESP_PIN_STEPPERLDS_END	D2 
		#define ESP_PIN_STEPPERLDS_CW	D3
		#define ESP_PIN_STEPPERLDS_ENABLE	D4
		#define ESP_PIN_SPI_SDA	D5
		#define ESP_PIN_SPI_CK	D6

		#define STEPPERLDS_CW digitalWrite(ESP_PIN_STEPPERLDS_CW,0);
		#define STEPPERLDS_CCW digitalWrite(ESP_PIN_STEPPERLDS_CW,1);
		#define STEPPERLDS_INVERTDIR digitalWrite(ESP_PIN_STEPPERLDS_CW,!digitalRead(ESP_PIN_STEPPERLDS_CW));

		#define MINIMUM_INTERRUPT_INTERVAL_MSEC 50
		//bool isHomePosition;
		//bool StepperEnd;
		//bool SepperIsMovingCW= true;

		#define Rads2Steps  31.8309886f		/* 200 step per giro / 2PI */
		//#define	_microStepDivider 2
		#define Steps2Rads PI/100.0


		#pragma region timer clock motore
			extern "C" {
				#include "user_interface.h"
			}

			os_timer_t myTimer;
			volatile byte ckStatus;
			// start of timerCallback
			void timerCallback(void *pArg) {
				// Toggle led's state
				ckStatus ^= 1;
				digitalWrite(ESP_PIN_STEPPERLDS_CK, ckStatus);

			} // End of timerCallback

		#pragma endregion

		#pragma region CLASSE STEPPER_C

			class myStepper_c
			{
				public:
					myStepper_c(int pinCK, int pinEn, int pinCw, int pinHome, int pinStop, int microstepDivider  );

					void goRadsPerSecond(float speed);
					void setHomePosition(bool flag);
					void setEndPosition(bool flag);
					void enable();
					void disable();
					void setCW(bool cw);
					bool isHomePosition();
					bool isEndPosition();
					bool isMovingCW();
					uint32_t radPerSec2Hz(float speed);

					int getStepDivider();
					#define MS1 1
					#define MS2 1
					// MS2  MS1				stepDivider
					//	0	0	< full step		/1
					//	0	1	< half step		/2 current
					//	1	0	< quarter step	/4
					//	1	1	< eighten		/8

				private:
					int _microStepDivider = 2;
					int _pinEn;
					int _pinCk;
					int _pinCw;
					int _pinHome;
					int _pinStop;
					long _stepDelay; // microseconds
					long _nextStep;
					volatile bool _isMovingCW = true;
					volatile bool _blHome; // gestito da interrupt
					volatile bool _blEnd; // gestito da interrupt
			};
  
			myStepper_c::myStepper_c(int pinEn, int pinCK, int pinCw, int pinHome, int pinStop, int microstepDivider  ) {
				_pinEn = pinEn;
				_pinCk = pinCK;
				_pinCw = pinCw;
				_pinHome = pinHome;
				_pinStop = pinStop;
				_microStepDivider = microstepDivider;
				pinMode(_pinEn, OUTPUT);	digitalWrite(_pinEn, 0);	//
				pinMode(_pinCk, OUTPUT);	digitalWrite(_pinCk, 0);	//
				pinMode(_pinCw, OUTPUT);	digitalWrite(_pinCw, 0);	//
				pinMode(_pinHome, INPUT_PULLUP);
				pinMode(_pinStop, INPUT_PULLUP);




				//6.28 rad / sec it will make one full revolution in 1 second, ie 200 steps.
				//2 * pi = 200 step / sec
				//1 rad/sec = 200 / 2 * PI step / sec

				//	resetTimer5();  
				//	goRadsPerSecond(0.0);

				//if ((MS1+MS2)==0)
				//{
				//	_divider = 1;
				//}
				//else
				//{
				//	_divider = 2 ^ ((MS2 << 1) + MS1);
				//	Serial.print("\n_divider:");	Serial.println(_divider);

				//}
				//	Serial.print("\n_divider:");	Serial.println(_stepDivider);

			}
			void myStepper_c::goRadsPerSecond(float speed)
			{
				//dbg2("speed demand: ",speed);

				if (speed == 0.0)
				{
					disable();
					//resetTimer5();
				}
				else
				{
					//DIRECTION
					if (speed > 0.0)
					{
						setCW(true);
					}
					else
					{
						setCW(false);
					}

					// 2Pi rads/sec = 200 step / sec
					// speed / 2PI = Hz/ 200
					// Hz = 200 * (speed /2PI)
					unsigned long Hz = (unsigned long)(abs(speed)* Rads2Steps*_microStepDivider);
					_stepDelay = (unsigned long)(1000000UL / Hz) / 2; // divido per 2 in quanto l'interupt che pilota lo stepper alza il segnale ogni 2 interrupt

 
					//startTimer5(_stepDelay);		//1000000L  = 1Hz

					enable();
				}
			}
			void myStepper_c::enable() {
				digitalWrite(_pinEn, 1);
			}
			void myStepper_c::disable() {
				digitalWrite(_pinEn, 0);
				//	noNewTone(_pinCK);
			}
			bool myStepper_c::isHomePosition() {
				//return digitalReadFast(_pinStop);
				//(condition) ? true - clause : false - clause
				//	_blHome = digitalRead(_pinStop);
				if (_blHome)
				{
					_blHome = false;
					return true;
				}
				else
				{
					return  false;

				}
			}
			bool myStepper_c::isEndPosition() {
				if (_blEnd)
				{
					_blEnd = false;
					return true;
				}
				else
				{
					return  false;

				}
			}

			void myStepper_c::setCW(bool cw = 1) {
				//(condition) ? (if_true) : (if_false)
				if (cw)
				{
					_isMovingCW = true;
					digitalWrite(_pinCw, 1);
				}
				else
				{
					
					_isMovingCW = false;
					digitalWrite(_pinCw, 0);
				}
				//	noNewTone(_pinCK);
			}
			void myStepper_c::setHomePosition(bool flag) {
				_blHome = flag;
			}
			void myStepper_c::setEndPosition(bool flag) {
				_blEnd = flag;
			}
			int myStepper_c::getStepDivider() {
				return _microStepDivider;
			}
			bool myStepper_c::isMovingCW() {
				return _isMovingCW;
			}
			uint32_t myStepper_c::radPerSec2Hz(float speed) {
				uint32_t Hz = (unsigned long)(abs(speed)* Rads2Steps*_microStepDivider);
				return Hz;
			}
		#pragma endregion



		myStepper_c myLDSstepper(ESP_PIN_STEPPERLDS_CK, ESP_PIN_STEPPERLDS_ENABLE, ESP_PIN_STEPPERLDS_CW, ESP_PIN_STEPPERLDS_HOME, ESP_PIN_STEPPERLDS_END,2);
 

		//// Esegue lo step 
		void stepperTimer_init(uint32_t msec) {
			/*
			os_timer_setfn - Define a function to be called when the timer fires

			void os_timer_setfn(
			os_timer_t *pTimer,
			os_timer_func_t *pFunction,
			void *pArg)

			Define the callback function that will be called when the timer reaches zero. The pTimer parameters is a pointer to the timer control structure.

			The pFunction parameters is a pointer to the callback function.

			The pArg parameter is a value that will be passed into the called back function. The callback function should have the signature:
			void (*functionName)(void *pArg)

			The pArg parameter is the value registered with the callback function.
			*/
			os_timer_setfn(&myTimer, timerCallback, NULL);

			/*
			os_timer_arm -  Enable a millisecond granularity timer.

			void os_timer_arm(
			os_timer_t *pTimer,
			uint32_t milliseconds,
			bool repeat)

			Arm a timer such that is starts ticking and fires when the clock reaches zero.

			The pTimer parameter is a pointed to a timer control structure.
			The milliseconds parameter is the duration of the timer measured in milliseconds. The repeat parameter is whether or not the timer will restart once it has reached zero.

			*/
			os_timer_arm(&myTimer, msec, true); // 10 = 100 step al secondo = mezzo giro/s = PI rad/s
		} // End of user_init

		static unsigned long last_interrupt_timeHome = 0;
		static unsigned long last_interrupt_timeEnd = 0;

		// ISR che Commuta la direzione dello stepper

		void ISRstepperSwitchHome() {
			LEDTOP_G_ON
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				myLDSstepper.setHomePosition(true);
				myLDSstepper.setCW(false);  ///angle positive ccw
											//	myLDSstepper.goRadsPerSecond(2 * LDS_STEPPER_SPEED);
			}
			last_interrupt_timeHome = interrupt_time;
		}
		// ISR che Commuta la direzione dello stepper
		void ISRstepperSwitchEnd() {
			LEDTOP_G_OFF
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{

				myLDSstepper.setEndPosition(true);
				myLDSstepper.setCW(true);  ///.disable();
										   //	myLDSstepper.goRadsPerSecond(LDS_STEPPER_SPEED);
			}
			last_interrupt_timeEnd = interrupt_time;
		}


		void setup_stepperLDS(float speed = 2 * PI) {

			pinMode(ESP_PIN_STEPPERLDS_HOME, INPUT_PULLUP);
			pinMode(ESP_PIN_STEPPERLDS_END, INPUT_PULLUP);
			pinMode(ESP_PIN_STEPPERLDS_CK, OUTPUT);
			pinMode(ESP_PIN_STEPPERLDS_CW, OUTPUT);
			pinMode(ESP_PIN_STEPPERLDS_ENABLE, OUTPUT);

			attachInterrupt(ESP_PIN_STEPPERLDS_HOME, ISRstepperSwitchHome, FALLING);  // add more attachInterrupt code as required
			attachInterrupt(ESP_PIN_STEPPERLDS_END, ISRstepperSwitchEnd, FALLING);  // add more attachInterrupt code as required
																						 // start stepper
   




			// Imposta la frequenza del clock motori
			#if SIMULATION_ON
				analogWriteFreq(10);// pre effetto microstepping la reale freq. è divisa per due
				analogWrite(LED_BUILTIN, 50); //duty cycle 50%
				stepperTimer_init(50); // per rendere visibile il ck
			#else
				//stepperTimer_init(10);
				//analogWriteFreq(radPerSec2Hz(PI));
				analogWriteFreq(400);// pre effetto microstepping la reale freq. è divisa per due
				analogWrite(ESP_PIN_STEPPERLDS_CK, 100); //duty cycle 50%
			#endif // SIMULATION_ON


		}

	#pragma endregion

	#pragma region LDS
		//#include <arduino.h>
		#include "Wire.h"
		//#include <ESP8266_libraries\ESP8266_VL53L0X\VL53L0X.h>
		#include <VL53L0X.h>

		VL53L0X LDS;
		#define LONG_RANGE
		// Uncomment ONE of these two lines to get
		// - higher speed at the cost of lower accuracy OR
		// - higher accuracy at the cost of lower speed

		//#define HIGH_SPEED
		//#define HIGH_ACCURACY


		#if SIMULATION_ON

			float getLDSDistance() {
 
				#define	SIM_SCAN_RANGE_AVG 1.0f


				float d;
				// random genera long tra min e max-1
				d = SIM_SCAN_RANGE_AVG + (float)random(1,101) /100;
				dbg(d);
				return d;
			}
			bool setup_LDS() {
				randomSeed(analogRead(A0));
				return true;
			}

		#else
			float getLDSDistance() {
				return (float)(LDS.readRangeSingleMillimeters() / 1000);
			}

			bool setup_LDS() {
				Wire.begin(ESP_PIN_SPI_SDA, ESP_PIN_SPI_CK);
				//LDS.init();
				byte failCount = 0;
				bool initDone = false;
				while (!initDone && (failCount < 10))
				{
					if (LDS.init())
					{
						initDone = true;
						dbg("LDS GO")
						return initDone;
					}
					else
					{
						dbg("LDS FAIL")
						delay(500);
						failCount++;
					}

				}
				LDS.setTimeout(100);
				#if defined LONG_RANGE
					Serial.println("Long range");
					// lower the return signal rate limit (default is 0.25 MCPS)
					LDS.setSignalRateLimit(0.1);
					// increase laser pulse periods (defaults are 14 and 10 PCLKs)
					LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
					LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
				#endif

				#if defined HIGH_SPEED
					// reduce timing budget to 20 ms (default is about 33 ms)
					distanceSensor.setMeasurementTimingBudget(20000);
				#elif defined HIGH_ACCURACY
					Serial.println("High Accuracy");
					// increase timing budget to 200 ms
					distanceSensor.setMeasurementTimingBudget(200000);
				#endif

				return initDone;
			}
		#endif // SIMULATION_ON

	#pragma endregion


	void setup_HW() {
		Serial.begin(57600);
 
		// LASER
		pinMode(Pin_LaserOn, OUTPUT);
		// LED
		pinMode(Pin_LED_TOP_G, OUTPUT);
		setup_LDS();
		setup_stepperLDS(PI);
	}


#pragma endregion

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
		
 
		String mac = WiFi.macAddress();
		dbg2("\nConnecting to ", ssid);
		dbg2( "MAC ADDR:", mac);
 

		//  Serial.print("\nConnecting to "); Serial.println(ssid);
		uint8_t i = 0;
		while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
		if(i == 21){

			dbg2("Could not connect to: ",ssid);

			while (1) {
				// LED flash
				digitalWrite(LED_BUILTIN, 1);		delay(200);
				digitalWrite(LED_BUILTIN, 0); 		delay(200);
			}
		}
		else //ok connesso
		{
			dbg2("Ready to use  IP: ",WiFi.localIP());
 
			#if SIMULATION_ON
				// led acceso per 2 secondi
				digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
				digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
				digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
				digitalWrite(LED_BUILTIN, 1);
			#endif
			//debug only
			//Serial.println("WiFi Connected"); Serial.println(ssid);
			//Serial.print("Ready! Use ");
			//Serial.print(WiFi.localIP());
			//Serial.println(" to access client");
			//Serial.print(1); //ok connected

		}
	}

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
	//#include <sensor_msgs/LaserScan.h>
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
	#define ROS_INFO(s) nh.loginfo(s);

	// laser data---------------
	#define LDSmicrostepsPerStep 2

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






	#pragma region ROS_SCAN_VECCHIA_PROCEDURA

	/*		//--------------------------------
		sensor_msgs::LaserScan scan_msg;;
		ros::Publisher pub_Laser("scan", &scan_msg);

		#define LDSsamples 15
		#define LDSspeed PI		// PI rad/sec = 180°/sec 
		#define SCAN_ANGLE   PI  //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		#define SCAN_ANGLE_MIN  -SCAN_ANGLE/2	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		#define SCAN_ANGLE_MAX   SCAN_ANGLE/2

		#define SCAN_ANGLE_INCREMENT  SCAN_ANGLE / LDSsamples;  // (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN)/ LDSsamples 
		#define SCAN_TIME  2 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX-SCAN_ANGLE_MIN )/LDSspeed
		#define SCAN_TIME_INCREMENT_MS  SCAN_TIME/LDSsamples
		#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667
		#define	SCAN_RANGE_MIN 0.01f
		#define	SCAN_RANGE_MAX 5.0f


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
			scan_msg.header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			scan_msg.angle_min = -SCAN_ANGLE / 2;
			scan_msg.angle_max = SCAN_ANGLE / 2;
			scan_msg.angle_increment = (float)SCAN_ANGLE / LDSsamples;
			scan_msg.scan_time = 2;	// 3.91 tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE / LDSspeed ;
			scan_msg.time_increment = scan_msg.scan_time / (float)(LDSsamples - 1);	//(1 / laser_frequency) / (num_readings);
			scan_msg.range_min = SCAN_RANGE_MIN;
			scan_msg.range_max = SCAN_RANGE_MAX;

			MSG2F("t_inc s.", scan_msg.time_increment);

		}
		void publish_laserscan() {
 

 

 
				// attende che arrivi in home se non è in simulazione
				#if not SIMULATION_ON
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
*/
	#pragma endregion

	#pragma region ROS_SCAN
		#include <sensor_msgs/LaserScan.h>
		#define LDSmicrostepsPerStep 2

		//--------------------------------
		sensor_msgs::LaserScan scan_msg;;
		ros::Publisher pub_Laser("scan", &scan_msg);

		#define LDSsamples 30
		#define LDSspeed PI		// PI rad/sec = 180°/sec 
		#define SCAN_ANGLE PI		//SCAN_ANGLE_MAX - SCAN_ANGLE_MIN
		#define SCAN_ANGLE_MIN  -SCAN_ANGLE/2	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		#define SCAN_ANGLE_MAX   SCAN_ANGLE/2

		#define SCAN_ANGLE_INCREMENT  SCAN_ANGLE / LDSsamples;  // (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN)/ LDSsamples 
		#define SCAN_TIME  2 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX-SCAN_ANGLE_MIN )/LDSspeed
		#define SCAN_TIME_INCREMENT_MS  SCAN_TIME/LDSsamples
		#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667
		#define	SCAN_RANGE_MIN 0.01f
		#define	SCAN_RANGE_MAX 5.0f

		//	float ranges[LDSsamples]; // max of 30 measurements
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
			scan_msg.header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			scan_msg.angle_min = -SCAN_ANGLE / 2;
			scan_msg.angle_max = SCAN_ANGLE / 2;
			scan_msg.angle_increment = (float)SCAN_ANGLE / LDSsamples;
			scan_msg.scan_time = 2;	// 3.91 tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE / LDSspeed ;
			scan_msg.time_increment = scan_msg.scan_time / (float)(LDSsamples - 1);	//(1 / laser_frequency) / (num_readings);
			scan_msg.range_min = SCAN_RANGE_MIN;
			scan_msg.range_max = SCAN_RANGE_MAX;

			//MSG2F("t_inc s.", scan_msg.time_increment);
		}

		void publish_laserscan() {


			int i = 0;
			uint32_t dt;
			// uint32_t	t, acqTime;  //for test only


			dt = (uint32_t)(1000 * scan_msg.time_increment) - 40;
			//MSG2("dt ", dt);

			if (myLDSstepper.isMovingCW())
			{
				// attende che arrivi in home
				dbg("wait home...");
				while (!myLDSstepper.isHomePosition()) { 
					delay(10); ESP.wdtFeed(); 
				}


				scan_msg.header.stamp = nh.now();

				// acquisisce le distanze
				dbg("acquiring...");
				//ROS_INFO("acquiring...");
				i = 0;


				LASER_ON;
				while (i < LDSsamples)	//riduco   per arrivare in tempo a fare la scansione in senso inverso
				{
					//t = millis();

					dbg2("\ni:",i)

					//---------------------------------------------------------------
					//acquisisco (36 - 37 mSec richiesti)
					//---------------------------------------------------------------
						ESP.wdtDisable();

					ranges[i] = (float)(LDS.readRangeSingleMillimeters() / 1000);
					ESP.wdtEnable(WDTO_1S);
					//ranges[i] = 1.0f;		// for test

					//if (ranges[i] > 2)
					//{
					//	// uso il sonar
					//	ranges[i] = (float)sonar.ping_cm() / 100;
					//}
					//---------------------------------------------------------------
					delay(1);
					dbg2("d:", ranges[i]);


					// test only -----------------
					//acqTime = millis() - t;  //37 mSec
					//MSG2("acqTime:", acqTime);



					i++; //incremento il puntatore

					delay(dt); 	//attendo time_increment
					// prove empiriche----------------------------
					//con  scan_time = PI
					//  18 trovato sperimentalmente ok 
					//con scantime  = 3.91
					// 25 è poco (finisce dopo EndPosition)
					// se metto  MSG2("acqTime:", acqTime ) , 50  è ok


				}
				LASER_OFF;
				dbg("end scan");

 
				// pubblica
				//MSG("publish /scan");
				scan_msg.ranges = ranges;
				scan_msg.intensities = intensities;
				pub_Laser.publish(&scan_msg);
				dbg("end publishing");

				}

				//-------------------------------------------------------------
				// scansione in senso opposto, movimento CW 
				// ma solo se arrivo prima che abbia già invertito il moto
				//-------------------------------------------------------------
				/*
				if (!myLDSstepper.isMovingCW())
				{
				while (!myLDSstepper.isEndPosition()) { delay(10); }
				// scansione in senso CW

				i = LDSsamples - 1;

				LEDTOP_B_ON
				while (i >= 0)
				{
				//t = millis();

				//acquisisco (36 - 37 mSec richiesti)
				ranges[i] = (float)LDS.readRangeSingleMillimeters() / 1000;
				i--; //decremento il puntatore
				delay(dt);


				}

				LEDTOP_B_OFF


				//MSG("end acquiring...");
				//ROS_INFO("end acquiring...");

				//for (unsigned int i = 0; i < LDSsamples; ++i) {
				//	ranges[i] = (float)LDS.readRangeSingleMillimeters()/1000;
				//	delay(SCAN_TIME_INCREMENT_MS);
				//}



				// pubblica
				MSG("publish /scan");
				scan_msg.ranges = ranges;
				scan_msg.intensities = intensities;
				pub_Laser.publish(&scan_msg);


				}
				else
				{
				ROS_INFO("skipping back scan!");

				}

				*/


				//imposta la prossima scansione
				scan_time = millis() + 2000;
				//MSG("-- x --");

 
		}

	#pragma endregion





	//-----------------------------------------------------------------

	/// ///////////////////////////////////////////////////////////////////////////////
	void publish_chatter(char* charVal) {


		//str_msg.data = strcat("ultra sound:", charVal);
		str_msg.data = charVal;
		chatter.publish(&str_msg);
 
	}


 


 


	#pragma region Setup_ROS



		void setup_ROS() {
		
			nh.initNode();
 
			nh.advertise(chatter);
			nh.advertise(pub_Laser);

			//setupUltrasound();
			//nh.advertise(pub_range);

			setup_ROS_LaserScan();
			dbg("End setup_Ros");
		}



	#pragma endregion	// Setup_ROS

#pragma endregion ROS 

void setup() {

	setup_HW();
	LEDTOP_G_ON;
	setup_WiFi();  
	delay(2000);


	setup_ROS();

	ledSeq1(500);
	dbg(" ####### EspRosScan ##########");
	LEDTOP_G_OFF;
	ESP.wdtEnable(WDTO_1S);

 }

long ms = millis();
void loop() {
 
 
		publish_laserscan(); // acquisisce le distanze...

		if (millis() > ms+1000)
		{
			ROS_INFO("scan alive");

			nh.spinOnce();  //elabora eventuali callBack

		}

		delay(10); // per il WiFi
 }
