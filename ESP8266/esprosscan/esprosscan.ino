#include <VL53L0X.h>
#include <Wire.h>

// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
#define SIMULATION_STEPPER_ON 0
#define SIMULATION_LDS_ON 0

#define DEBUG_ON 1
#define SERIAL_SPEED 74880		//default at boot=	74880
//#include "Arduino.h"


#pragma region LDS System
	enum scannerStatus_e {
		STARTING = 0,
		WIFICONNECTED,
		ROSCONNECTED

	};
	#define SCAN_SPEED_SEARCHING  0.3
	#define SCAN_SPEED_SCANNING  1.5	// 10
	#define SCAN_SPEED_RETURNING 4.5	// 8 velocità di ritorno
	// PROVE
	// SCAN_SPEED_SCANNING	SCAN_SPEED_RETURNING
	//		5						10					FUNZIONA REGOLARMENTE MA TERMINA LE MISURE PRIMA DI STEPER_END 
	scannerStatus_e LDSStatus = scannerStatus_e::STARTING;
	void startAcquiringDistances(int samples, float time_interval_sec);

#pragma endregion



#pragma region NODEMCU_HW

	#pragma region NODEMCU_PIN

		//LED
		#define Pin_LaserOn					D0	// Accensione Laser
		#define Pin_LED_TOP_B				LED_BUILTIN
		// I2C
		#define ESP_PIN_SPI_SDA				D1
		#define ESP_PIN_SPI_CK				D2

		// SWITCH
		#define ESP_PIN_STEPPERLDS_HOME		D3		//	D9 funziona ma vedo caratteri spuri su seriale 
		#define ESP_PIN_STEPPERLDS_END		D5		// Spostato da D4 a D5 perchè 	D4 coincide con il led interno all'ESP8266

		// LED EXT.
		#define ESP_PIN_LED2				D9

		// STEPPER
		#define ESP_PIN_STEPPERLDS_CW		D6
		#define ESP_PIN_STEPPERLDS_CK		D7	/*LED_BUILTIN   =D16 */
		#define ESP_PIN_STEPPERLDS_ENABLE	D8





		#define LED2_ON digitalWrite(ESP_PIN_LED2, 1);
		#define LED2_OFF digitalWrite(ESP_PIN_LED2, 0);



		#define LED_ON digitalWrite(Pin_LED_TOP_B, 0);
		#define LED_OFF digitalWrite(Pin_LED_TOP_B, 1);

		//LASER



		#define writeFast1(gpIO)	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 4,  (1<<gpIO));
		#define writeFast0(gpIO) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 8,  (1<<gpIO));

		//#define LASER_ON digitalWrite(Pin_LaserOn,1);
		//#define LASER_OFF digitalWrite(Pin_LaserOn,0);

		#define LASER_ON writeFast1(Pin_LaserOn);
		#define LASER_OFF writeFast0(Pin_LaserOn);

#pragma region esp_tft

		////################ TFT su SPI CONNECTION #######
		//#if 0
		//#include <Adafruit_GFX.h>
		//#include <gfxfont.h>
		//#include "Adafruit_ILI9341.h"


		//#define _CS   D4 // D4 goes to TFT CS
		//#define _DC   D8 // D8 goes to TFT DC
		//#define _mosi D7 // D7 goes to TFT MOSI
		//#define _sclk D5 // D5 goes to TFT SCK/CLK
		//#define _rst     // ESP RST to TFT RESET
		//#define _miso    // Not connected
		////       3.3V    // Goes to TFT LED  
		////       5v      // Goes to TFT Vcc
		////       Gnd     // Goes to TFT Gnd        

		//// Use hardware SPI (on ESP D4 and D8 as above)
		//Adafruit_ILI9341 tft = Adafruit_ILI9341(_CS, _DC);

		//bool    Centred = true, notCentred = false;

		//void clear_screen() {
		//	tft.fillScreen(ILI9341_BLACK);
		//}
		//void display_progress(String title, int percent) {
		//	int title_pos = (320 - title.length() * 12) / 2; // Centre title
		//	int x_pos = 35; int y_pos = 105;
		//	int bar_width = 250; int bar_height = 15;
		//	tft.fillRect(x_pos - 30, y_pos - 20, 320, 16, ILI9341_BLACK); // Clear titles
		//	display_text(title_pos, y_pos - 20, title, ILI9341_GREEN, 2, notCentred);
		//	tft.drawRoundRect(x_pos, y_pos, bar_width + 2, bar_height, 5, ILI9341_YELLOW); // Draw progress bar outline
		//	tft.fillRoundRect(x_pos + 2, y_pos + 1, percent*bar_width / 100 - 2, bar_height - 3, 4, ILI9341_BLUE); // Draw progress
		//	delay(1000);
		//}

		//void display_text(int x, int y, String message, int txt_colour, int txt_size, bool centred) {
		//	int txt_scale = 6; // Defaults to size = 1 so Font size is 5x8 !
		//	if (txt_size == 2) txt_scale = 11; // Font size is 10x16
		//	if (txt_size == 3) txt_scale = 17;
		//	if (txt_size == 4) txt_scale = 23;
		//	if (txt_size == 5) txt_scale = 27;
		//	if (centred) {
		//		x = x - message.length()*txt_scale / 2;
		//	}
		//	tft.setCursor(x, y);
		//	tft.setTextColor(txt_colour);
		//	tft.setTextSize(txt_size);
		//	tft.print(message);
		//	tft.setTextSize(2); // Back to default text size
		//}

		//#endif // 0

#pragma endregion


		//################ VARIABLES #################
		

	//#include <TickerScheduler\TickerScheduler.h>
	//		#define TASK_STEPPER_CK 1
	//		TickerScheduler sched(5);
	//
	//		void setup_scheduler() {
	//			boolean add(TASK_STEPPER_CK, 5, &stepper_ck , true);
	//		}
	//	#pragma endregion

	#pragma region Helper Functions


		#if DEBUG_ON
			#define dbg(s) 	Serial.println(s); 
			#define dbg2(s,v)  	Serial.print(s);Serial.println(v);  
			#define dbgV(__VA_ARGS__) Serial.printf(__VA_ARGS__)

		#else
			#define dbg(s) 
			#define dbg2(s,v)
		#endif

		void ledSeq1(int ms, int times =3) {
			pinMode(Pin_LED_TOP_B, OUTPUT);
			for (int i = 0; i < times; i++)
			{
			digitalWrite(Pin_LED_TOP_B, 0); delay(ms);
			digitalWrite(Pin_LED_TOP_B, 1); delay(ms);

			}
 
		}


		#pragma region Hearthbeat

				//const unsigned char _heartbeat_values[] = { 21,21,21,21,21,21,21,21,21,21,21,21,21,22,23,25,
				//28,34,42,54,71,92,117,145,175,203,228,246,255,254,242,220,191,157,121,87,
				//58,34,17,6,1,0,2,5,9,13,16,18,19,20,21,21,21,21,21,21,21,21,21,21,21,21,21,21 };
				const unsigned char _heartbeat_values[] = { 1,2,3,5,7,9,1,1,1,21,21,21,21,22,23,25,
					28,34,42,54,71,92,117,145,175,203,228,246,255,254,242,220,191,157,121,87,
					58,34,17,6,1,0,2,5,9,13,16,18,19,20,21,21,21,21,21,21,21,21,9,7,5,3,2,1 };
				const unsigned char _HEARTBEAT_INDEXES = 64;

				//byte ledPin = 9; //Must be PWM capable pin
				//int heartbeat_period = 500; //[ms] 60000/120 (120 beats x min, baby) 
				int heartbeat_period = 1200; //[ms] 60000/70 (70 beats x min, adult in rest) 
											 //int heartbeat_period = 1000; // slower, feels like soothing


				void heartbeat(byte pwmLedPin) {
					int index = (millis() % heartbeat_period) * _HEARTBEAT_INDEXES / heartbeat_period;
					analogWrite(pwmLedPin, _heartbeat_values[index]);
					delay(20);
				}

		#pragma endregion

	#pragma endregion

	#pragma region STEPPER_LDS
		#define MS1 1   // Se il pin è libero = 1 
		#define MS2 1	// Se il pin è libero = 1 

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
			#include <math.h>       /* pow */
			class myStepper_c
			{
				public:
					myStepper_c(int pinCK, int pinEn, int pinCw, int pinHome, int pinStop, byte ms1, byte ms2 );

					void goRadsPerSecond(float speed);
					void stop();
					void setHomePosition(bool flag);
					void setEndPosition(bool flag);
					void setCW(bool cw);
					bool isHomePosition();
					bool isEndPosition();
					bool isMovingCW();
					uint32_t radPerSec2Hz(float speed);

					int getStepDivider();
					int setStepDivider(byte m1, byte m2);

				private:
					void enable();
					void disable();
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
  
			//myStepper_c::myStepper_c(int pinEn, int pinCK, int pinCw, int pinHome, int pinStop, int microstepDivider  ) {
			myStepper_c::myStepper_c(int pinEn, int pinCK, int pinCw, int pinHome, int pinStop, byte ms1, byte ms2 ) {
				_pinEn = pinEn;
				_pinCk = pinCK;
				_pinCw = pinCw;
				_pinHome = pinHome;
				_pinStop = pinStop;
				_microStepDivider = setStepDivider(ms1, ms2);
				pinMode(_pinEn, OUTPUT);	digitalWrite(_pinEn, 0);	//
				pinMode(_pinCk, OUTPUT);	digitalWrite(_pinCk, 0);	//
				pinMode(_pinCw, OUTPUT);	digitalWrite(_pinCw, 0);	//
				pinMode(_pinHome, INPUT_PULLUP);
				pinMode(_pinStop, INPUT_PULLUP);

				disable();


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

					analogWriteFreq(Hz);// per effetto microstepping la reale freq. è divisa per due
					//analogWrite(ESP_PIN_STEPPERLDS_CK, 100); //duty cycle 50%

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
			int myStepper_c::setStepDivider(byte ms1, byte ms2) {
				//#define MS1 1   // Se il pin è libero = 1 
				//#define MS2 1		 // Se il pin è libero = 1 
				// MS2  MS1				stepDivider
				//	0	0	< full step		/1
				//	0	1	< half step		/2 current
				//	1	0	< quarter step	/4
				//	1	1	< eighten		/8
				byte MS12 = ((ms2 << 1) | ms1);
				//dbg2("MS12", MS12);
				_microStepDivider =(int)pow( 2 , MS12 ); 
				return _microStepDivider;
			}
			bool myStepper_c::isMovingCW() {
				return _isMovingCW;
			}
			uint32_t myStepper_c::radPerSec2Hz(float speed) {
				uint32_t Hz = (unsigned long)(abs(speed)* Rads2Steps*_microStepDivider);
				return Hz;
			}

			void myStepper_c::stop() {

			}
		#pragma endregion



		myStepper_c myLDSstepper(ESP_PIN_STEPPERLDS_CK, ESP_PIN_STEPPERLDS_ENABLE,
			ESP_PIN_STEPPERLDS_CW, ESP_PIN_STEPPERLDS_HOME, ESP_PIN_STEPPERLDS_END,MS1,MS2);
 

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
		// e che fa iniziare la scansione
		void ISRstepperSwitchHome() {
			//LED_ON
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				
				if (LDSStatus == scannerStatus_e::ROSCONNECTED)
				{
					myLDSstepper.goRadsPerSecond(-SCAN_SPEED_SCANNING);

				}
				else
				{
					myLDSstepper.goRadsPerSecond(-SCAN_SPEED_SEARCHING);
				}
				myLDSstepper.setHomePosition(true);
				//myLDSstepper.setCW(false);  ///angle positive ccw
											//	myLDSstepper.goRadsPerSecond(2 * LDS_STEPPER_SPEED);
			}
			last_interrupt_timeHome = interrupt_time;
		}
		// ISR che Commuta la direzione dello stepper
		// e fa iniziare l'invio
		void ISRstepperSwitchEnd() {
			LED_OFF
			unsigned long interrupt_time = millis();
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				if (LDSStatus == scannerStatus_e::ROSCONNECTED)
				{
					myLDSstepper.goRadsPerSecond(SCAN_SPEED_RETURNING);
				}
				else
				{
					myLDSstepper.goRadsPerSecond(SCAN_SPEED_SEARCHING);
				}

				myLDSstepper.setEndPosition(true);
				//myLDSstepper.setCW(true);  ///.disable();
										   //	myLDSstepper.goRadsPerSecond(LDS_STEPPER_SPEED);
			}
			last_interrupt_timeEnd = interrupt_time;
		}

		//Livello ALTO per disabilitare
	#define STEPPER_LDS_ENABLE digitalWrite(ESP_PIN_STEPPERLDS_ENABLE,0);
	#define STEPPER_LDS_DISABLE digitalWrite(ESP_PIN_STEPPERLDS_ENABLE,1); 
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
			#if SIMULATION_STEPPER_ON
				analogWriteFreq(10);// per effetto microstepping la reale freq. � divisa per due
				analogWrite(LED_BUILTIN, 50); //duty cycle 50%
				stepperTimer_init(50); // per rendere visibile il ck
			#else
				//stepperTimer_init(10);
				//analogWriteFreq(radPerSec2Hz(PI));
				analogWriteFreq(400);// per effetto microstepping la reale freq. � divisa per due
				analogWrite(ESP_PIN_STEPPERLDS_CK, 100); //duty cycle 50%
			#endif // SIMULATION_STEPPER_ON


		}

	#pragma endregion

	#pragma region LDS
		//#include <arduino.h>
		//#include <Wire\Wire.h>		
		//#include <Wire.h>
		//#include <ESP8266_libraries\ESP8266_VL53L0X\VL53L0X.h>
		//#include <espVL53L0X.h>
		#include <VL53L0X.h>
		VL53L0X LDS;
		#define LONG_RANGE
		// Uncomment ONE of these two lines to get
		// - higher speed at the cost of lower accuracy OR
		// - higher accuracy at the cost of lower speed

		#define HIGH_SPEED
		//#define HIGH_ACCURACY

		void testLDS(int n = 20) {
		
			for (size_t i = 0; i < n; i++)
			{
				ESP.wdtFeed();
				dbg2("mm=", getLDSDistance_mm());
			}
		}
		#if SIMULATION_LDS_ON

			#define	SIM_SCAN_RANGE_AVG 1.0f
			#define	SIM_SCAN_RANGE_AVG_MM 1000
			float getLDSDistance() {
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
			uint16_t getLDSDistance_mm() {
				uint16_t d;
				// random genera long tra min e max-1
				d = SIM_SCAN_RANGE_AVG_MM + random(1, 10);
				dbg(d);
				return d;
			}

		#else

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
						dbg("\n ### LDS GO ###\n");
						 
						 
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
					dbg("Long range");
					// lower the return signal rate limit (default is 0.25 MCPS)
					LDS.setSignalRateLimit(0.1);
					// increase laser pulse periods (defaults are 14 and 10 PCLKs)
					LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
					LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
				#endif

				#if defined HIGH_SPEED
					// reduce timing budget to 20 ms (default is about 33 ms)
					LDS.setMeasurementTimingBudget(20000);
				#elif defined HIGH_ACCURACY
					dbg("High Accuracy");
					// increase timing budget to 200 ms
					LDS.setMeasurementTimingBudget(200000);
				#endif
				//LDS.startContinuous(2);


				return initDone;
			}

			uint16_t  getLDSDistance_mm() {
				//return LDS.readRangeSingleMillimeters();
				return LDS.readRangeContinuousMillimeters();
			}
			float getLDSDistance() {
				//return (float)(LDS.readRangeSingleMillimeters() / 1000);
				return (float)(LDS.readRangeContinuousMillimeters() / 1000);
			}


		#endif // SIMULATION_LDS_ON
		// acquisisce le distanze nel vettore ranges[i]

	#pragma endregion
/*
At startup, pins are configured as INPUT.
GPIO0-GPIO15 can be INPUT, OUTPUT, or INPUT_PULLUP.
GPIO16 can be INPUT, OUTPUT, or INPUT_PULLDOWN_16. 
It is also XPD for deepSleep() (perhaps via a small capacitor.)
Note that GPIO6-GPIO11 are typically used to interface with the flash memory 
ICs on most esp8266 modules, so these pins should not generally be used.
*/

	void printESPinfo() {
		dbg2("CPU freq.",ESP.getCpuFreqMHz());
		dbg2("FlashChipRealSize: ", ESP.getFlashChipRealSize());

	}
	void setup_HW() {

		
			
		//// TFT
		//tft.begin();
		//tft.setRotation(3);
		//tft.setTextSize(2);
		//clear_screen();
		//display_progress("Initialising", 5);

		// LED
		pinMode(Pin_LED_TOP_B, OUTPUT);
		pinMode(ESP_PIN_LED2, OUTPUT);

		// STEPPER
		dbg("\n setup stepper");
		setup_stepperLDS(PI);
		LED2_ON;
		delay(1000);
		LED2_OFF;
		myLDSstepper.stop();

		// LDS
		dbg("\n setup LDS");
		setup_LDS();

		// LASER
		//pinMode(Pin_LaserOn, OUTPUT);
		// start of timerCallback, repeat every "period"


		printESPinfo();

		/*
			int period=50;        // PID sample timer period in ms

			os_timer_t myTimer;

			void tic(void *pArg) {
				// tic counter
			}		
			os_timer_setfn(&myTimer, tic, NULL);
			os_timer_arm(&myTimer, period, true);   // timer in ms
		
		
		*/

	}


#pragma endregion



	void softReset() {
		Serial.println("Restarting...");
		//Serial.println(WiFi.status());
		//esp_wifi_wps_disable() // add this, okay
		ESP.restart();
	}
	//#include <Servo.h>


	// memoria libera ----
	extern "C" {
		#include "user_interface.h"
	}
	// usa così: uint32_t free = system_get_free_heap_size();
#pragma endregion


	#pragma region WIFI

	//#include <WiFiManager.h>

	//WiFiManager wifiManager; // Connect to Wi-Fi

		//////////////////////
		// WiFi Definitions //
		//////////////////////

		#include <ESP8266WiFi.h>
		const char* ssid = "FASTWEB-CSRLCU";
		const char* password = "cesarini";
		#define ROS_TCP_CONNECTION_PORT 11411
		#define ROS_MASTER_URI	"192.168.0.51"

		const uint16_t port = 80;
		const char * host = ROS_MASTER_URI; // ip or dns

		//IPAddress server(192, 168, 0, 51); // ip of your ROS server
		//IPAddress ip_address;
		int status = WL_IDLE_STATUS;



		// Use WiFiClient class to create TCP connections
		WiFiClient client;
	

		// classe usata da node handle di ROS
		class WiFiHardware {

		public:
			WiFiHardware() {};

			void init() {
				// do your initialization here. this probably includes TCP server/client setup

				//client.connect(host, ROS_TCP_CONNECTION_PORT); //client.connect(server, ROS_TCP_CONNECTION_PORT);
				//dbg2("WiFiClient connected to port ", ROS_TCP_CONNECTION_PORT);
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

			uint8_t i = 0;
			while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
			if(i == 21){

				dbg2("Could not connect to: ",ssid);
				unsigned long t0 = millis();
				while (1) {
					// LED flash per 3 sec...
					while( (millis() - t0) < 3000)
					{
						LED_ON; delay(10);LED_OFF;	delay(500);				 		


					}
					// poi faccio il reboot
					softReset();

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

				LDSStatus = scannerStatus_e::WIFICONNECTED;
				delay(2000);
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
//	#include <tf/transform_broadcaster.h>


	//	ros::NodeHandle  nh;
	ros::NodeHandle_<WiFiHardware> nh;

	//--------------------------------
//	geometry_msgs::TransformStamped t;
//	tf::TransformBroadcaster broadcaster;

	//--------------------------------
	std_msgs::String str_msg;
	ros::Publisher chatter("chatter", &str_msg);
	////--------------------------------
	//sensor_msgs::Range rosmsg_range;
	//ros::Publisher pub_range("ultrasound", &rosmsg_range);
 
	//--------------------------------
	#define ROS_INFO(s) nh.loginfo(s);

	// laser data---------------
	#define LDSmicrostepsPerStep 2

 
	char base_link[] = "base_link";
	char odom[] = "odom";
//	char frameid[] = "ultrasound";
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

	byte hbFreq = 10; //heartbeat


	#pragma region ROS_SCAN
		#include <sensor_msgs/LaserScan.h>
		#define LDSmicrostepsPerStep 2

		//--------------------------------
		sensor_msgs::LaserScan scan_msg;;
		ros::Publisher pub_Laser("/scan", &scan_msg);

		float ldsSpeed;
		int   LDSsamples;

		// Costanti indipendenti
		#define LDSsamples_default 45
		#define LDSsamples_max 90	// dimensione dell'array
		#define LDSspeed_default PI		// PI rad/sec = 180�/sec 
		#define SCAN_ANGLE 3*PI/2		// 270° , SCAN_ANGLE_MAX - SCAN_ANGLE_MIN

		#define	SCAN_RANGE_MIN 0.01f
		#define	SCAN_RANGE_MAX 5.0f


		// Costanti derivate

		#define SCAN_ANGLE_MIN  -SCAN_ANGLE/2	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		#define SCAN_ANGLE_MAX   SCAN_ANGLE/2

		#define SCAN_ANGLE_INCREMENT  SCAN_ANGLE / LDSsamples;  // (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN)/ LDSsamples 
		#define SCAN_TIME  2 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX-SCAN_ANGLE_MIN )/LDSspeed
		#define SCAN_TIME_INCREMENT_MS  SCAN_TIME/LDSsamples
		#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667

		//	float ranges[LDSsamples]; // max of 30 measurements
		float intensities[LDSsamples_max]; // buffer 
									   //------------------------
		float *ranges = new float[LDSsamples_max];
		unsigned long scan_time;


		void rosSetup_LaserScan( float scanAngle, float scanTime, float timeIncrement,   int LDSsamples ) {
			if (LDSsamples > LDSsamples_max)
			{
				LDSsamples = LDSsamples_max;
			};

			scan_msg.ranges_length = LDSsamples;
			scan_msg.intensities_length = LDSsamples;

			// create the test data
			for (int z = 0; z < LDSsamples; z++)
			{
				ranges[z] = 1.0;
				intensities[z] = 1.0;
				delay(1);
			}
			scan_msg.header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
			scan_msg.angle_min = -scanAngle / 2;
			scan_msg.angle_max = scanAngle / 2;
			scan_msg.range_min = SCAN_RANGE_MIN;
			scan_msg.range_max = SCAN_RANGE_MAX;


			scan_msg.scan_time = scanTime;	// 3.91 tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE / LDSspeed ;
			scan_msg.angle_increment = scanAngle / LDSsamples;
			scan_msg.time_increment = timeIncrement; // (float)scan_msg.scan_time / (float)(LDSsamples - 1);	//(1 / laser_frequency) / (num_readings);
			
 
			//char ros_info_msg[30];sprintf(ros_info_msg, "LDSsamples: %d", LDSsamples);	ROS_INFO(ros_info_msg);

		}


		void publish_laserscan() {
			// pubblica
			//ROS_INFO("publishing /scan");
			scan_msg.ranges = ranges;
			scan_msg.intensities = intensities;

			pub_Laser.publish(&scan_msg);
 
		}

	#pragma endregion





	#pragma region Setup_ROS

		// acquisisce i parametri da parameter server
		void readParameters(int LDSsamples, float  ldsSpeed) {
			if (nh.getParam("/ldsSpeed", &ldsSpeed, LDSspeed_default) )
			{
				dbg2("/ldsSpeed", ldsSpeed);
				myLDSstepper.goRadsPerSecond(ldsSpeed);
			}
			if ( nh.getParam("/LDSsamples", &LDSsamples, LDSsamples_default))
			{
				dbg2("/LDSsamples", LDSsamples);

			}
			

		}


		void setup_ROS() {
			dbg("Connecting to Ros...");



			client.connect(host, ROS_TCP_CONNECTION_PORT); //client.connect(server, ROS_TCP_CONNECTION_PORT);
			nh.initNode();
			dbg2("WiFiClient connected to port ", ROS_TCP_CONNECTION_PORT);

			// recupero parametri
			
 
			nh.advertise(chatter);

			nh.advertise(pub_Laser);
 
			nh.spinOnce();
			//delay(3000);




			//"no more advertise admitted...");
			while (!nh.connected()) { 
					nh.spinOnce();   
					delay(1000); LED_ON;  delay(5); LED_OFF; 
					nh.spinOnce();
					
								
			}
			//while (!nh.connected())
			//{
			//	nh.spinOnce();
			//	//dbg2(".", hbFreq);
			//	delay(50);
			//	analogWrite(Pin_LED_TOP_B, hbFreq++);
			//	analogWriteFreq(hbFreq++);// per effetto microstepping la reale freq. � divisa per due
			//	analogWrite(LED_BUILTIN, 50); //duty cycle 50%
			//}
			/*			*/




			LDSStatus = scannerStatus_e::ROSCONNECTED;
			dbg("CONNESSO A ROS");		
			nh.loginfo("CONNESSO A ROS");
			delay(5);
		




			
		}


	#pragma endregion	// Setup_ROS

#pragma endregion ROS 

	// inizia ad acquisire n[samples] con cadenza costante data da  [time_interval_sec]  (intervallo dato da dt)
	// debug su seriale delle distanze
	void startAcquiringDistances(int samples, float time_interval_sec ) {
		scan_msg.header.stamp = nh.now();
		uint32_t dt = (uint32_t)(1000 * time_interval_sec) ;  //time_interval_sec circa 0,045
		uint32_t nextLoop=0; // calcolo il prossimo loop

		int i = 0;
		uint16_t mm =1000;
		LDS.setTimeout(dt-1);
		//ESP.wdtDisable();// disabilito il Watchdog
		while ( (i < samples) && (!myLDSstepper.isMovingCW()))	//riduco   per arrivare in tempo a fare la scansione in senso inverso
		{

			//dbg2("\ni:", i)

			//---------------------------------------------------------------
			//acquisisco (richiesti 36 - 37 mSec )
			//---------------------------------------------------------------
			ESP.wdtFeed();
			nextLoop =millis()  + dt;// calcolo il prossimo loop

			mm = getLDSDistance_mm();//mm++;

			//dbg(mm);
			ESP.wdtFeed();
			ranges[i] = (float)mm / 1000;
 
			//if (ranges[i] > 2)
			//{
			//	// uso il sonar
			//	ranges[i] = (float)sonar.ping_cm() / 100;
			//}
			//---------------------------------------------------------------


			//delay(1);


			// test only -----------------
			//dbg2("d:", ranges[i]);
			//acqTime = millis() - t;  //37 mSec
			//MSG2("acqTime:", acqTime);



			i++; //incremento il puntatore
			int remainingTime = nextLoop - millis();
			//dbg2("ms: ", remainingTime);
			if (remainingTime > 0)
			{
				delay(remainingTime); 	//attendo time_increment
							// prove empiriche----------------------------
							//con  scan_time = PI
							//  18 trovato sperimentalmente ok 
							//con scantime  = 3.91
							// 25 � poco (finisce dopo EndPosition)
							// se metto  MSG2("acqTime:", acqTime ) , 50  � ok

			}


		}


	}


void setup() {
	Serial.begin(SERIAL_SPEED);
	Serial.setDebugOutput(false);// to enable output from printf() function.  >>http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#timing-and-delays

	#if SIMULATION_STEPPER_ON || SIMULATION_LDS_ON
		printf("\n######## ### Start EspRosScan SIMULATO #####\n\n\n\n\n");

	#else
		dbg("\n---------  Start EspRosScan  ---------\n")
	#endif // 0


	setup_WiFi();  	
	setup_HW();	


	dbg2("microstep divider = ", myLDSstepper.getStepDivider());

	setup_ROS(); //fa il setup dei vari nodi e legge i parametri


	#if 0 // loop infinito per verificare che non intervenga il wtd
		int i = 0;
		dbg("\nTest no crash");
		while (true)
		{
			while (nh.connected())
			{
				ESP.wdtFeed();
				dbg2("n=", i++);
				client.flush();
				ROS_INFO("scan alive");
				delay(1000);  // 500 -->dopo n-=120  LmacRxBlk:1 which mean that the WiFi input buffer of the SDK is overloaded
			}
			dbg("\n Lost ROS");
			setup_ROS(); //fa il setup dei vari nodi e legge i parametri

		}
	#endif // 0



	//ESP.wdtFeed();
	
	//setup_stepperLDS(PI);


	//	myLDSstepper.goRadsPerSecond(0);
	//	LED_ON;

	myLDSstepper.goRadsPerSecond(SCAN_SPEED_SEARCHING);

	//delay(2000);


	//	STEPPER_LDS_DISABLE


	ledSeq1(300,3);
 
	//	STEPPER_LDS_ENABLE
	//	myLDSstepper.goRadsPerSecond(ldsSpeed);

	dbg(" test LDS -----");
	LDS.startContinuous(0);
	
	for (int i = 0; i < 10; i++)		//while (false)
	{
		//	heartbeat(Pin_LED_TOP_B);
		ESP.wdtFeed();

		uint32 mm = LDS.readRangeContinuousMillimeters();
		dbg2("mm ", mm);
		//delay(300);
	}

	ESP.wdtFeed();

	ESP.wdtEnable(WDTO_1S);




	LDSsamples = 60; //con 40 funziona  devono essere minore di  LDSsamples_max
	if (LDSsamples > LDSsamples_max) { LDSsamples = LDSsamples_max; }

	float scanAngle = SCAN_ANGLE;  // 3*PI/2 =4.71  >> 270°
	// tempo di scansione  = angolo di scansione / velocità 
	float scanSpeed = SCAN_SPEED_SCANNING;
	//time_increment = tempo di scansione / samples
	float timeIncrement = (scanAngle / scanSpeed)/LDSsamples;

	// tempo tra due scansioni
	float scanTime = scanAngle / scanSpeed + scanAngle/SCAN_SPEED_RETURNING ;	//
	//float angleIncrement = scanAngle / LDSsamples;
	rosSetup_LaserScan(scanAngle, scanTime, timeIncrement,    LDSsamples);


	myLDSstepper.goRadsPerSecond(-1);


	dbg(" --- Start /scan loop ---");
	dbg2("samples  : ", scan_msg.ranges_length );
	dbg2("angle_inc: ", scan_msg.angle_increment);
	dbg2("time_incr: ", scan_msg.time_increment);
	dbg2("scan_time: ", scan_msg.scan_time);
	dbg(" ------------------------");

}
unsigned long nextMsg_time = 0;
long loopcnt = 0;
void loop(){
	dbg(".");
	while  (  nh.connected())
	{ 
		ESP.wdtFeed();

		// wait for HomePosition
		if (myLDSstepper.isHomePosition())
		{
			loopcnt++;
			//analogWrite(Pin_LED_TOP_B, hbFreq++);
			LED_ON;			//			LASER_ON
			long t1 = millis();
			startAcquiringDistances(LDSsamples,scan_msg.time_increment );
			LED_OFF;

 
			printf("Scan %d in %d ms\n", loopcnt, millis() - t1);



 
	
			delay(1); // per il WiFi
			//publish_laserscan(); // acquisisce le distanze...
			//dbg("end publishing");


			//char ros_info_msg[30]; 			sprintf(ros_info_msg, "scan loop: %d",  loopcnt);			ROS_INFO(ros_info_msg);


		}
		if (myLDSstepper.isEndPosition())
		{
			//uint32_t free = system_get_free_heap_size();
			//dbg2("free ",free);  //42792
			LED2_ON;
			publish_laserscan(); // pubblica la scansione
			//char ros_info_msg[10]; 			sprintf(ros_info_msg, "P: %d",  loopcnt);			ROS_INFO(ros_info_msg);
			LED2_OFF;
			dbg("Pub");
		}


		// Ogni 1 secondo segnala che è vivo
		if (millis() > nextMsg_time + 1000)
		{

			//readParameters(LDSsamples, );
			//dbg2("Loop: ", loopcnt);  //42792
			//nh.advertise(chatter); // genera errore lato ROS dopo la connessione
			//nh.advertise(pub_Laser);
			//ROS_INFO("scan alive");


			nh.spinOnce();  //elabora eventuali callBack
			nextMsg_time = millis();
		}



 
	}


	// fermo il motore
	myLDSstepper.goRadsPerSecond(0);
	dbg("lost ROS connection !!!");

	LDSStatus = scannerStatus_e::STARTING;
	setup_ROS(); //fa il setup dei vari nodi

	ledSeq1(200,5);

	delay(1000);

}

