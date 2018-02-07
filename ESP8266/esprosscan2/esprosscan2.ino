/////////////////////////////////////////////////////////////////////
////  VERSIONE CON 
//	WIFI MANAGER
// PARAMETRI CARICATI DA ROS
/////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
// PROBLEMI NOTI
/////////////////////////////////////////////////////////////////////
//// STABILE (ALMENO FINO A 619 LOOP)
/////////////////////////////////////////////////////////////////////
//#include <ArduinoHardware.h>
#include <ros.h>

#include <VL53L0X.h>
#include <Wire.h>
//#include "Arduino.h"
// FILE (debug) :  C:\Users\Luca\AppData\Local\Temp\VMBuilds\esprosscan2\esp8266_nodemcuv2\Release\



#pragma region Blynk
	#define OPT_BLINK 0

	#if OPT_BLINK
		#include <BlynkSimpleEsp8266.h>
		#define BLYNK_TOKEN "10cc6df79d9e4cabb0fc54f093ea7769"
		char auth[] = BLYNK_TOKEN;
		#define BLYNK_PRINT Serial
		// This is called when Smartphone App is opened
		BLYNK_APP_CONNECTED() {
			Serial.println("Blynk App Connected.");
		}
		// This is called when Smartphone App is closed
		BLYNK_APP_DISCONNECTED() {
			Serial.println("Blynk App Disconnected.");
		}



	#endif

#pragma endregion


// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
#define SIMULATION_STEPPER_ON 0
#define SIMULATION_LDS_ON 0
#define	WEBSERVER 0
#define DEBUG_ON 1
#define SERIAL_SPEED		 115200		//default at boot=	74880


#pragma region LDS_System
	enum LDSStatus_e {
		STARTING = 0,
		WIFICONNECTED,
		ROSCONNECTED,
		SCANNING,
		ENDSCAN,
		PUBLISHING,
		ENDPUBLISH,
		SLOWPUBLISH, // la pubblicazione termina dopo l'arrivo a HOME
		SLOWSCAN	//la scansione termina dopo l'arrivo a END

	};
	#define SCAN_SAMPLES_DEFAULT 80



	//!!  Valori  di velocità espressi senza segno !!!
	#define SCAN_SPEED_SEARCHING  0.3
	#define SCAN_SPEED_SCANNING  1.5f	// 1.5
	#define SCAN_SPEED_RETURNING 5.0f	// 8 velocità di ritorno

	float scanSpeed = SCAN_SPEED_SCANNING; //sovrascritta mediante lettura ros parameter
	float returnSpeed = SCAN_SPEED_RETURNING;
	// PROVE
	// SCAN_SPEED_SCANNING	SCAN_SPEED_RETURNING
	//		5						10					FUNZIONA REGOLARMENTE MA TERMINA LE MISURE PRIMA DI STEPPER_END 
	LDSStatus_e LDSStatus = LDSStatus_e::STARTING;
//	void startAcquiringDistances(sensor_msgs::LaserScan* scan_msg, int samples, float time_interval_sec);
	long lastHomeTime;
	long lastEndTime;
 #pragma endregion	//LDS System



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
	#pragma  endregion


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

		void ledSeq1(int ms, int times = 3) {
			pinMode(Pin_LED_TOP_B, OUTPUT);
			for (int i = 0; i < times; i++)
			{
				digitalWrite(Pin_LED_TOP_B, 0); delay(ms);
				digitalWrite(Pin_LED_TOP_B, 1); delay(ms);

			}

		}


		String ftoa(float number, uint8_t precision, uint8_t size) {
			// Based on mem,  16.07.2008
			// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num = 1207226548/6#6

			// prints val with number of decimal places determine by precision
			// precision is a number from 0 to 6 indicating the desired decimial places
			// example: printDouble(3.1415, 2); // prints 3.14 (two decimal places)

			// Added rounding, size and overflow #
			// ftoa(343.1453, 2, 10) -> "    343.15"
			// ftoa(343.1453, 4,  7) -> "#      "
			// avenue33, April 10th, 2010

			String s = "";

			// Negative 
			if (number < 0.0) {
				s = "-";
				number = -number;
			}

			double rounding = 0.5;
			for (uint8_t i = 0; i < precision; ++i)    rounding /= 10.0;

			number += rounding;
			s += String(uint16_t(number));  // prints the integer part

			if (precision > 0) {
				s += ".";                // prints the decimal point
				uint32_t frac;
				uint32_t mult = 1;
				uint8_t padding = precision - 1;
				while (precision--)     mult *= 10;

				frac = (number - uint16_t(number)) * mult;

				uint32_t frac1 = frac;
				while (frac1 /= 10)    padding--;
				while (padding--)      s += "0";

				s += String(frac, DEC);  // prints the fractional part
			}

			if (size > 0)                // checks size
				if (s.length() > size)        return("#");
				else while (s.length() < size) s = " " + s;

				return s;
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

		#pragma endregion //Hearthbeat


		void printESPinfo() {
			dbg2("CPU freq.", ESP.getCpuFreqMHz());
			dbg2("FlashChipRealSize: ", ESP.getFlashChipRealSize());

		}

	#pragma endregion	//Helper Functions

	#pragma region STEPPER_LDS
		#define MS1 1   // Se il pin è libero = 1 
		#define MS2 1	// Se il pin è libero = 1 

		#define STEPPERLDS_CW digitalWrite(ESP_PIN_STEPPERLDS_CW,0);
		#define STEPPERLDS_CCW digitalWrite(ESP_PIN_STEPPERLDS_CW,1);
		#define STEPPERLDS_INVERTDIR digitalWrite(ESP_PIN_STEPPERLDS_CW,!digitalRead(ESP_PIN_STEPPERLDS_CW));
		#define STEPPERLDS_GOHOME 	myLDSstepper.goRadsPerSecond(returnSpeed);

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
			myStepper_c(int pinCK, int pinEn, int pinCw, int pinHome, int pinStop, byte ms1, byte ms2);

			void goRadsPerSecond(float speed);
			void setSpeed(float speed);
			float getSpeed();

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
			float _speed;
			volatile bool _isMovingCW = true;
			volatile bool _blHome; // gestito da interrupt
			volatile bool _blEnd; // gestito da interrupt
		};

		//myStepper_c::myStepper_c(int pinEn, int pinCK, int pinCw, int pinHome, int pinStop, int microstepDivider  ) {
		myStepper_c::myStepper_c(int pinEn, int pinCK, int pinCw, int pinHome, int pinStop, byte ms1, byte ms2) {
			_pinEn = pinEn;
			_pinCk = pinCK;
			_pinCw = pinCw;
			_pinHome = pinHome;
			_pinStop = pinStop;
			_microStepDivider = setStepDivider(ms1, ms2);
			_speed =0;
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
		// speed > 0 -->ruota  CW (ritorno a home)

		// imposta la velocità , clone di goRadsPerSecond
		void myStepper_c::setSpeed(float speed) {
			_speed = speed;



		}
		float myStepper_c::getSpeed( ) {
			return _speed;
		}
		void myStepper_c::goRadsPerSecond(float speed)
		{
			//dbg2("speed demand: ",speed);
			_speed = speed;
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
				unsigned long Hz = (unsigned long)(abs(speed)* Rads2Steps*_microStepDivider);// per effetto microstepping la reale freq. è divisa per _microStepDivider
				//_stepDelay = (unsigned long)(1000000UL / Hz) / 2; // divido per 2 in quanto l'interupt che pilota lo stepper alza il segnale ogni 2 interrupt

				analogWriteFreq(Hz);
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
			_microStepDivider = (int)pow(2, MS12);
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
			ESP_PIN_STEPPERLDS_CW, ESP_PIN_STEPPERLDS_HOME, ESP_PIN_STEPPERLDS_END, MS1, MS2);


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





		  ///////////////////////////////////////////////////////////////////////////////////////
		  ///////////////////////////////////////////////////////////////////////////////////////
		  //
		  //	ISR
		  //
		  ///////////////////////////////////////////////////////////////////////////////////////
		  //////////////////////////////////////////////////////////////////////////////////////
		static unsigned long last_interrupt_timeHome = 0;
		static unsigned long last_interrupt_timeEnd = 0;

		// ISR che Commuta la direzione dello stepper
		// e che fa iniziare la scansione
		// ICACHE_RAM_ATTR è fondamentale per la stabilità del programma!!!
		void ICACHE_RAM_ATTR ISRstepperSwitchHome() {
			//LED_ON
			unsigned long interrupt_time = millis();
			lastHomeTime = interrupt_time;
			float speedCCW = 0.0;
			// If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeHome > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				myLDSstepper.setHomePosition(true);
				switch (LDSStatus)
				{
				case STARTING:
				case WIFICONNECTED:
					speedCCW = -SCAN_SPEED_SEARCHING;

					break;
				case ROSCONNECTED:	//situazione iniziale
				case ENDPUBLISH:	// situazione normale a regime
					//LDSStatus = SCANNING;

					speedCCW = -scanSpeed;
					break;

				case SCANNING:
				case SLOWPUBLISH:
					// ignoro 
					break;

				case PUBLISHING:
					// Se arrivo a home che sto pubblicando, torno indietro comunque ma salto un'asquisizione
					LDSStatus = LDSStatus_e::SLOWPUBLISH;
					speedCCW = -returnSpeed;


					break;


				default:
					speedCCW = -SCAN_SPEED_SEARCHING;
					break;
				}

				

				//myLDSstepper.setSpeed(speedCCW); 
				myLDSstepper.goRadsPerSecond(speedCCW);

			}
			last_interrupt_timeHome = interrupt_time;
		}
		// ISR che Commuta la direzione dello stepper
		// e fa iniziare l'invio
			void ICACHE_RAM_ATTR ISRstepperSwitchEnd() {

			unsigned long interrupt_time = millis();
			lastEndTime = interrupt_time; // per debug
										  // If interrupts come faster than 200ms, assume it's a bounce and ignore
			if (interrupt_time - last_interrupt_timeEnd > MINIMUM_INTERRUPT_INTERVAL_MSEC)
			{
				float speedCW = 0.0;
				switch (LDSStatus)
				{
				case STARTING:
				case WIFICONNECTED:
					speedCW = SCAN_SPEED_SEARCHING;
					break;



				case ROSCONNECTED:
					speedCW = returnSpeed;
					break;
				case SCANNING:
					// devo fermarmi
					LDSStatus = LDSStatus_e::SLOWSCAN;
					speedCW = returnSpeed;
					//myLDSstepper.stop();
					break;

				case ENDSCAN: //ok
					// torno indietro
					speedCW = returnSpeed;
					break;
					//				myLDSstepper.goRadsPerSecond(returnSpeed);
					//				break;

				case PUBLISHING:
				case SLOWPUBLISH:
				case ENDPUBLISH:
					speedCW = returnSpeed;
					break;
				default:
					break;

				}
				//myLDSstepper.setSpeed(speedCW); 
				myLDSstepper.goRadsPerSecond(speedCW);
				myLDSstepper.setEndPosition(true);


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

			for (int i = 0; i < n; i++)
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
				d = SIM_SCAN_RANGE_AVG + (float)random(1, 101) / 100;
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
			//Wire.begin(ESP_PIN_SPI_SDA, ESP_PIN_SPI_CK);
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



		int scanI2C()
		{
			byte error, address;
			int nDevices;

			Serial.println("I2C Scanning...");

			nDevices = 0;
			for (address = 1; address < 127; address++)
			{
				ESP.wdtFeed();
				Wire.beginTransmission(address);
				error = Wire.endTransmission();

				if (error == 0)
				{
					Serial.print("I2C device found at address 0x");
					if (address < 16)
						Serial.print("0");
					Serial.print(address, HEX);
					Serial.println("  !");

					nDevices++;
				}
				else if (error == 4)
				{
					Serial.print("Unknow error at address 0x");
					if (address < 16)
						Serial.print("0");
					Serial.println(address, HEX);
				}
			}
			if (nDevices == 0)
				Serial.println("\n########################\nNo I2C devices found\n########################");
			else
				Serial.println("done\n");

			return nDevices;
		}
	#pragma endregion	//LDS
	/*
	At startup, pins are configured as INPUT.
	GPIO0-GPIO15 can be INPUT, OUTPUT, or INPUT_PULLUP.
	GPIO16 can be INPUT, OUTPUT, or INPUT_PULLDOWN_16.
	It is also XPD for deepSleep() (perhaps via a small capacitor.)
	Note that GPIO6-GPIO11 are typically used to interface with the flash memory
	ICs on most esp8266 modules, so these pins should not generally be used.
	*/

	void setup_HW() {

		printESPinfo();


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
		dbg("\nSetup stepper-----------");
		setup_stepperLDS(PI);
		LED2_ON;
		delay(1000);
		LED2_OFF;
		myLDSstepper.stop();
		dbg2("microstep divider = ", myLDSstepper.getStepDivider());


		// LASER
		//pinMode(Pin_LaserOn, OUTPUT);
		// start of timerCallback, repeat every "period"



		/*
		int period=50;        // PID sample timer period in ms

		os_timer_t myTimer;

		void tic(void *pArg) {
		// tic counter
		}
		os_timer_setfn(&myTimer, tic, NULL);
		os_timer_arm(&myTimer, period, true);   // timer in ms


		*/
		Wire.begin(ESP_PIN_SPI_SDA, ESP_PIN_SPI_CK);


		int i2cDevices = scanI2C();
		if (i2cDevices > 0)
		{
			printf("trovate %d periferiche I2C", i2cDevices);

			// LDS
			dbg("\n setup LDS");
			setup_LDS();


			dbg(" test LDS -----");
			LDS.startContinuous(0);

		}
		else
		{
			printf("Impossibile continuare. Soft Reset...");
			//	ESP.restart();
		}


		for (int i = 0; i < 10; i++)		//while (false)
		{
			//	heartbeat(Pin_LED_TOP_B);
			ESP.wdtFeed();

			uint32 mm = LDS.readRangeContinuousMillimeters();
			dbg2("mm ", mm);
			//delay(300);
		}


	}


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
#pragma endregion	// NODEMCU_HW


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




// ritorna true se riesce a connettersi al wifi
bool setup_WiFi(int maxRetries = 5)
{











	bool connected = false;

 
	WiFi.begin(ssid, password);

	String mac = WiFi.macAddress();
	dbg2("\nConnecting to ", ssid);
	dbg2("MAC ADDR:", mac);

	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < maxRetries) delay(1000);



	if (WiFi.status() != WL_CONNECTED) {
		connected = false;
		dbg2("Could not connect to: ", ssid);


		// LED flash per 3 sec...
		unsigned long t0 = millis();
		while ((millis() - t0) < 3000)
		{
			LED_ON; delay(10); LED_OFF;	delay(500);
		}

		//while (1) {
		//	// LED flash per 3 sec...
		//	while( (millis() - t0) < 3000)
		//	{
		//		LED_ON; delay(10);LED_OFF;	delay(500);				 		


		//	}
		//	// poi faccio il reboot
		//	softReset();

		//}
	}
	else //ok connesso
	{
		connected = true;
		dbg2("Ready to use  IP: ", WiFi.localIP());

#if SIMULATION_ON
		// led acceso per 2 secondi
		digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
		digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
		digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
		digitalWrite(LED_BUILTIN, 1);
#endif

		LDSStatus = LDSStatus_e::WIFICONNECTED;
		delay(2000);
	}

	return connected;
}
#pragma endregion



///////////////////////////////////////////////////////////////////////
// codice da http://usemodj.com/2016/08/04/esp8266-wifi-smartconfig/
//#include <WiFiManager\WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <WiFiManager.h>
#include <DNSServer.h>

//void configModeCallback(WiFiManager *myWiFiManager) {
//	Serial.println("Entered config mode");
//	Serial.println(WiFi.softAPIP());
//	//if you used auto generated SSID, print it
//	Serial.println(myWiFiManager->getConfigPortalSSID());
//}
//


//bool setup_WiFiManager() {
//
//
//	//WiFiManager
//	//Local intialization. Once its business is done, there is no need to keep it around
//	WiFiManager wifiManager;
//	//reset settings - for testing
//	//wifiManager.resetSettings();
//
//	//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
//	wifiManager.setAPCallback(configModeCallback);
//
//	//fetches ssid and pass and tries to connect
//	//if it does not connect it starts an access point with the specified name
//	//here  "AutoConnectAP"
//	//and goes into a blocking loop awaiting configuration
//	if (!wifiManager.autoConnect()) {
//		Serial.println("failed to connect and hit timeout");
//		//reset and try again, or maybe put it to deep sleep
//		ESP.reset();
//		delay(1000);
//	}
//
//	//if you get here you have connected to the WiFi
//	Serial.println("connected...yeey :)");
//
//
//							//if you used auto generated SSID, print it
//	Serial.print("SSID is:");
//	Serial.println(wifiManager.getConfigPortalSSID());
//
//}

//////////////////////
///
// WEB SERVER		 
///
//////////////////////
#pragma region WEB SERVER
#if WEBSERVER
#include <ESP8266WebServer.h>  // ho aggiunto come path File di Origine: C:\Users\Luca\AppData\Local\arduino15\packages\esp8266\hardware\esp8266\2.2.0\libraries\ESP8266WebServer\src
ESP8266WebServer server(80);
#include "webpages.hpp"
#endif
#pragma endregion





// ///////////////////////////////////////////////////////////////////////////////
///
//		ROS
///
// ///////////////////////////////////////////////////////////////////////////////

#pragma region ROS MACRO REGION


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
#define SCAN_SAMPLES_MAX 90	// dimensione dell'array
#define LDSspeed_default PI		// PI rad/sec = 180�/sec 

// Angolo massimo= un po' meno di 270° ( vanno tolti 3 step da 1.8° = 5.4°)
// 270 - 5.4 = 264.6° =  264.6° = 4.618 rad
//  (rad=  d*2Pi/360 )
#define SCAN_ANGLE_MAX	4.618		// 270° , SCAN_ANGLE_MAX_MAX - SCAN_ANGLE_MAX_MIN

#define	SCAN_RANGE_MIN 0.01f
#define	SCAN_RANGE_MAX 5.0f


// Costanti derivate

#define SCAN_ANGLE_MAX_MIN  -SCAN_ANGLE_MAX/2	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
#define SCAN_ANGLE_MAX_MAX   SCAN_ANGLE_MAX/2

#define SCAN_ANGLE_MAX_INCREMENT  SCAN_ANGLE_MAX / LDSsamples;  // (SCAN_ANGLE_MAX_MAX - SCAN_ANGLE_MAX_MIN)/ LDSsamples 
#define SCAN_TIME  2 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX_MAX-SCAN_ANGLE_MAX_MIN )/LDSspeed
#define SCAN_TIME_INCREMENT_MS  SCAN_TIME/LDSsamples
#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667

//	float ranges[LDSsamples]; // max of 30 measurements
//float intensities[SCAN_SAMPLES_MAX]; // buffer 
									 //------------------------
float* intensities = new float[SCAN_SAMPLES_MAX];
float* ranges = new float[SCAN_SAMPLES_MAX];
unsigned long scan_time;


// inizializza il contenuto di scan_msg in base ai parametri 
void rosSetup_LaserScan(sensor_msgs::LaserScan* scan_msg, float scanAngle, float scanTime, float timeIncrement, int LDSsamples) {
	if (LDSsamples > SCAN_SAMPLES_MAX)
	{
		dbg2("***WARNING samples > SCAN_SAMPLES_MAX ", LDSsamples);
		LDSsamples = SCAN_SAMPLES_MAX;
	};

	scan_msg->ranges_length = LDSsamples;
	scan_msg->intensities_length = LDSsamples;

	// create the test data
	for (int z = 0; z < LDSsamples; z++)
	{
		ranges[z] = 1.0;
		intensities[z] = 1.0;
		delay(1);
	}
	scan_msg->ranges = ranges;
	scan_msg->intensities = intensities;


	scan_msg->header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	scan_msg->angle_min = -scanAngle / 2;
	scan_msg->angle_max = scanAngle / 2;
	scan_msg->range_min = SCAN_RANGE_MIN;
	scan_msg->range_max = SCAN_RANGE_MAX;

	// tempo di andata + ritorno
	scan_msg->scan_time = scanTime;	// 3.91 tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE_MAX / LDSspeed ;
	scan_msg->angle_increment = scanAngle / LDSsamples;
	scan_msg->time_increment = timeIncrement; // (float)scan_msg.scan_time / (float)(LDSsamples - 1);	//(1 / laser_frequency) / (num_readings);


											 //char ros_info_msg[30];sprintf(ros_info_msg, "LDSsamples: %d", LDSsamples);	ROS_INFO(ros_info_msg);

	dbg("\n --- Scan parameters ---");

	dbg2("topic  : ", scan_msg->header.frame_id);
	dbg2("samples  : ", scan_msg->ranges_length);
	dbg2("angle_inc: ", scan_msg->angle_increment);
	dbg2("time_incr: ", scan_msg->time_increment);
	dbg2("scan_time: ", scan_msg->scan_time);
	dbg(" ------------------------");

}


inline void publish_laserscan() {
	// pubblica
	//ROS_INFO("publishing /scan");
	//scan_msg.ranges = ranges;
	//	scan_msg.intensities = intensities;

	pub_Laser.publish(&scan_msg);

}

#pragma endregion





#pragma region Setup_ROS

//// acquisisce i parametri da parameter server
//void readParameters(int LDSsamples, float  ldsSpeed) {
//	if (nh.getParam("/ldsSpeed", &ldsSpeed, LDSspeed_default))
//	{
//		dbg2("/ldsSpeed", ldsSpeed);
//		myLDSstepper.goRadsPerSecond(ldsSpeed);
//	}
//	if (nh.getParam("/LDSsamples", &LDSsamples, SCAN_SAMPLES_DEFAULT))
//	{
//		dbg2("/LDSsamples", LDSsamples);
//
//	}
//
//
//}

/// ///////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////
///								
///	FUNZIONI PER CARICARE I PARAMETRI ROS DAI FILE *.YAML					
///								
/// ///////////////////////////////////////////////////////////////
/// ////////////////////////////////////////////////////////////////

/*
Lanciare Rosserial node così:
<node ns="robot"  name="serial_node_wifi" pkg="rosserial_python" type="serial_node.py" output="screen">
<param name="port" value="tcp" />
<param name="/scan_speed" value="1.6"/>
<param name="/scan_samples" value="33"/>
</node>
*/
String thisNs = "/robot";
String thisNode = "/serial_node_wifi";


int loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, int defaultValue) {
	String parPath = thisNs + thisNode + "/" + parName;
	int parValue = defaultValue;
	ESP.wdtFeed();
	if (nh->getParam(parPath.c_str(), &parValue))
	{
		printf("\n Loaded int   [%s]:\t %d", parName, parValue);
		char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded int param  [%s]:\t %d", parName, parValue); nh->loginfo(ros_info_msg);

		return parValue;
	}
	else
	{
		char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** int param. [%s] not found. Using default value %f ", parPath.c_str(), defaultValue); nh->logwarn(ros_info_msg);
		printf("***Warning*** Int param. [%s] not found. \n\t\t Using default value %d \n\n", parPath.c_str(), defaultValue);
		printf(parPath.c_str());
		return defaultValue;
	}
}
float loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, float defaultValue) {

	String parPath = thisNs + thisNode + "/" + parName;
	float parValue = defaultValue;
	ESP.wdtFeed();//necessario
	if (nh->getParam(parPath.c_str(), &parValue))
	{
		String parValueStr = ftoa(parValue, 4, 10);
		printf("Loaded float [%s]:\t %s", parName, parValueStr.c_str());  // prinf con %f non è supportato
		char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded float param [%s]:\t  %s", parName, parValueStr.c_str() ); nh->loginfo(ros_info_msg);
		return parValue;
	}
	else
	{
		char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** float param. [%s] not found. Using default value %s ", parPath.c_str(),ftoa( defaultValue,4,10).c_str()); nh->logwarn(ros_info_msg);
		printf(" ***Warning*** float param. [%s] not found. \n\t \tUsing default value %s \n\n", parPath.c_str(), ftoa(defaultValue,4,10).c_str());
		return defaultValue;
	}
}

/**/


/*
std::string loadRosParameterStr(ros::NodeHandle* nh, std::string  parName, std::string defaultValue) {

std::string parPath = thisNs + thisNode + "/" + parName;
std::string parValue = defaultValue;

if (nh->getParam(parName, parValue))
{
printf("\n Loaded [%s ]: %s", parName.c_str(), parValue.c_str());
return parValue;
}
else
{
printf("\n ***Warning***  [%s] not found. Using default value %s", parPath.c_str(), defaultValue.c_str());
return defaultValue;
}
}



void connectToServer() {
Serial.print("connecting to ");
Serial.println(host);
const int httpPort = 21;
if (!client.connect(host, httpPort)) {
Serial.println("connection failed");
return;
}
}
*/


void ROS_loadParameters() {

	//LDSsamples =SCAN_SAMPLES_DEFAULT; //con 40 funziona  devono essere minore di  SCAN_SAMPLES_MAX

	LDSsamples = loadRosParameter(&nh, "scan_samples", SCAN_SAMPLES_DEFAULT);
	scanSpeed = loadRosParameter(&nh, "scan_speed", SCAN_SPEED_SCANNING);
	returnSpeed = loadRosParameter(&nh, "return_speed", SCAN_SPEED_RETURNING);

	if (LDSsamples > SCAN_SAMPLES_MAX) { LDSsamples = SCAN_SAMPLES_MAX; }

	float scanAngle = SCAN_ANGLE_MAX;
	// tempo di scansione  = angolo di scansione / velocità 

	//time_increment = tempo di scansione / samples
	float timeIncrement = (scanAngle / scanSpeed) / LDSsamples;

	// tempo tra due scansioni
	float scanTime = scanAngle / scanSpeed + scanAngle / returnSpeed;	//
																		//float angleIncrement = scanAngle / LDSsamples;
	rosSetup_LaserScan(&scan_msg, scanAngle, scanTime, timeIncrement, LDSsamples);


}




/// ///////////////////////////////////////////////////////////////
/// ////////////////////////////////////////////////////////////////

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
		if (!client.connected())
		{
			client.connect(host, ROS_TCP_CONNECTION_PORT);

		}
		#if OPT_BLYNK
				Blynk.virtualWrite(2, 250 * digitalRead(ESP_PIN_STEPPERLDS_HOME));
				Blynk.virtualWrite(3, 250 * digitalRead(ESP_PIN_STEPPERLDS_END));
				Blynk.run();

		#endif // OPT_BLYNK

		
		nh.spinOnce();
		delay(1000); LED_ON;  delay(5); LED_OFF;

		#if  WEBSERVER
				server.handleClient();
		#endif
	}






	LDSStatus = LDSStatus_e::ROSCONNECTED;
	dbg("CONNESSO A ROS");
	nh.loginfo("CONNESSO A ROS");
	delay(5);

	ROS_loadParameters();




}


#pragma endregion	// Setup_ROS

#pragma endregion ROS 

// inizia ad acquisire n[samples] con cadenza costante data da  [time_interval_sec]  (intervallo dato da dt)
// debug su seriale delle distanze
void startAcquiringDistances(sensor_msgs::LaserScan* scan_msg ) {

 
	float time_increment_msec = (uint32_t)(1000 * scan_msg->time_increment);

	uint32_t nextLoop = 0; // calcolo il prossimo loop

	int i = 0;
	uint16_t mm = 1000;
	LDS.setTimeout(time_increment_msec -2); // imposta il timeout per la misura della distanza

	scan_msg->header.stamp = nh.now();
	//ESP.wdtDisable();// disabilito il Watchdog
	//while ((i < scan_msg->ranges_length) && (!myLDSstepper.isMovingCW()))	//riduco   per arrivare in tempo a fare la scansione in senso inverso
															//while (i < samples - 1)	//riduco   per arrivare in tempo a fare la scansione in senso inverso
	while (LDSStatus== LDSStatus_e::SCANNING)
	{

		ESP.wdtFeed();
		nextLoop = millis() + time_increment_msec;// calcolo il prossimo loop


		//---------------------------------------------------------------
		//acquisisco (richiesti 36 - 37 mSec )
		//---------------------------------------------------------------
		mm = getLDSDistance_mm();//mm++;
								 //ESP.wdtFeed();
								 //ranges[i] = (float)mm / 1000;
		scan_msg->ranges[i] = (float)mm / 1000;


		// attendo per il tempo rimanente nel caso LDS risponda prima del timeout
		int remainingTime = nextLoop - millis();
		if (remainingTime > 0)
		{
			delay(remainingTime);
		}

		i++; //incremento il puntatore

		// se ho raggiunto il numero di campioni cambio stato
		if (i>= scan_msg->ranges_length)
		{
			LDSStatus = LDSStatus_e::ENDSCAN;
		}



	
	} // while i < samples 



	if (LDSStatus == LDSStatus_e::SLOWSCAN)  // sta già tornando indietro
	{
		// non ho completato tutte le acquisizioni
		//devo correggere i parametri

		dbg2("Scan late ms:", millis() - lastEndTime);
		dbg2("Samples are ", i);
		dbg2("         of ", scan_msg->ranges_length);

		char ros_info_msg[20]; sprintf(ros_info_msg, "Late scan: %d of %d",i-1, scan_msg->ranges_length); nh.logwarn(ros_info_msg);


		//scan_msg->ranges_length = i - 1;
		//scan_msg->angle_max = scan_msg->angle_min +  scan_msg->angle_increment * scan_msg->ranges_length;
	}

	LDSStatus == LDSStatus_e::ENDSCAN;
}







// ///////////////////////////////////////////////////////////////////////////////
///
//		S E T U P
///
// ///////////////////////////////////////////////////////////////////////////////

void setup() {
	Serial.begin(SERIAL_SPEED);
	Serial.setDebugOutput(false);// to enable output from printf() function.  >>http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#timing-and-delays
	LDSStatus = LDSStatus_e::STARTING;

	#if SIMULATION_STEPPER_ON || SIMULATION_LDS_ON
		printf("\n######## ### Start EspRosScan SIMULATO #####\n\n\n\n\n");

	#else
		dbg("\n---------  Start EspRosScan  ---------\n");
	#endif // 0

	//Local intialization. Once its business is done, there is no need to keep it around
	WiFiManager wifiManager;
	//wifiManager.resetSettings();//reset settings - for testing
	//wifiManager.setAPCallback(configModeCallback);//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode

	//fetches ssid and pass and tries to connect
	//if it does not connect it starts an access point with the specified name and goes into a blocking loop awaiting configuration
	if (!wifiManager.autoConnect("ESP","cesarini")) {
		Serial.println("failed to connect and hit timeout");
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(1000);
	}

	//if you get here you have connected to the WiFi
	Serial.println("WiFi onnected...yeey :)");


	//if you used auto generated SSID, print it
	Serial.print("SSID is:");
	Serial.println(wifiManager.getConfigPortalSSID());
	Serial.print("----------------------------------------");

/*
*/


	#if OPT_BLINK		

	

		Blynk.begin(auth, ssid, password);

	#endif





//	bool connectedToWiFi = setup_WiFi(20);  	// se non si connette diventa webserver
//	bool connectedToWiFi = setup_WiFiManager();
 
	#if WEBSERVER
													//	setup_WebServer(connectedToWiFi);
		start_WebServer();
		#if 0	// test webserver
			while (true)
			{
				LED_ON;
				delay(10);
				LED_OFF;
				server.handleClient();
			}

		#endif // 1
	#endif

	LDSStatus = LDSStatus_e::WIFICONNECTED;



	setup_HW();

#if OPT_BLINK
	Blynk.virtualWrite(2, digitalRead(ESP_PIN_STEPPERLDS_HOME));

	Blynk.run();
#endif


	setup_ROS(); //fa il setup dei vari nodi e legge i parametri


	LDSStatus = LDSStatus_e::ROSCONNECTED;

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








	ledSeq1(300, 3);

	ESP.wdtFeed();

//	ESP.wdtEnable(WDTO_1S);

	#if OPT_BLINK
		//Blynk.begin(auth);

		// You can also specify server:
		//Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8442);
		//Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8442);


		while (true)
		{
			Blynk.virtualWrite(2, 250 * digitalRead(ESP_PIN_STEPPERLDS_HOME));
			Blynk.virtualWrite(3, 250 * digitalRead(ESP_PIN_STEPPERLDS_END));
			Blynk.run();

		}
	#endif

	// inizia
	dbg("\n\n --- Start /scan loop ---");
	STEPPERLDS_GOHOME;
	LDSStatus = LDSStatus_e::ROSCONNECTED;
}




unsigned long nextMsg_time = 0;
long publishTime = 0;		/// tempo impiegato per pubblicare
long publishStartTime = 0;  ///inizio pubblicazione
long loopcnt = 0;



void loop() {
	while (nh.connected())
	{
		ESP.wdtFeed();

		/// //////////////////////////////////
		// Fase Scan (rotazione CCW, speed < 0)
		/// //////////////////////////////////
		if (myLDSstepper.isHomePosition())		// wait for HomePosition

		{
			myLDSstepper.goRadsPerSecond(myLDSstepper.getSpeed());

			switch (LDSStatus)
			{
			case ROSCONNECTED:	//situazione iniziale
			case ENDPUBLISH:	// situazione a regime
				// se il publish arriva lungo, lo stato viene messo a SLOWPUBLISH dall' IRS
				

				loopcnt++;

				// SCANSIONE--------------------------
				LDSStatus = LDSStatus_e::SCANNING;
				//analogWrite(Pin_LED_TOP_B, hbFreq++);
				LED_ON;			//			LASER_ON
				//long t1 = millis();
				startAcquiringDistances(&scan_msg);
				LDSStatus = LDSStatus_e::ENDSCAN;
				LED_OFF;
				//-----------------------------------



				break;


			// casi in cui non fa nulla
			case STARTING:
			case WIFICONNECTED:
			case SCANNING:
			case ENDSCAN:
			case SLOWPUBLISH:
			case PUBLISHING:
			case SLOWSCAN:
				break;


			default:
				break;
			}





			delay(1); // per il WiFi
					  //publish_laserscan(); // acquisisce le distanze...
					  //dbg("end publishing");


					  //char ros_info_msg[30]; 			sprintf(ros_info_msg, "scan loop: %d",  loopcnt);			ROS_INFO(ros_info_msg);


		}



		// / /////////////////////////////////////////////////////////////////////////////
		/// ///////////////////////////////////////////////////////////////////////////////
		// Fase Publish (da End a Home)
		/// ///////////////////////////////////////////////////////////////////////////////
		// / ///////////////////////////////////////////////////////////////////////////
		if (myLDSstepper.isEndPosition())
		{
			myLDSstepper.goRadsPerSecond(myLDSstepper.getSpeed());

			switch (LDSStatus)
			{
			case STARTING:
				break;
			case WIFICONNECTED:
				break;
			case ROSCONNECTED:
				break;
			case SCANNING:
				break;
			case ENDSCAN:	// situazione normale


				// Pubblica---------------------------------
				LDSStatus = LDSStatus_e::PUBLISHING;
				publishStartTime = millis();
				myLDSstepper.goRadsPerSecond(returnSpeed);
				LED2_ON;
				publish_laserscan(); // pubblica la scansione
				publishTime = millis() - publishStartTime;
				LED2_OFF;
				//-------------------------------------------





				if (LDSStatus == LDSStatus_e::SLOWPUBLISH) // consuma lo stato 
				{
					myLDSstepper.isHomePosition();  //clear flag homePosition

					dbg2("Slow pub!  ms ", publishTime);
					dbg2("   instead of ", lastHomeTime - lastEndTime );

				}

				LDSStatus = LDSStatus_e::ENDPUBLISH;
				
/*				
				
				//char ros_info_msg[10];sprintf(ros_info_msg, "P: %d",  loopcnt);ROS_INFO(ros_info_msg);

				//devo finire prima che arrivi a home altrimenti l'IRS di HomePosition imposta LDSStatus_e::SLOWPUBLISH
				if (LDSStatus == LDSStatus_e::SLOWPUBLISH) {
					
//					char ros_info_msg[70]; sprintf(ros_info_msg, "Slow pub %d", publishTime); nh.logwarn(ros_info_msg);

//					sprintf(ros_info_msg, "Slow down returnSpeed to [%f]rad/s",returnSpeed);
				//adatta la velocità di ritorno al tempo necessario alla pubblicazione

				}

				returnSpeed = (scan_msg.angle_max - scan_msg.angle_min) / publishTime;
				dbg2("Slow down returnSpeed to  rad/s ", returnSpeed);
*/
				//myLDSstepper.goRadsPerSecond(5);

				break;

			case PUBLISHING:
				break;
			case ENDPUBLISH:
				break;
			case SLOWPUBLISH:
				break;
			case SLOWSCAN:
				break;
			default:
				break;
			}



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
			/////////////////////////////////////////////////////////////////////

		}


	#if WEBSERVER
			server.handleClient();
	#endif
	#if OPT_BLINK
			Blynk.virtualWrite(2, 250* digitalRead(ESP_PIN_STEPPERLDS_HOME));

			Blynk.run();
	#endif

	}


	// fermo il motore
	myLDSstepper.goRadsPerSecond(0);
	dbg("##########################");
	dbg("## Lost ROS connection! ##");
	dbg("##########################");

	LDSStatus = LDSStatus_e::STARTING;
	setup_ROS(); //fa il setup dei vari nodi

	ledSeq1(200, 5);

	delay(1000);

}

