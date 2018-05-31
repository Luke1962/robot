/////////////////////////////////////////////////////////////////////
////  NODO ROS CHE GENERA Odom gestendo via interrupt due Encoder differenziali
//	WIFI MANAGER
// PARAMETRI CARICATI DA ROS
/////////////////////////////////////////////////////////////////////
/*
	launch file:


*/
/////////////////////////////////////////////////////////////////////
// PROBLEMI NOTI v1.0
/////////////////////////////////////////////////////////////////////
// P1]  gli risulta ROS non conneso anche se i messaggi li manda
// P2] non gestisce i parametri
/////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
// PARAMETRI DI COMPILAZIONE
/////////////////////////////////////////////////////////////////////
#pragma region ParametriDiCompilazione


#include <Esp32Hardware.h>
#include <ArduinoHardware.h>
#define ESP32 1

#define MBEDTLS_CONFIG_FILE "config.h"


	#include <Wire.h>		//I2C library

	#define OPT_DEBUG 1
	#define SERIAL_SPEED		 115200		//default at boot=	74880

	#define OPT_USE_SERIAL_DBG 1 // 1 manda i dati letti dal LIDAR sulla seriale (per test) ;  0 = nessun i/o su seriale
	#define SERIAL_DBG Serial

	//#include <SoftwareSerial.h>
	//SoftwareSerial SerialLidar(D2, D1, false, 1000); // RX, TX, inverse , buffer

	#define SERIAL_LDS Serial
	//#define SERIAL_LDS SerialLidar

#pragma endregion




// ///////////////////////////////////////////////////////////////////////////////
///
//		DEBUG
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region debug
	#if OPT_DEBUG
		#define dbg(s) 	SERIAL_DBG.println(s); 
		#define dbg2(s,v)  	SERIAL_DBG.print(s);SERIAL_DBG.println(v);  
		#define dbgV(...) SERIAL_DBG.printf(__VA_ARGS__)
		#define dbgf(format, ...)  SERIAL_DBG.printf(format "\n", ##__VA_ARGS__)

	#else
		#define dbg(s) 
		#define dbg2(s,v)
		#define dbgV(...)
	#endif  
#pragma endregion

// ///////////////////////////////////////////////////////////////////////////////
///
//		MACRO e Variabili globali di sistema memorizzate in EEPROM
///
// ///////////////////////////////////////////////////////////////////////////////
#define min(a,b) ((a)<(b)?(a):(b))



// ///////////////////////////////////////////////////////////////////////////////
///
//		NODEMCU HW
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region NODEMCU_HW


void isr_displayChangeWindow(); // --> sezione OLED Display




#pragma region ESP32_PIN
		#define Pin_EncRa 36
		#define Pin_EncRb 25
		#define Pin_EncLa 12
		#define Pin_EncLb 13

		//Guru Meditation Error: Core  0 panic'ed (IllegalInstruction)
		//#define Pin_EncRa 0
		//#define Pin_EncRb 2
		//#define Pin_EncLa 7
		//#define Pin_EncLb 6


								/*
								GPIO		LOLIN 32	GPIO
										------------
				Pin_EncRa		36	--- |			| --  39
				Pin_EncRb		25	--- |			| --  16	PIN_FLASHBUTTON
								26	--- |			| --   5	SDA
								8	--- |			| --   4	SCL
								11	--- |			| --   0			
								7	--- |			| --   2			
								6	--- |			| --  14			
								3.3v	|			| --  12	Pin_EncLa		
									GND	|			| --  13	Pin_EncLb
									5v	|			| --  15			PIN_LED
										|			| --   3	RX0
										|			| --   1	TX0
										|			| -- 3.3v
										|			| -- GND
										|			| -- GND
										|			| -- 5v
										|			|
										|	   USB	|
										------------
								*/



								/*

								*/

								//LED

#define PIN_LED				15
#define PIN_FLASHBUTTON		0 
								// I2C
#define PIN_SDA				5		//D2 su ESP8266
#define PIN_CK				4		//D1 su ESP8266
								// LED EXT.

#define INTERRUPT_PIN		16 // use pin 15 on ESP8266




#define IO_REG_TYPE			uint32_t
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
//#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_READ(pin)			(((*(PIN_TO_BASEREG(pin))) & (PIN_TO_BITMASK(pin))) ? 1 : 0)

#define LED_ON digitalWrite(PIN_LED, 1);
#define LED_OFF digitalWrite(PIN_LED, 0);

#define BLINK  LED_ON; delay(5); LED_OFF;
#define BLINK(ms)  LED_ON; delay(ms); LED_OFF;

#define TOGGLEPIN(pin) digitalWrite(!digitalRead(pin), pin);
#define TOGGLE_LED TOGGLEPIN(PIN_LED);


								//LASER
								//#define writeFast1(gpIO)	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 4,  (1<<gpIO));
								//#define writeFast0(gpIO) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 8,  (1<<gpIO));
								//#define writeFast(gpIO,on) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + (on?4,8),  (1<<gpIO));
								//#define TOGGLE_LED			writeFast(PIN_LED, !digitalRead(PIN_LED));

	#define GETBUTTON  !digitalRead(PIN_FLASHBUTTON)	/*negato perchè ritorna 0 se premuto*/	
	#pragma  endregion

									/// ENCODER ///////////////////////////////////////////////////






	// memoria libera ----
	// si usa così: uint32_t free = system_get_free_heap_size();
	//extern "C" {	
	//	#include "user_interface.h"	
	//}
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// STRUTTURE DATI A LIVELLO DI SISTEMA
///  
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

#pragma endregion	// NODEMCU_HW
	enum wifiStatus_e {
		DISCONNECTED,
		SMARTCONFIG,
		CONNECTED,
	};

	/// direzioni del robot
	enum robotDir_e {
		DIR_S = 0,	//S=STOP
		DIR_F,	// Forward 
		DIR_B,	//Backward
		DIR_R,	// Right
		DIR_L	// Left 
	};

	struct ros_t {
	};
	ros_t robot;
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// ENCODERS 
/// opera solo se  ogni ISR_MINMUM_INTERVAL_MSEC
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////



#if 1
	struct encoder_t {
		int32_t count;  // contatore encoder 
		int deltaCount;
		bool interruptFlag;
		long last_change;
	};


	#define	ENCODER_USE_INTERRUPTS
	#include <Encoder-master\Encoder.h>



	// Change these two numbers to the pins connected to your encoder.
	//   Best Performance: both pins have interrupt capability
	//   Good Performance: only the first pin has interrupt capability
	//   Low Performance:  neither pin has interrupt capability
	Encoder encoderL(Pin_EncLa, Pin_EncLb);
	Encoder encoderR(Pin_EncRa, Pin_EncRb);
	//   avoid using pins with LEDs attached


	void setup_encoder() {
		// void, ma tenerlo 
	}







#endif // 1



// ///////////////////////////////////////////////////////////////////////////////
///
//		MEMORIZZAZIONE PARAMETRI IN EEPROM
///
// ///////////////////////////////////////////////////////////////////////////////

#pragma region EEPROM_CONFIGURATION
	#include <EEPROMAnything.h>
	#include <EEPROM.h>
	const int N_ANGLES = 360;                // # of angles (0..359)

	struct EEPROM_Config {
		byte id;
		char version[6];

		//wifi
		char*  lastSSID;
		char*  lastPassword;
		wifiStatus_e wifiStatus;
		//int motor_pwm_pin;


		//ROS
		String ros_master_uri;


		bool rosConnected;
		char* frame_id;
		int ros_tcp_port;
		String ros_topic;

		robotDir_e dir;	// pubblicato dal nodo IMU
		encoder_t encR, encL;
		double g_steps2m;
		double g_steps2rads;
		double g_enc2steps;

		float odom_x; //posizione x
		float odom_y; //posizione y
		float odom_r; // rotazione in radianti
		float vel_x;
		float vel_y;
		float twist_z;

	}
	odomSystem;

	///////////////////////////////////////////////
	// Variabili globali del sistema di Odometria
	////////////////////////////////////////////////




	void saveConfig() {
		EEPROM_writeAnything(0, odomSystem);
		dbg(F("Config Saved."));
	}
#pragma endregion




// ///////////////////////////////////////////////////////////////////////////////


// ///////////////////////////////////////////////////////////////////////////////
///
//		WIFI
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region WIFI


	//////////////////////
	// Librerie  WiFi   //
	//////////////////////

	#include "WiFi.h"




	const char* wifi_ssid = "FASTWEB-CSRLCU";
	const char* wifi_password = "";
	#define ROS_TCP_CONNECTION_PORT 11411
	#define ROS_MASTER_URI_DEFAULT	"192.168.0.51"
	IPAddress rosIp;

	const uint16_t port = 80;
	const char * hostRos = ROS_MASTER_URI_DEFAULT; // ip or dns

												//IPAddress server(192, 168, 0, 51); // ip of your ROS server
												//IPAddress ip_address;
	int status = WL_IDLE_STATUS;



	// Use WiFiClient class to create TCP connections
	//WiFiClient WiFi; incluso nell'extern 


	/*  vecchia funzione setup_WiFi che andava su ESP8266
	*/
	// ritorna il mac address
	String getStrMacAddress() {
		String strMacAddr = "";
		uint8_t mac[4];
		WiFi.macAddress(mac);
		for (int i = 0; i < 4 - 1; i++)
		{
			strMacAddr += String(mac[i]) + ".";
		}
		strMacAddr += String(4 - 1);
		return strMacAddr;
	}



/*
void setup_wifi() {
		//Local intialization. Once its business is done, there is no need to keep it around
		WiFiManager wifiManager;
		if (!wifiManager.autoConnect("ESP32", "cesarini")) {
			#if OPT_USE_SERIAL_DBG
				SERIAL_DBG.println("failed to connect and hit timeout");
			#endif
		
			delay(3000);
			ESP.restart();
			delay(5000);
		}
		#if OPT_USE_SERIAL_DBG
			//if you get here you have connected to the WiFi
			SERIAL_DBG.print(F("WiFi connected! IP address: "));
			SERIAL_DBG.println(WiFi.localIP());
		#endif
	}
	
	
	void setup_wifi_static() {

		//WiFiManager
		//Local intialization. Once its business is done, there is no need to keep it around
		WiFiManager wifiManager;


		//set static ip
		//block1 should be used for ESP8266 core 2.1.0 or newer, otherwise use block2

		//start-block1
		//IPAddress _ip,_gw,_sn;
		//_ip.fromString(static_ip);
		//_gw.fromString(static_gw);
		//_sn.fromString(static_sn);
		//end-block1

		//start-block2
		IPAddress _ip = IPAddress(10, 0, 1, 78);
		IPAddress _gw = IPAddress(10, 0, 1, 1);
		IPAddress _sn = IPAddress(255, 255, 255, 0);
		//end-block2

		wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);


		//tries to connect to last known settings
		//if it does not connect it starts an access point with the specified name
		//here  "AutoConnectAP" with password "password"
		//and goes into a blocking loop awaiting configuration
		if (!wifiManager.autoConnect("ESP32", "cesarini")) {
			Serial.println("failed to connect, we should reset as see if it connects");
			delay(3000);
			ESP.restart();
			delay(5000);
		}

		//if you get here you have connected to the WiFi
		Serial.println("connected...yeey :)");


		Serial.println("local ip");
		Serial.println(WiFi.localIP());


	}

*/


#pragma endregion



// ///////////////////////////////////////////////////////////////////////////////
///
//		OLED DISPLAY  128x64 
///
// ///////////////////////////////////////////////////////////////////////////////



#pragma region OledDisplay 
	//https://github.com/ThingPulse/esp8266-oled-ssd1306
	#define DISPLAY_FRAMES 5	// Numero di schede 0: messaggi, 1: compass, 2: quaternions, 3:config
	#define DISPLAY_SIZE_X 128
	#define	DISPLAY_SIZE_Y 64

	//int displayCurrentFrameIndex = 0; // scheda corrente, incrementata dal pulsante flash 
	static volatile uint8_t displayCurrentFrameIndex = 0; // this variable will be changed in the ISR, and Read in main loop
												  // static: says this variable is only visible to function in this file, its value will persist, it is a global variable
												  // volatile: tells the compiler that this variables value must not be stored in a CPU register, it must exist
												  //   in memory at all times.  This means that every time the value of intTriggeredCount must be read or
												  //   changed, it has be read from memory, updated, stored back into RAM, that way, when the ISR 
												  //   is triggered, the current value is in RAM.  Otherwise, the compiler will find ways to increase efficiency
												  //   of access.  One of these methods is to store it in a CPU register, but if it does that,(keeps the current
												  //   value in a register, when the interrupt triggers, the Interrupt access the 'old' value stored in RAM, 
												  //   changes it, then returns to whatever part of the program it interrupted.  Because the foreground task,
												  //   (the one that was interrupted) has no idea the RAM value has changed, it uses the value it 'know' is 
												  //   correct (the one in the register).  

	//ISR CHE INCREMENTA SOLAMENTE L'INDICE DELL FRAME CORRENTE
	//IRAM_ATTR tells the complier, that this code Must always be in the 
	// ESP32's IRAM, the limited 128k IRAM.  use it sparingly.
	static volatile long isr_displayLastTime;
	void IRAM_ATTR isr_displayChangeWindow() {
		unsigned long now = millis();
		
		// If interrupts come faster than 200ms, assume it's a bounce and ignore
		if (now - isr_displayLastTime > 500)
		{
			displayCurrentFrameIndex += 1;
			if (displayCurrentFrameIndex >= DISPLAY_FRAMES) { displayCurrentFrameIndex = 0; }
			isr_displayLastTime = now; 

		}


	}
	#include <TimeLib.h>
	#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
	#include "SSD1306.h" // alias for #include "SSD1306Wire.h" driver for the SSD1306 based 128x64 pixel OLED display 

	// Include the UI lib
	#include "OLEDDisplayUi.h"
	#include <CircularBuffer\CircularBuffer.h>
	CircularBuffer<String, 4> displayBufferMsgs;
	//	CircularBuffer<String, 4> displayBufferQuat;

	// Include custom images
	// #include "images.h"
	const char activeSymbol[] PROGMEM = {
		B00000000,
		B00000000,
		B00011000,
		B00100100,
		B01000010,
		B01000010,
		B00100100,
		B00011000
	};

	const char inactiveSymbol[] PROGMEM = {
		B00000000,
		B00000000,
		B00000000,
		B00000000,
		B00011000,
		B00011000,
		B00000000,
		B00000000
	};


	// Initialize the OLED display using Wire library
	SSD1306  display(0x3c, PIN_SDA, PIN_CK);

	OLEDDisplayUi ui(&display);

	int screenW = DISPLAY_SIZE_X;
	int screenH = DISPLAY_SIZE_Y;
	int clockCenterX = screenW / 2;
	int clockCenterY = ((screenH - 16) / 2) + 16;   // top yellow part is 16 px height
	int clockRadius = 23;

	void drawProgressBarDemo(int percent) {

		// draw the progress bar
		display.drawProgressBar(0, 32, 120, 10, percent);

		// draw the percentage as String
		display.setTextAlignment(TEXT_ALIGN_CENTER);
		display.drawString(64, 15, String(percent) + "%");
	}
	void drawTextFlowDemo(String s) {
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawStringMaxWidth(0, 0, 128, s);
	}


	void  displayFrame(int frameNo );
	#define DISPLAY_BUFF_ROWS 4 /*righe che il dislay è in grado di visualizzare*/
	#define DISPLAY_INTERLINEA 11
	#define DISPLAY_HEADER_SIZE_Y 13
	#define DISPLAY_ROW(r) DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (r-1)
	#define dbgD(s) displayBufferMsgs.push(s);  displayFrame(-1);  //display.display();//displayCircularBuffer(false,false);
	void displayCircularBufferMsgs(bool blClear = false, bool blDisplay = true) {
		// Initialize the log buffer
		// allocate memory to store 8 lines of text and 30 chars per line.
		display.setLogBuffer(DISPLAY_BUFF_ROWS, 30);

		// distanza in pixel tra due righe; dipende dal font size
		if (blClear)
		{
			display.clear();
		}
		// 4 linee x 21 caratteri
		for (uint8_t i = 1; i <= DISPLAY_BUFF_ROWS; i++) {
			// Print to the screen
			display.drawString(0, DISPLAY_ROW(i), displayBufferMsgs[i]);
			//display.println(displayBuffer[i]);


			// Draw it to the internal screen buffer
			display.drawLogBuffer(0, 0);
		}
		if (blDisplay)// Display it on the screen
		{
			display.display();

		}
	}


	// disegna un cerchio e un raggio orientato secondo l'angolo rad
	void displayCompass(float rad) {
		// offset da impostare per allineare la barra sul display all'asse x del magnetometro
		// positivo in senso orario
		#define OFFSET PI/2

		#define CENTER_X 80
		#define CENTER_Y 38
		#define RADIUS 22
		display.drawCircle(CENTER_X, CENTER_Y, RADIUS);
		int dx = round(RADIUS* cos(-rad + OFFSET));
		int dy = round(RADIUS* sin(-rad + OFFSET));
		display.drawLine(CENTER_X, CENTER_Y, CENTER_X + dx, CENTER_Y + dy);
	}

	void setup_display() {
		// Sets the current font. Available default fonts
		// ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
		display.setFont(ArialMT_Plain_10);


		// The ESP is capable of rendering 60fps in 80Mhz mode
		// but that won't give you much time for anything else
		// run it in 160Mhz mode or just set it to 30 fps
		ui.setTargetFPS(30);

		// Customize the active and inactive symbol
		ui.setActiveSymbol(activeSymbol);
		ui.setInactiveSymbol(inactiveSymbol);

		// You can change this to
		// TOP, LEFT, BOTTOM, RIGHT
		ui.setIndicatorPosition(TOP);

		// Defines where the first frame is located in the bar.
		ui.setIndicatorDirection(LEFT_RIGHT);

		// You can change the transition that is used
		// SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
		ui.setFrameAnimation(SLIDE_LEFT);

		// Add frames
		//ui.setFrames(frames, frameCount);

		// Add overlays
		//ui.setOverlays(overlays, overlaysCount);

		// Initialising the UI will init the display too.
		ui.init();

		display.flipScreenVertically();

		

		display.display();


		dbg("\nOled Display setup complete\n");
		dbgD("###################");
		dbgD("# ESP32 Odom ROS  #");
		dbgD("###################");


	}



	inline void displayFrameHeader(String frameName) {
		display.clear();
		String s = String(displayCurrentFrameIndex) + "] " + frameName;
		display.drawString(0, 0, s); //titolo											 
		display.drawString(90, 0, String(millis() / 1000) + "''");//millisecondi trascorsi
	}


	// Se frameNo <0 => Visualizza il frame corrente (incrementato da interrupt o dal pulsante flash)
#define DISPLAY_FRAME_MESSAGGI	0
#define DISPLAY_FRAME_ENCODER	1
#define DISPLAY_FRAME_ROS		2
#define DISPLAY_FRAME_ODOM		3
#define DISPLAY_FRAME_WIFI		4

	void  displayFrame(int frameNo = -1) {

		if (GETBUTTON) /*negato perchè ritorna 0 se premuto*/
		{
			displayCurrentFrameIndex += 1;
			if (displayCurrentFrameIndex >= DISPLAY_FRAMES) { displayCurrentFrameIndex = 0; }

		}


		String s;
		uint8_t zBar = 0;
		#define DISPLAY_MAX_RANGE_PIXELS 40
		int rescaledDist = 0;



		if (frameNo<=-1)
		{
			frameNo = displayCurrentFrameIndex;
		}
		else if(frameNo >= DISPLAY_FRAMES)
		{
			frameNo = DISPLAY_FRAMES % frameNo;
		}

		switch (frameNo)
		{
		case DISPLAY_FRAME_MESSAGGI:
			//messaggi		
			display.clear();
			displayFrameHeader("Messaggi");
			displayCircularBufferMsgs(false, false);
			display.display();

			break;

		case DISPLAY_FRAME_ENCODER:
			//Encoder values
			display.clear();
			displayFrameHeader("[Encoder ]");

			//s = "Enc L: " + String((int)encoderL.read());
			s = "Enc L: " + String((int)odomSystem.encL.count);
			display.drawString(0, DISPLAY_ROW(1), s);
			
			s = "Pin Enc L a: " + String(Pin_EncLa) + ", b " + String(Pin_EncLb);
			display.drawString(0, DISPLAY_ROW(2), s);



			s = "Enc R: " + String((int)odomSystem.encR.count);
			//s = "Enc R: " + String((int)encoderR.read());
			display.drawString(0, DISPLAY_ROW(3), s);

			s = "Enc R Pin a: " + String(Pin_EncRa) + ", b " + String(Pin_EncRb);
			display.drawString(0, DISPLAY_ROW(4), s);


			display.display();
			break;			
		case DISPLAY_FRAME_ROS:
			// ROS
			displayFrameHeader("[ROS]");

			s =odomSystem.rosConnected ? "[ROS CONNECTED]" :"[ROS NOT CONNECTED]";
			display.drawString(0, DISPLAY_ROW(1), s);
			s = "ROS URI:" + odomSystem.ros_master_uri;
			display.drawString(0, DISPLAY_ROW(2), s);
			s = "TCP PORT: " + String(odomSystem.ros_tcp_port);
			display.drawString(0, DISPLAY_ROW(3), s);



			display.display();
			break;
		case DISPLAY_FRAME_WIFI:
			//WIFI

			display.clear();
			displayFrameHeader("[WiFi Config]");


			switch (odomSystem.wifiStatus)
			{
			case	wifiStatus_e::CONNECTED		:
				//s = WiFi.isConnected() ? "WiFi connected" : "Wifi NOT Connected";
				s = "WiFi connected" ;
				display.drawString(0, DISPLAY_ROW(1), s);
				s = "SSID:" + WiFi.SSID();
				display.drawString(0, DISPLAY_ROW(2), s);

				s = "IP:" + WiFi.localIP().toString();
				display.drawString(0, DISPLAY_ROW(3), s);
				//s = "Mac:" +String( WiFi.macAddress);
				s = "Mac:" + getStrMacAddress();
				display.drawString(0, DISPLAY_ROW(4), s);
				break;

			case	wifiStatus_e::SMARTCONFIG	:	
				s = "USE SMARTCONFIG!!" ;
				display.drawString(0, DISPLAY_ROW(1), s);
				break;

			case	wifiStatus_e::DISCONNECTED	:

				s ="Wifi NOT Connected";
				display.drawString(0, DISPLAY_ROW(1), s);

				break;
			default:
				break;


			}



			//s = "MQTT:" + String(mqtt_server);
			//display.drawString(0, DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (2), s);

			display.display();
			break;

		case DISPLAY_FRAME_ODOM:
			// ROS
			displayFrameHeader("[ROS odometry]");

			s = "topic: " + odomSystem.ros_topic;
			display.drawString(0, DISPLAY_ROW(1), s);

			//s = "frame id: " + String(odomSystem.frame_id);
			//display.drawString(0, DISPLAY_ROW(2), s);


			s = "x: " +String(odomSystem.odom_x,3);
			display.drawString(0, DISPLAY_ROW(2), s);
			s = "y: " + String(odomSystem.odom_y, 3);
			display.drawString(0, DISPLAY_ROW(3), s);
			s = "r: " + String(odomSystem.odom_r, 3);
			display.drawString(0, DISPLAY_ROW(4), s);

			displayCompass(odomSystem.odom_r);


			display.display();
			break;


		default:
			break;

		}


	}



#pragma endregion  // OLED DISPLAY


// ///////////////////////////////////////////////////////////////////////////////
///
//		HELPER FUNCTIONS
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region HelperFunctions


	enum systemStatus_e {
		STARTING = 0,
		WIFICONNECTED,
		ROSCONNECTED,
		RUNNING,

	};

	systemStatus_e SystemStatus;


	void ledSeq1(int ms, int times = 3) {
		pinMode(LED_BUILTIN, OUTPUT);
		for (int i = 0; i < times; i++)
		{
			digitalWrite(LED_BUILTIN, 0); delay(ms);
			digitalWrite(LED_BUILTIN, 1); delay(ms);

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





	void printESPinfo() {
		dbg2("CPU freq.", ESP.getCpuFreqMHz());
		dbg2("Flash chip Size: ", ESP.getFlashChipSize());

	}

	void softReset() {
		#if OPT_USE_SERIAL_DBG
			SERIAL_DBG.println("Restarting...");
			//SERIAL_DBG.println(WiFi.status());
			//esp_wifi_wps_disable() // add this, okay
		#endif
		ESP.restart();
	}
#pragma endregion	//HelperFunctions








// ///////////////////////////////////////////////////////////////////////////////
///
//		ROS
///
// ///////////////////////////////////////////////////////////////////////////////

#pragma region ROS MACRO REGION
	//	#define ROS_MASTER_URI_DEFAULT "192.168.0.51"
	//String ros_master_uri ;  --> LSD.ros_master_uri
	//#include <Esp32Hardware.h> //#include <ArduinoHardware.h>
	#include <ros.h>
	#include <ros/time.h>
	#include <std_msgs/String.h>



	//--------------------------------
	// TF 
	//--------------------------------
	#include <tf/tf.h>
	#include <tf/transform_broadcaster.h>
	tf::TransformBroadcaster tfBr;
	geometry_msgs::TransformStamped t;

	WiFiClient tcpClient;



	void chatterCallback(const std_msgs::String& msg) {
		dbgD(msg.data);
			
	}

	ros::Subscriber<std_msgs::String> sub("message", &chatterCallback);




	// questo è come in espRosScan che funziona
	ros::NodeHandle nh;		// con aggiunta in Ros.h di: 	typedef NodeHandle_<Esp8266Hardware, 25, 25, 512, 1024> NodeHandle;



		//std_msgs::String str_msg;
		//ros::Publisher chatter("chatter", &str_msg);
		////--------------------------------
		//sensor_msgs::Range rosmsg_range;
		//ros::Publisher pub_range("ultrasound", &rosmsg_range);

		//--------------------------------
		#define ROS_INFO(s) nh.loginfo(s);
		
		#define TSK_ROS_HZ 50.0f //frequenza alla quale deve girare il task





		const int adc_pin = 0;
		double x = 1.0;
		double y = 0.0;
		double theta = 1.57;

		unsigned long g_prev_command_time = 0;

		byte hbFreq = 10; //heartbeat


		/// ////////////////////////////////////////////////////////////////////////////////////////////////////
		/// R O S
		/// O D O ME T R Y
		/// ////////////////////////////////////////////////////////////////	
	#pragma region ROS_ODOMETRY
		#include <nav_msgs/Odometry.h>
		nav_msgs::Odometry msg_odom;
		ros::Publisher pub_odom("/odom", &msg_odom);

		#include <std_msgs/Int32.h>
		#include <geometry_msgs/PoseWithCovarianceStamped.h>		// per InitialPose
		std_msgs::Int32 msg_encoderCountL;
		std_msgs::Int32 msg_encoderCountR;
		ros::Publisher pub_encL("/encoderL", &msg_encoderCountL);
		ros::Publisher pub_encR("/encoderR", &msg_encoderCountR);



		// pubblica il messaggio sul topic msg-topic
		inline void publish_odometry() {

			pub_encL.publish(&msg_encoderCountL);
			pub_encR.publish(&msg_encoderCountR);
			pub_odom.publish(&msg_odom);

		}

		/// ////////////////////////////////////////////////////////////////////////////////////////////////////
		/// versione che usa un solo encoder ,
		/// assumendo 0 drift laterale
		/// ////////////////////////////////////////////////////////////////	
		#include "parameters.h"

		#define INTERVAL_ODOM_UPDATE_MSEC 100  /// ogni quanto aggiorno l'odom'. limitato da rosRate
		long g_last_robotOdom_update_time = 0;

		//Aggiorna odomSystem.odom_x ,y,z e odomSystem.vel_x ,y,z sulla base del valore incrementale di conteggio degli encoder

		//versione senza attesa per multitasking
		void odom_msg_update(nav_msgs::Odometry* msg_odom,  int32_t encL, int32_t encR, bool blCountIsAbsolute=true) {
			//Aggiorno i contatori totali
			if (blCountIsAbsolute)
			{
				odomSystem.encL.deltaCount = encL - odomSystem.encL.count;
				odomSystem.encR.deltaCount = encR - odomSystem.encR.count;
				odomSystem.encL.count = encL;
				odomSystem.encR.count = encR;

			}
			else
			{
				odomSystem.encL.deltaCount = encL ;
				odomSystem.encR.deltaCount = encR ;
				odomSystem.encL.count += encL;
				odomSystem.encR.count += encR;

			}

			msg_encoderCountL.data = odomSystem.encL.count;
			msg_encoderCountR.data = odomSystem.encR.count;

			if (abs(odomSystem.encL.deltaCount) + abs(odomSystem.encR.deltaCount) > 0) ///> 0 se si è mosso
			{

			//dbg("#")

				/// I delta sono calcolati rispetto a /base_link
				//				double deltaStepsForward = (float)((encL.count+encR.count)/2)*(float)ROBOT_MOTOR_STEPS_PER_ENCODERTICK ; ///somma 0 se ruota				
				//				double deltaStepsRotationCCW = 0.5*(encRcount-encLcount)*(float )ROBOT_MOTOR_STEPS_PER_ENCODERTICK ;
				double deltaStepsForward = (float)((odomSystem.encL.deltaCount + odomSystem.encR.deltaCount) / 2)*(float)odomSystem.g_enc2steps; ///somma 0 se ruota				
				double deltaStepsRotationCCW = 0.5*(odomSystem.encR.deltaCount - odomSystem.encL.deltaCount)*(float)odomSystem.g_enc2steps;
			//dbg("@1")

				//				double deltaForward_m = deltaStepsForward *(float)ROBOT_STEPS2M; 
				//				double deltaTheta_rad = (float )deltaStepsRotationCCW* ROBOT_STEPS2RAD /2;/// converto in angolo 
				double deltaForward_m = deltaStepsForward *(float)odomSystem.g_steps2m;
				double deltaTheta_rad = (float)deltaStepsRotationCCW* odomSystem.g_steps2rads;/// converto in angolo 

			//dbg("@2")


				// delta spostamento in metri rispetto al frame /odom
				double deltaOdom_x = deltaForward_m * cos(odomSystem.odom_r);
				double deltaOdom_y = deltaForward_m * sin(odomSystem.odom_r);

			//dbg("@3")
					
				///calculate current position of the robot vs /odom frame
				odomSystem.odom_x += deltaOdom_x;
				odomSystem.odom_y += deltaOdom_y;
				odomSystem.odom_r += deltaTheta_rad;
				if (odomSystem.odom_r > 2*PI)
				{
					odomSystem.odom_r -=2 * PI;
				}
					
			//dbg("@4")


				/// compute velocity
				//ros::Time now = ros::Time::now();
				//double dt = (now - g_last_robotOdom_update_time).toSec();
				long now = millis();
				long dt = now -g_last_robotOdom_update_time;
				g_last_robotOdom_update_time = now;

				float dtSec = dt / 1000;
				odomSystem.vel_x = deltaOdom_x / dtSec;
				odomSystem.vel_y = deltaOdom_y / dtSec;
				odomSystem.twist_z = deltaTheta_rad / dtSec;
			//dbg("@5")

				msg_odom->pose.pose.position.x = odomSystem.odom_x;
				msg_odom->pose.pose.position.y = odomSystem.odom_y;
				msg_odom->pose.pose.orientation.z = odomSystem.odom_r;

				msg_odom->twist.twist.linear.x = odomSystem.vel_x;
				msg_odom->twist.twist.linear.y = odomSystem.vel_y;
				msg_odom->twist.twist.angular.z = odomSystem.twist_z;
			//dbg("@6")
			}


		}	// end odom_msg_update()
		#pragma endregion // ROS ODOMETRY


	/// ///////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////
	///								
	///	FUNZIONI PER CARICARE I PARAMETRI ROS DAI FILE *.YAML					
	///								
	/// ///////////////////////////////////////////////////////////////
	/// ////////////////////////////////////////////////////////////////
	#pragma region ROS_PARAMETERS
		
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


		//int loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, int defaultValue) {
		int loadRosParameter(ros::NodeHandle* nh, const char* parName, int defaultValue) {
			String parPath = thisNs + thisNode + "/" + parName;
			int parValue = defaultValue;
			//ESP.wdtFeed();
			if (nh->getParam(parPath.c_str(), &parValue))
			{
				printf("\n Loaded int   [%s]:\t %d", parName, parValue);
				char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded int param  [%s]:\t %d", parName, parValue); nh->loginfo(ros_info_msg);

				return parValue;
			}
			else
			{
				char ros_info_msg[200]; 
				sprintf(ros_info_msg, "***Warning*** int param. [%s] not found. Using default value %f ", parPath.c_str()); nh->logwarn(ros_info_msg);
				sprintf(ros_info_msg,"  Using default value %d ", defaultValue); nh->logwarn(ros_info_msg);
				printf("***Warning*** Int param. [%s] not found. \n\t\t Using default value %d \n\n", parPath.c_str(), defaultValue);
				printf(parPath.c_str());
				return defaultValue;
			}
		}
		//float loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, float defaultValue) {
		float loadRosParameter(ros::NodeHandle* nh, const char* parName, float defaultValue) {

			String parPath = thisNs + thisNode + "/" + parName;
			float parValue = defaultValue;
			//ESP.wdtFeed();//necessario
			if (nh->getParam(parPath.c_str(), &parValue))
			{
				String parValueStr = ftoa(parValue, 4, 10);
				printf("Loaded float [%s]:\t %s", parName, parValueStr.c_str());  // prinf con %f non è supportato
				char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded float param [%s]:\t  %s", parName, parValueStr.c_str()); nh->loginfo(ros_info_msg);
				return parValue;
			}
			else
			{
				char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** float param. [%s] not found. Using default value %s ", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str()); nh->logwarn(ros_info_msg);
				printf(" ***Warning*** float param. [%s] not found. \n\t \tUsing default value %s \n\n", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str());
				return defaultValue;
			}
		}

		/**/


		/**/
		//String loadRosParameterStr(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, String defaultValue) {
		String loadRosParameterStr(ros::NodeHandle* nh, const char* parName, String defaultValue) {

			String parPath = thisNs + thisNode + "/" + parName;
			char**  parValuePtr;
			defaultValue.toCharArray(*parValuePtr,10);

			if (nh->getParam(parName, parValuePtr))
			{
				printf("\n Loaded [%s ]: %s", parName, parValuePtr);
				String returnStr;
				
				return String(*parValuePtr);
			}
			else
			{
				printf("\n ***Warning***  [%s] not found. Using default value %s", parPath.c_str(), defaultValue.c_str());
				return defaultValue;
			}
		}



		void connectToServer(char* host, uint16_t httpPort =21) {
			dbgD("connecting to ");
			dbgD(host);

			if (!tcpClient.connect(host, httpPort)) {
				dbgD("connection failed");
				return;
			}
		}
		


		void ROS_loadParameters() {




			odomSystem.odom_x = loadRosParameter(&nh, "x", (int)0);
			odomSystem.odom_y = loadRosParameter(&nh, "y", (int)0);
			odomSystem.odom_r = loadRosParameter(&nh, "r", (int)0);

		}


	#pragma endregion // ROS_PARAMETERS

	/// ///////////////////////////////////////////////////////////////
	/// ////////////////////////////////////////////////////////////////
	#pragma region Setup_ROS
		void ros_initialize_old(String rosServer) {
			IPAddress rosIp;
			rosIp.fromString(rosServer);

			dbg2("ros_initialize()]Connecting to Ros core  ", rosServer);
			// Ros objects constructors   
			nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);
			nh.initNode();
			tfBr.init(nh);
/**/
			while (!nh.connected()) {
				if (!tcpClient.connected())
				{
					tcpClient.connect(rosIp, ROS_TCP_CONNECTION_PORT);

				}



				nh.spinOnce();

			}

			if (tcpClient.connected())
			{
				
				dbgD("ROS Connected");
				dbg("\ROS Connected");

				// recupero parametri-------------------
				///motor_rpm = loadRosParameter(&nh,"scan_speed_rpm", 300);
				//odomSystem.ros_topic = loadRosParameterStr(&nh, "topic", "odom");

				// esegue gli advertise----------
				//nh.advertise(chatter);
				msg_odom.header.frame_id = "base_link";
				msg_odom.pose.pose.position.x = 0.0;

				
				nh.advertise(pub_odom);





				nh.spinOnce();
				//delay(3000);
				dbg("\n[setup_ROS()] End");
				ROS_INFO("[setup_ROS()] End");


			}
			else
			{


				dbgD("ROS NOT AVAILABLE");
				dbg("\ROS NOT AVAILABLE");

			}

		
		
		
		}
		void setup_ROS(String rosServer, bool firstTime = true) {
			odomSystem.ros_master_uri = ROS_MASTER_URI_DEFAULT;

			//attende la connessione wifi
			while (!odomSystem.wifiStatus == wifiStatus_e::CONNECTED) { yield(); }


			if (firstTime)
			{
				ros_initialize(rosServer);
			}


			//"no more advertise admitted...");
			while (!nh.connected()) {
				odomSystem.rosConnected = false;

				nh.spinOnce();
				delay(1000);
				BLINK(500);

				///gestione pagina per impostare i parametri ROS  
				#if  WEBSERVER
						server.handleClient();
				#endif

				displayFrame();

			}

			// qui ci deve arrivare solo quando si è connesso a ROS
			//LDSStatus = LDSStatus_e::ROSCONNECTED;
			odomSystem.rosConnected = true;
			dbgD("CONNESSO A ROS");
			nh.loginfo("## ESP CONNESSO A ROS ##");
			delay(5);

			//ROS_loadParameters();




		}
		void ros_initialize(String rosServer) {
			IPAddress rosIp;
			rosIp.fromString(rosServer);
			odomSystem.rosConnected = false;

			dbg2("ros_initialize()] Connecting to Ros core  ", rosServer);
			// Ros objects constructors   
			nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);
			nh.initNode();


			// inizializzo tf
			tfBr.init(nh);


			//inizializzo i messaggi
			msg_odom.header.stamp = nh.now();
			msg_odom.header.frame_id = "odom"; 
			msg_odom.child_frame_id ="base_link";
			msg_odom.pose.pose.position.x = 0.0;
			msg_odom.pose.pose.position.y = 0.0;
			msg_odom.pose.pose.position.z = 0.0;

			msg_odom.pose.pose.orientation.x = 0.0;
			msg_odom.pose.pose.orientation.y = 0.0;
			msg_odom.pose.pose.orientation.z = 0.0;
			msg_odom.pose.pose.orientation.w = 1.0;
			for (int i = 0; i < 36; i++)
			{
				msg_odom.pose.covariance[i] = 0.0;
			}
			//set the covariance
			msg_odom.pose.covariance[0] = 0.2; // xx
			msg_odom.pose.covariance[7] = 0.2;// yy
			msg_odom.pose.covariance[14] = 1e100;// zz
			msg_odom.pose.covariance[21] = 1e100; // rr
			msg_odom.pose.covariance[28] = 1e100;// pp
			msg_odom.pose.covariance[35] = 0.2; // yawyaw






			msg_encoderCountL.data = 0; //contatori assoluti
			msg_encoderCountR.data = 0;







			nh.advertise(pub_odom);
			nh.advertise(pub_encL);
			nh.advertise(pub_encR); //contatori assoluti

			nh.spinOnce(); //chiama negoziateTopics che asua volta pone conneted = true;
			if (nh.connected())
			{
				odomSystem.rosConnected = true;
				dbgD("ROS Connected");
				dbg("ros_initialize()] ROS Connected");

			}


			dbg("[ros_initialize()] End");
			ROS_INFO("[ros_initialize()] End");


 
		
		}


		/// inzializza  odom 
		void odom_set(const geometry_msgs::Point &p, const geometry_msgs::Quaternion  &quat_msg) {


			msg_odom.pose.pose.position = p;

			msg_odom.pose.pose.orientation = quat_msg;

			////ROS uses 2 quaternion datatypes: msg and 'tf.' To convert between them, use the methods of tf/transform_datatypes:
			//tf::Quaternion quat_tf;
			//quaternionMsgToTF(quat_msg, quat_tf);


			//tf::Matrix3x3 m(quat_tf);
			//double roll, pitch, yaw;
			//m.getRPY(roll, pitch, yaw);
			//robotOdom.r = (float)yaw;
		}

		void cbk_initialPose(const geometry_msgs::PoseWithCovarianceStamped& initialpose) {


			odom_set(initialpose.pose.pose.position, initialpose.pose.pose.orientation);

			//static tf2_ros::TransformBroadcaster tfbr;
			geometry_msgs::TransformStamped tf;

			tf.header.stamp = ros::Time::now();
			tf.header.frame_id = "odom";
			tf.child_frame_id = "base_link";



			tf.transform.translation.x = initialpose.pose.pose.position.x;
			tf.transform.translation.y = initialpose.pose.pose.position.y;
			tf.transform.translation.z = initialpose.pose.pose.position.z;


			//tf2::Quaternion q;
			//q.setRPY(0, 0, odom->r);
			tf.transform.rotation.x = initialpose.pose.pose.orientation.x; // q.x();
			tf.transform.rotation.y = initialpose.pose.pose.orientation.y; //q.y();
			tf.transform.rotation.z = initialpose.pose.pose.orientation.z; //q.z();
			tf.transform.rotation.w = initialpose.pose.pose.orientation.w; //q.w();

			tfBr.sendTransform(tf);

			printf("\nSet pose to x:%f ,y:%f", initialpose.pose.pose.position.x, initialpose.pose.pose.position.y);
			ROS_INFO("Set pose to initilpose");


		}

	#pragma endregion	// Setup_ROS


#pragma endregion ROS macro region

// inizia ad acquisire n[samples] con cadenza costante data da  [time_interval_sec]  (intervallo dato da dt)
// debug su seriale delle distanze







// ///////////////////////////////////////////////////////////////////////////////
///
//		W E B  S E R V E R
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region webServer
	#include <WiFi.h>
	#include <FS.h>
	#include <AsyncTCP.h>
	#include <ESPAsyncWebServer.h>


	const char *ESP32_ssid = "ESP32";
	const char *ESP32_password = "123";


	// l'ESP32 diventa un Access Point nel cas non riesca a connettermi al WiFi
	void setup_AP() {
//		Serial.begin(115200);

		WiFi.softAP(ESP32_ssid, ESP32_password);

		dbg2("\n IP address: ",WiFi.softAPIP());
		dbgD("IP addr: " + WiFi.softAPIP().toString());


		//server.on("/hello", HTTP_GET, [](AsyncWebServerRequest *request) {
		//	request->send(200, "text/plain", "Hello World");
		//});

		//server.begin();
	}

	//const char* ssid = "yourNetworkSSID";
	//const char* password = "yourNetworkPassword";

	AsyncWebServer server(80);

	const char HTML[] PROGMEM = "<form onSubmit=\"event.preventDefault()\"><label class=\"label\">Network Name</label><input type=\"text\" name=\"ssid\"/><br/><label>Password</label><input type=\"text\" name=\"pass\"/><br/><input type=\"submit\" value=\"Submit\"></form>";
	//----------------------------
	void handleWiFi() {
		dbg("\n handleWiFi");
		//if (server.hasArg("SSID") && server.hasArg("PASSWORD")) {
		//	ssid = server.arg("SSID");
		//	password = server.arg("PASSWORD");
		//}
		//server.send(200, "text/html", content);
		AsyncWebServerRequest *request;
		//	request->send(200, "text/html", content);
	}




	//----------------------------------
	void setup_webserver()
	{
		//Serial.begin(115200);

		//WiFi.begin(ssid, password);

		//while (WiFi.status() != WL_CONNECTED) {
		//	delay(1000);
		//	Serial.println("Connecting to WiFi..");
		//}

		//Serial.println(WiFi.localIP());


		server.onNotFound([](AsyncWebServerRequest *request) {
			Serial.printf("NOT_FOUND: ");
			if (request->method() == HTTP_GET)
				Serial.printf("GET");
			else if (request->method() == HTTP_POST)
				Serial.printf("POST");
			else if (request->method() == HTTP_DELETE)
				Serial.printf("DELETE");
			else if (request->method() == HTTP_PUT)
				Serial.printf("PUT");
			else if (request->method() == HTTP_PATCH)
				Serial.printf("PATCH");
			else if (request->method() == HTTP_HEAD)
				Serial.printf("HEAD");
			else if (request->method() == HTTP_OPTIONS)
				Serial.printf("OPTIONS");
			else
				Serial.printf("UNKNOWN");
			Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

			if (request->contentLength()) {
				Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
				Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
			}

			int headers = request->headers();
			int i;
			for (i = 0; i < headers; i++) {
				AsyncWebHeader* h = request->getHeader(i);
				Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
			}

			int params = request->params();
			for (i = 0; i < params; i++) {
				AsyncWebParameter* p = request->getParam(i);
				if (p->isFile()) {
					Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
				}
				else if (p->isPost()) {
					Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
				}
				else {
					Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
				}
			}

			request->send(404);
		});



		server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {


			// Metodo2:  mediante string ------------------------------------
			String msg = "CIAO";

			String content = "<html><body><form action='/login' method='POST'>To log in, please use : admin/admin<br>";
			content += "User:<input type='text' name='SSID' placeholder='user name'><br>";
			content += "Password:<input type='password' name='PASSWORD' placeholder='password'><br>";
			content += "<input type='submit' name='SUBMIT' value='Submit'></form>" + msg + "<br>";
			content += "You also can go <a href='/inline'>here</a></body></html>";

			//was request->send(200, "text/html", HTML);
			request->send(200, "text/html", content);
		});





		server.on("/html", HTTP_POST, [](AsyncWebServerRequest *request)
		{
			int params = request->params();
			dbgf("Save settings, %d params", params);
			for (int i = 0; i < params; i++) {
				AsyncWebParameter* p = request->getParam(i);
				//if (p->isFile()) {
				//	dbgf("_FILE[%s]: %s, size: %u", p->name().c_str(), p->value().c_str(), p->size());
				//}
				//else 
				if (p->isPost()) {
					dbgf("_POST[%s]: %s", p->name().c_str(), p->value().c_str());
				}
				else {
					dbgf("_GET[%s]: %s", p->name().c_str(), p->value().c_str());
				}
			}

		});

		server.begin();
	}

	//
	void handlerWebServer() {
		dbg("###  handlerWebServer");

	}
#pragma endregion

// ///////////////////////////////////////////////////////////////////////////////
///
//	FREERTOS	TASKS 
///
// ///////////////////////////////////////////////////////////////////////////////
	#pragma region RTOS_Tasks



	// ///////////////////////////////////////////////////////////////////////////////
	///
	//	SEMAFORI	FREE RTOS
	///
	// ///////////////////////////////////////////////////////////////////////////////
		/* initialize binary semaphore */
		//usare con  xSemaphoreTake(S, portMAX_DELAY);  e xSemaphoreGive(S);		http://esp32.info/docs/esp_idf/html/d0/d2f/group__xSemaphoreGive.html
		SemaphoreHandle_t sem_odom_msg = xSemaphoreCreateBinary();

		SemaphoreHandle_t sem_wifi = xSemaphoreCreateBinary();
		SemaphoreHandle_t sem_ros = xSemaphoreCreateBinary();


		TaskHandle_t h_ros;
		TaskHandle_t h_display;
		TaskHandle_t h_wifi;




		//---------------------------------------------------------------------------------------
		// task WIFI
		//---------------------------------------------------------------------------------------
		const TickType_t xDelay_wifi = 1000 / portTICK_PERIOD_MS;/* 10Hz */
		void tsk_wifi(void * parameter) {
			//---------------------------------------------------------------------------------------
			// setup WIFI
			dbg("\n[tsk_wifi STARTED... ]");



			//		dbgD("setup WiFi... ");

			//WiFi.begin();
			WiFi.begin("FASTWEB-CSRLCU","cesarini");
			vTaskDelay(5000/portTICK_PERIOD_MS);// 5sec

			for (;;)
			{
				if (!WiFi.isConnected())
				{
					displayCurrentFrameIndex = DISPLAY_FRAME_WIFI;

					//xSemaphoreTake(sem_wifi, portMAX_DELAY);
					odomSystem.wifiStatus = wifiStatus_e::DISCONNECTED;
					//setup_wifi_smartconfig();

					// prova a connettersi con l'ultima configurazione
					WiFi.begin();

					vTaskDelay(xDelay_wifi);


					if (!WiFi.isConnected()) {
						//WiFi.disconnect(true);

						dbgD("RUN SMARTCONFIG");
						dbg("Please RUN SMARTCONFIG ON SMARTPHONE")
						//Init WiFi as Station, start SmartConfig
						WiFi.mode(WIFI_AP_STA); //WiFi.mode(WIFI_STA);
						WiFi.beginSmartConfig();
						odomSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;

						while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
							dbgD("RUN SMARTCONFIG");
							dbgD("ON SMARTPHONE");
							 
							vTaskDelay(xDelay_wifi);

						}

					}

					odomSystem.wifiStatus = wifiStatus_e::CONNECTED;

					// segnala a tsk_ros che la connessione c'è
					/* Set bit 8 in the notification value of the task referenced by xTask1Handle. */
					xTaskNotify(h_ros, (1UL << 8UL), eSetBits);
					//xSemaphoreGive(sem_wifi);

					vTaskDelay(xDelay_wifi);


				}
				else
				{
					vTaskDelay(10 * xDelay_wifi);
				}
				//displayFrame(-1);
			}



		}


	//---------------------------------------------------------------------------------------
	// task ROS
	//---------------------------------------------------------------------------------------
	#if 0

		//const TickType_t xDelay_ros = 500 / portTICK_PERIOD_MS;/* 10Hz */

		const TickType_t xDelay_ros = 1000 / TSK_ROS_HZ* portTICK_PERIOD_MS;
		TaskHandle_t h_ros;
		void tsk_ros(void * parameter) {

			dbg("\n[tsk_ros START...]");
			odomSystem.rosConnected = false;
			IPAddress rosIp;
			rosIp.fromString(odomSystem.ros_master_uri);
			bool blFirstTime = true;
			//ros_initialize(odomSystem.ros_master_uri);

			//attende la connessione wifi
			while (!odomSystem.wifiStatus == wifiStatus_e::CONNECTED) { vTaskDelay(10 * xDelay_ros); }



/*


			dbg2("ros_initialize()]Connecting to Ros core ... ", odomSystem.ros_master_uri);
			// Ros objects constructors   
			nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);

			nh.initNode();
			tfBr.init(nh);
			nh.spinOnce();

			while (!nh.connected()) {
				if (!tcpClient.connected())
				{ 
					nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);
					//tcpClient.connect(rosIp, ROS_TCP_CONNECTION_PORT);
					dbg("[tsk_ros setup] .");

				}
				else
				{

				}



				nh.spinOnce();
				vTaskDelay(xDelay_ros);
			}

			dbg("[tsk_ros setup] CONNECTED!!!");
			odomSystem.rosConnected = true;

*/
			dbg("\n[tsk_ros CONNECTING...]");
			for (;;)//task Loop phase
			{
				//odom_msg_update(encoderL.read(), encoderR.read(), true);
				if (odomSystem.wifiStatus == wifiStatus_e::CONNECTED)
				{

					if (nh.connected())
					{
						xSemaphoreTake(sem_odom_msg, portMAX_DELAY);

						publish_odom();						//pub_odom.publish(&msg_odom);
						dbg(" pb");
						xSemaphoreGive(sem_odom_msg);
						nh.spinOnce();

						vTaskDelay(xDelay_ros);

					}

					else// ROS non (più) connesso
					{
						odomSystem.rosConnected = false;
						displayCurrentFrameIndex = DISPLAY_FRAME_ROS;

						if (blFirstTime)
						{
							dbg("[tsk_ros loop] CONNECTING ROS");

						}
						else
						{
							dbg("[tsk_ros loop] LOST ROS");
						}


						// devo tentare di riconnettermi

						//"no more advertise admitted...");
						while (!nh.connected()) {
							ros_initialize(odomSystem.ros_master_uri);

							nh.spinOnce();
							dbg("[tsk_ros loop] \tconnecting...");
							dbgD("ROS connect...");
							vTaskDelay(5 * xDelay_ros);



						}
						blFirstTime = false;
						// qui ci deve arrivare solo quando si è connesso a ROS
						//LDSStatus = LDSStatus_e::ROSCONNECTED;
						odomSystem.rosConnected = true;

						nh.loginfo("[tsk_ros loop] ## ESP CONNESSO A ROS ##");
						

						//ROS_loadParameters();



						vTaskDelay(xDelay_ros);

					}

				}
				else // NON connesso a wifi, attende wifi
				{
					dbg("[tsk_ros loop] WiFi lost...");
					vTaskDelay(10 * xDelay_ros);
				}



			}


		}

	#endif // 1
	#if 1

		//const TickType_t xDelay_ros = 500 / portTICK_PERIOD_MS;/* 10Hz */

		const TickType_t xDelay_ros = 1000 / TSK_ROS_HZ* portTICK_PERIOD_MS;
		void tsk_ros(void * parameter) {
			uint32_t ulNotifiedValue;
			/*
			Non parte finchè non riceve l'ok dal WiFi

			Block indefinitely (without a timeout, so no need to check the function's
			return value) to wait for a notification.
			Bits in this RTOS task's notification value are set by the notifying
			tasks and interrupts to indicate which events have occurred. */
			//xTaskNotifyWait(0x00,						/* Don't clear any notification bits on entry. */
			//                ULONG_MAX,					/* Reset the notification value to 0 on exit. */
			//                &ulNotifiedValue,			/* Notified value pass out in  ulNotifiedValue. */
			//                portMAX_DELAY);				/* Block indefinitely. */

			//while (!odomSystem.wifiStatus == wifiStatus_e::CONNECTED) { vTaskDelay(10 * xDelay_ros); }

			//attende la connessione wifi
			while (!WiFi.isConnected()) { vTaskDelay(10 * xDelay_ros); }
			displayCurrentFrameIndex = DISPLAY_FRAME_ROS;





			dbg("\n[tsk_ros START...]");
			odomSystem.rosConnected = false;
			IPAddress rosIp;
			rosIp.fromString(odomSystem.ros_master_uri);
			bool blFirstTime = true;




			dbg("\n[tsk_ros CONNECTING...]");
			ros_initialize(odomSystem.ros_master_uri);
			for (;;)//task Loop phase
			{
				dbg("@#1")

				odom_msg_update(&msg_odom,encoderL.read(), encoderR.read(), true);
				dbg("@#2")

				publish_odometry();						//pub_odom.publish(&msg_odom);
				dbg("@#3")
				nh.spinOnce();
				dbg("@#4")

				vTaskDelay(xDelay_ros);

			}


		}

	#endif // 1
		//---------------------------------------------------------------------------------------



	//---------------------------------------------------------------------------------------
	// task MONITOR
	//---------------------------------------------------------------------------------------
	#if 1
		unsigned long tsk_monitor_lastloop = 0;
		float tsk_monitor_interval_ms = 1000.0 / 0.5; //Herz al denom.

		const TickType_t xDelay_monitor = 3000 / portTICK_PERIOD_MS;/* Block for 500ms. */
																	//Se sono passati i millis, manda su seriale i valori di odomSystem.encL.coun
		void tsk_monitor_loop() {
			if (millis() - tsk_monitor_lastloop > tsk_monitor_interval_ms) {
				dbg2("@ms: ", millis());
				//format the output for printing 
				char buf[50];
				sprintf(buf, "encoderL.read(): %d", encoderL.read()); 	Serial.println(buf);
				sprintf(buf, "encoderR.read(): %d", encoderR.read()); 	Serial.println(buf);
				sprintf(buf, "odomSystem.encL.count: %d", odomSystem.encL.count); 	Serial.println(buf);
				sprintf(buf, "odomSystem.encR.count: %d", odomSystem.encR.count); 	Serial.println(buf);

				//check the motion
				//Motion::motion m = encoderL.motion();
				//dbg2("\nEncR:", text(motionR));
				//dbg2("EncL:", text(motionL));

				tsk_monitor_lastloop = millis();
				yield();

			}
			else
			{
				yield();
			}

		}

		TaskHandle_t h_monitor;
		void tsk_monitor(void * parameter) {
			dbg("\n[tsk_monitor STARTED... ]");
			while (true) //task loop
			{
				tsk_monitor_loop();
				vTaskDelay(xDelay_monitor);
			}
		}

	#endif // 1

	//---------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------
	// task DISPLAY
	//---------------------------------------------------------------------------------------
	//unsigned long tsk_display_lastloop = 0;
	//float tsk_display_interval_ms = 1000.0 / 0.5; //Herz al denom.
	//Se sono passati i millis, manda su seriale i valori di odomSystem.encL.coun
	#if 1
		const TickType_t xDelay_display = 500 / portTICK_PERIOD_MS;/* Block for 500ms. */
			void tsk_display(void * parameter) {
				dbg("\n[tsk_display STARTED... ]");
				while (true) //task loop
				{

					displayFrame(-1);
					vTaskDelay(xDelay_display);

				}

			}

	#endif // 1
	//---------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------
	// task ODOM
	//---------------------------------------------------------------------------------------
	TaskHandle_t h_odom;
	void tsk_odom(void * pvParameters)
	{
		dbg("\n[tsk_odom STARTED... ]");

		const TickType_t xDelay_odom = 50 / portTICK_PERIOD_MS;/* Block for 20Hz. */

		for (;; )
		{
			/* Simply toggle the LED every 500ms, blocking between each toggle. */
			//TOGGLE_LED;
			odom_msg_update(&msg_odom,encoderL.read(), encoderR.read(), true);
			vTaskDelay(xDelay_odom);
		}
	}
	//---------------------------------------------------------------------------------------


		// lancia i vari task paralleli
		static int Core0 = 0;
		static int Core1 = 1;


		void setup_tasks() {
			//  http://esp32.info/docs/esp_idf/html/dd/d3c/group__xTaskCreate.html

			//			xTaskCreate(	task,   name of task, Stack size , Parameter , Priority , Task handle)
			//xTaskCreatePinnedToCore(	task,   name of task, Stack size , Parameter , Priority , Task handle, core)
//			xTaskCreatePinnedToCore(tsk_odom, "odom", 3000, NULL, 3, &h_odom, 1);
			xTaskCreatePinnedToCore(tsk_wifi,		"wifi",		3000,	NULL, 3, &h_wifi,		Core1);
			xTaskCreatePinnedToCore(tsk_ros,		"ros",		3000,	NULL, 3, &h_ros,		Core1);
			xTaskCreatePinnedToCore(tsk_display,	"display",	3000,	NULL, 3, &h_display,	Core1);
//			xTaskCreatePinnedToCore(tsk_monitor,	"monitor",	3000,	NULL, 3, &h_monitor,	Core1);


			LED_OFF;
			dbg("\n\n  setup_tasks()] ---all task created ---");

		}












#pragma endregion


// ///////////////////////////////////////////////////////////////////////////////
///
//		LAYER DI INTEGRAZIONE FRA I VARI COMPONENTI
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region IntegrationLayer
	//imposta I2C,Display,Interrupt, direzione GPIO

	unsigned long nextMsg_time = 0;
	long publishTime = 0;		/// tempo impiegato per pubblicare
	long publishStartTime = 0;  ///inizio pubblicazione
	long loopcnt = 0;
	#define DISPLAY_SCAN_MAX_RAY 20 /*dimensione massima del cerchio che rappresenta lo scan alla massima distanza*/



	void init_globalVars() {
		odomSystem.encL.count = 0;
		odomSystem.encR.count = 0;
		odomSystem.wifiStatus = wifiStatus_e::DISCONNECTED;
		odomSystem.odom_x = 0.0;
		odomSystem.odom_y = 0.0;
		odomSystem.odom_r = 0.0;

		odomSystem.ros_master_uri = ROS_MASTER_URI_DEFAULT;
		odomSystem.ros_tcp_port = ROS_TCP_CONNECTION_PORT;
		odomSystem.rosConnected = false;
		odomSystem.ros_topic = "/odom";
		odomSystem.frame_id = "base_link";



		odomSystem.g_steps2m = ROBOT_STEPS2M;
		odomSystem.g_steps2rads = ROBOT_STEPS2RAD;
		odomSystem.g_enc2steps = ROBOT_MOTOR_STEPS_PER_ENCODERTICK;

	}
	// inizializza: I2C, encoder ,pulsanti
	void setup_HW() {





		// ///////////////////////////////////////////////////////////////////////////////
		///
		//		HW DI BASE
		///
		// ///////////////////////////////////////////////////////////////////////////////

		Wire.begin(PIN_SDA, PIN_CK);
		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


		printESPinfo();

		// LED
		pinMode(PIN_LED, OUTPUT);




//		pinMode(PIN_FLASHBUTTON, INPUT_PULLUP);
		pinMode(INTERRUPT_PIN, INPUT_PULLUP);

		//Commuta il frame del display visualizzato
		attachInterrupt(INTERRUPT_PIN, isr_displayChangeWindow, RISING);

		interrupts();  // abilita gli interrupt

		//pinMode(D4, FUNCTION_4);
		// comandi che possono tornare utili
		///uart_set_debug(UART0);
		///pinMode(uart->tx_pin, FUNCTION_4);



		// ///////////////////////////////////////////////////////////////////////////////
		///
		//		PERIFERICHE 
		///
		// ///////////////////////////////////////////////////////////////////////////////


		/*ENCODERS
		pinMode(Pin_EncRa, INPUT);	///encoder Right Motor 
		pinMode(Pin_EncRb, INPUT);	///encoder Left Motor
		pinMode(Pin_EncLa, INPUT);	///encoder Right Motor 
		pinMode(Pin_EncLb, INPUT);	///encoder Left Motor
									
		attachInterrupt(Pin_EncRa, isrEncR, RISING);
		attachInterrupt(Pin_EncLa, isrEncL, RISING);
		interrupts();  // abilita gli interrupt
		*/
		setup_encoder();

		dbg("\nEnd setup HW... ");


	}

	#pragma region setup_wifi_smartconfig
		#include "WiFi.h"
		void setup_wifi_smartconfig() {
			odomSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;

			// prova a connettersi con l'ultima configurazione
			WiFi.begin(); 
			dbgD("Connecting WiFi....");
			delay(5000);

			if (!WiFi.isConnected()){

				//Init WiFi as Station, start SmartConfig
				WiFi.mode(WIFI_AP_STA); //WiFi.mode(WIFI_STA);
				WiFi.beginSmartConfig();
				Serial.println("Waiting for SmartConfig.");
				while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
					delay(5000);
					//Serial.print(".");
					dbgD("Pls use Smartconfig");
				}

			}

			odomSystem.wifiStatus = wifiStatus_e::CONNECTED;

			dbgD("WiFi Connected.");
			dbgD("--IP Address-- ");
		 
			dbgD(WiFi.localIP().toString());
			dbgD("---.---.-.--- ");


			Serial.print("Wifi Connected. \nIP Address: ");
			Serial.println(WiFi.localIP());
		}

		// come smartconfig ma senza dbg o dbgD
		void setup_wifi_silent() {


			//WiFi.begin(); 
			delay(5000);

			if (!WiFi.isConnected()){

				//Init WiFi as Station, start SmartConfig
				odomSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;
				WiFi.mode(WIFI_AP_STA); //WiFi.mode(WIFI_STA);
				WiFi.beginSmartConfig();
				while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
					delay(5000);
				}

			}

			odomSystem.wifiStatus = wifiStatus_e::CONNECTED;

		}
		//classic way
		void setup_wifi(char* ssid, char* pwd) {

			displayCurrentFrameIndex = 3;
			WiFi.begin(ssid,pwd); 
			Serial.printf("\nsetup_wifi] Connecting to SSID %s  ...", ssid);
			delay(5000);
			odomSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;
			while (WiFi.status() != WL_CONNECTED) {
				delay(2000);
				Serial.print(".");
				displayFrame(-1);
			}

			Serial.println("\nsetup_wifi] Connected to the WiFi network");
			dbgD("WIFI Connected  ");

			//if (!WiFi.isConnected()){

			//	//Init WiFi as Station, start SmartConfig
			//	odomSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;
			//	WiFi.mode(WIFI_AP_STA); //WiFi.mode(WIFI_STA);
			//	WiFi.beginSmartConfig();
			//	while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
			//		delay(5000);
			//	}

			//}

			odomSystem.wifiStatus = wifiStatus_e::CONNECTED;

		}

	#pragma endregion


#pragma endregion  //IntegrationLayer


// ///////////////////////////////////////////////////////////////////////////////
///
//		S E T U P
///
// ///////////////////////////////////////////////////////////////////////////////

void setup() {
	bool blSkipRosConnection = true;
	if (GETBUTTON == 1) /*se tengo premuto inizialmente salto il setup wifi*/
	{
		blSkipRosConnection = true;
	}

	#if OPT_USE_SERIAL_DBG 
		SERIAL_DBG.begin(SERIAL_SPEED);
		SERIAL_DBG.setDebugOutput(true);// to enable output from printf() function.  >>http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#timing-and-delays
		dbg("\n######## ### Esp32RosOdom v0.1 #####\n\n\n\n\n");
	#endif

		init_globalVars(); //integration layer
	//---------------------------------------------------------------------------------------
	// setup HW
	dbg("\nsetup HW.. ");
	setup_HW();	//imposta I2C,Display,Interrupt, direzione GPIO


	//---------------------------------------------------------------------------------------
	// setup DISPLAY
	dbg("\nsetup_display..\n");
	displayCurrentFrameIndex = DISPLAY_FRAME_MESSAGGI; //display msgs
	setup_display();


	//---------------------------------------------------------------------------------------
	//// setup WIFI
	//---------------------------------------------------------------------------------------
	//dbg("\nsetup WiFi... ");
	//WiFi.disconnect(true);
	//dbgD("setup WiFi... ");
	//// prova a connettersi con l'ultima configurazione
	//dbg("\n>>>>>>>>>>>");

//	setup_wifi_silent();

//    setup_wifi("FASTWEB-CSRLCU", "cesarini");
	dbg("\n<<<<<<<<<<<..\n");

	//---------------------------------------------------------------------------------------
	// setup WEBSERVER
	//---------------------------------------------------------------------------------------
	//dbgD("setup webserver.. ");
	//dbg("\nsetup webserver.. ");
	//setup_webserver();	//  webserver per permettere di impostare i parametri WIFI e ROS 


	displayCurrentFrameIndex = DISPLAY_FRAME_WIFI; // DISPLAY_FRAME_ENCODER; //display san

	//fine setup  inizia loop
	LED_OFF;
	dbg("\n\n --[STARTING LOOP]--");
	dbgD("--[STARTING LOOP]--");
	setup_tasks();


}



void loop() { 
	nh.spinOnce();
}






