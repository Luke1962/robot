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
#define ESP32


#include <Esp32Hardware.h>
#include <ArduinoHardware.h>
//#define ESP32 1

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

/////////////////////////////////////////////////////////////////////
// FILE SYSTEM
/////////////////////////////////////////////////////////////////////
#include <vfs_api.h>
#include <FSImpl.h>
#include <FS.h>



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
#include "parameters.h"


// ///////////////////////////////////////////////////////////////////////////////
///
//		NODEMCU HW
///
// ///////////////////////////////////////////////////////////////////////////////
#if 1
	#pragma region NODEMCU_HW


	void isr_displayChangeWindow(); // --> sezione OLED Display




	#pragma region ESP32_PIN
	#define Pin_Default_EncRa 36
	#define Pin_Default_EncRb 25
	#define Pin_Default_EncLa 12
	#define Pin_Default_EncLb 13

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
	struct encoder_t {
		int32_t count;  // contatore cumulativo encoder 
		int deltaCount; // tick tra due misure
		bool interruptFlag;
		long last_change;
	};

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

		// HW encoders
		int Pin_EncLa;
		int Pin_EncLb;
		int Pin_EncRa;
		int Pin_EncRb;



		//ROS generic
		String ros_master_uri;
		int rosCoreAddress;
		bool rosConnected;
		int rosRate;

		// ROS Node specific
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
		int32_t raspicam_tilt_deg;

	} __attribute__((aligned(4)))
	thisSystem;

	///////////////////////////////////////////////
	// Variabili globali del sistema di Odometria
	////////////////////////////////////////////////


	void EEPROM_load() {
		EEPROM_readAnything(0, thisSystem);
		dbg("\nConfig Loaded.");
		if (thisSystem.rosCoreAddress == 0)
		{
			thisSystem.rosCoreAddress = 100;
		}
		dbg2("rosCoreAddress:", thisSystem.rosCoreAddress);



		if ((thisSystem.Pin_EncLa == 0) && (thisSystem.Pin_EncLb == 0))
		{
			//inizializzo
			thisSystem.Pin_EncRa = Pin_Default_EncRa;	//sostituisce  #define Pin_EncRb 12
			thisSystem.Pin_EncRb = Pin_Default_EncRb;	//sostituisce  #define Pin_EncRa 25
			thisSystem.Pin_EncLa = Pin_Default_EncLa;	//sostituisce #define Pin_EncRa 36
			thisSystem.Pin_EncLb = Pin_Default_EncLb;	//sostituisce  #define Pin_EncLb 13
		}
		



	}

	void EEPROM_save() {
		EEPROM_writeAnything(0, thisSystem);
		dbg("\nConfig Saved.");
	}




#pragma endregion



// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
// ENCODERS 
/// opera solo se  ogni ISR_MINMUM_INTERVAL_MSEC
// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1


	#define	ENCODER_USE_INTERRUPTS
	#include <Encoder-master\Encoder.h>


	// Change these two numbers to the pins connected to your encoder.
	//   Best Performance: both pins have interrupt capability
	//   Good Performance: only the first pin has interrupt capability
	//   Low Performance:  neither pin has interrupt capability
	Encoder encoderL(thisSystem.Pin_EncLa, thisSystem.Pin_EncLb);
	Encoder encoderR(thisSystem.Pin_EncRa, thisSystem.Pin_EncRb);
	//   avoid using pins with LEDs attached


	void setup_encoder() {
		// void, ma tenerlo 
	}







#endif // 1




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
	#define ROS_MASTER_URI_DEFAULT	"192.168.0.100"
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






#pragma endregion


// ///////////////////////////////////////////////////////////////////////////////
///
//		W E B  S E R V E R
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region webServer
	#define OPT_WEBSERVER 1
	#if OPT_WEBSERVER


	#include <WiFiClient.h>
	#include <ESP32WebServer.h>
	#include <WiFi.h>
	#include <ESPmDNS.h>


	ESP32WebServer webServer(80);

	String Argument_Name, Clients_Response1, Clients_Response2;



	// gestione parametri ros
	void wsrv_handle_root()
	{
		String IPaddress = WiFi.localIP().toString();
		String webpage;


		// POST ?
		if (webServer.args() > 0) { // Arguments were received
			for (uint8_t i = 0; i < webServer.args(); i++) {
				Serial.print(webServer.argName(i)); // Display the argument
				Argument_Name = webServer.argName(i);


				if (webServer.argName(i) == "Pin_EncLa") {
					Serial.print(" Input for [Pin_EncLa]  received was: ");	Serial.println(webServer.arg(i));
					thisSystem.Pin_EncLa = webServer.arg(i).toInt();
					webpage += "<BR>Pin_EncRb_[" + String(thisSystem.Pin_EncLa) + "]";
				}

				if (webServer.argName(i) == "Pin_EncLb") {
					Serial.print(" Input for [Pin_EncLb]  received was: ");	Serial.println(webServer.arg(i));
					thisSystem.Pin_EncLb = webServer.arg(i).toInt();
					webpage += "<BR>Pin_EncRb_[" + String(thisSystem.Pin_EncLb) + "]";
				}
				if (webServer.argName(i) == "Pin_EncRa") {
					Serial.print(" Input for [Pin_EncRa]  received was: ");	Serial.println(webServer.arg(i));
					thisSystem.Pin_EncRa = webServer.arg(i).toInt();
					webpage += "<BR>Pin_EncRb_[" + String(thisSystem.Pin_EncRa) + "]";
				}

				if (webServer.argName(i) == "Pin_EncRb") {
					Serial.print(" Input for [Pin_EncRb]  received was: ");	Serial.println(webServer.arg(i));
					thisSystem.Pin_EncRb = webServer.arg(i).toInt();
					webpage += "<BR>Pin_EncRb_[" + String(thisSystem.Pin_EncRb) + "]";
				}




				if (webServer.argName(i) == "g_steps2m") {
					Serial.print(" Input for [g_steps2m]  received was: ");	Serial.println(webServer.arg(i));
					//Clients_Response1 = webServer.arg(i);
					thisSystem.g_steps2m = webServer.arg(i).toFloat();
					// e.g. range_maximum = webServer.arg(i).toInt();   // use string.toInt()   if you wanted to convert the input to an integer number
					// e.g. range_maximum = webServer.arg(i).toFloat(); // use string.toFloat() if you wanted to convert the input to a floating point number
				}

				if (webServer.argName(i) == "g_steps2rads") {
					Serial.print(" Input for [g_steps2rads] was: "); Serial.println(webServer.arg(i));
					thisSystem.g_steps2rads = webServer.arg(i).toFloat();
				}

				if (webServer.argName(i) == "rosRate") {
					Serial.print(" Input for [rosRate]  received was: ");	Serial.println(webServer.arg(i));
					thisSystem.rosRate = webServer.arg(i).toInt();
					webpage += "<BR>rosRate[" + String(thisSystem.rosRate) + "]";
				}
				if (webServer.argName(i) == "rosCoreAddress") {
					Serial.print(" Input for [rosCoreAddress]  received was: ");	Serial.println(webServer.arg(i));
					thisSystem.rosCoreAddress = webServer.arg(i).toInt();
					webpage += "<BR>rosCoreAddress[" + String(thisSystem.rosCoreAddress) + "]";

				}
			}
			webpage += "<h1>PARAMETRI SALVATI</h1>";

			EEPROM_save();

		}



		webpage = "<html>";
		webpage +=	"<head><title>ROS parameters</title>";
		webpage +=		"<style>";
		webpage +=			"body { background-color: #E0E0FA; font-family: Arial, Helvetica, Sans-Serif; Color: blue;}";
		webpage +=		"</style>";
		webpage +=	"</head>";
		webpage +=	"<body>";

		webpage +=		"<h1><br>ESP32 ROS Odom v1.0</h1>";
		webpage +=		"<a href = '/'>Reload</a> Time ms: " +String(millis());
		webpage +=		"<form action='http://" + IPaddress + "/' method='POST'>";


		webpage += "<h1><br>HW Parameters</h1>";//----------------------------------------------------------------------------
		webpage += "<p>ENCODER LEFT  A Pin: :<input  width='10' type='number' value = '" + String(thisSystem.Pin_EncLa) + "' name='Pin_EncLa'><BR>";
		webpage += "<p>ENCODER LEFT  B Pin: :<input  width='10' type='number' value = '" + String(thisSystem.Pin_EncLb) + "' name='Pin_EncLb'><BR>";
		webpage += "<p>ENCODER RIGHT A Pin: :<input  width='10' type='number' value = '" + String(thisSystem.Pin_EncRa) + "' name='Pin_EncRa'><BR>";
		webpage += "<p>ENCODER RIGHT B Pin: :<input  width='10' type='number' value = '" + String(thisSystem.Pin_EncRb) + "' name='Pin_EncRb'><BR>";

		webpage += "<p>rosCore Ip Address 192.168.0.<input  width='30' type='number' value = '" + String(thisSystem.rosCoreAddress) + "' name='rosCoreAddress'><BR>";
		webpage += "<p>node rate <input  width='30' type='number' value = '" + String(thisSystem.rosRate) + "' name='rosRate'><BR>";

		// fine form
		webpage +=		"<input  type='submit' value='Save and restart'>";
		webpage += "</form>";

		webpage += "</body>";
		webpage += "</html>";
		webServer.send(200, "text/html", webpage); // Send a response to the client asking for input


	}


	void wsrv_handleNotFound() {

		String message = "File Not Found\n\n";
		message += "URI: ";
		message += webServer.uri();
		message += "\nMethod: ";
		message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
		message += "\nArguments: ";
		message += webServer.args();
		message += "\n";
		for (uint8_t i = 0; i < webServer.args(); i++) {
			message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
		}
		webServer.send(404, "text/plain", message);

	}



	void webServerStart() {
		webServer.begin();
		dbg("WEB SERVER BEGIN!!");
	}
	void webServerStop() {
		webServer.stop();
		dbg("WEB SERVER STOP!!");
	}

	//----------------------------------
	void setup_webserver()
	{

		if (MDNS.begin("esp32")) {
			Serial.println("M DNS responder started");
		}

		webServer.on("/",		wsrv_handle_root);

		webServer.onNotFound(wsrv_handleNotFound);

		webServer.begin();
		Serial.println("HTTP server started");
	}


#endif // OPT_WEBSERVER

#pragma endregion


// ///////////////////////////////////////////////////////////////////////////////
///
//		OLED DISPLAY  128x64 
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region OledDisplay 
	//https://github.com/ThingPulse/esp8266-oled-ssd1306
	#define DISPLAY_FRAMES 8	// Numero di schede 0: messaggi, 1: compass, 2: quaternions, 3:config
	#define DISPLAY_SIZE_X 128
	#define	DISPLAY_SIZE_Y 64

	//int displayCurrentFrameIndex = 0; // scheda corrente, incrementata dal pulsante flash 
	static volatile uint8_t displayCurrentFrameIndex = 0; 

		// this variable will be changed in the ISR, and Read in main loop
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
	CircularBuffer<String, 5> displayBufferMsgs;
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


	void  displayPanel(int frameNo );
	#define DISPLAY_BUFF_ROWS 4 /*righe che il dislay è in grado di visualizzare*/
	#define DISPLAY_INTERLINEA 11
	#define DISPLAY_HEADER_SIZE_Y 13
	#define DISPLAY_ROW(r) DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (r-1)
	#define dbgD(s) displayBufferMsgs.push(s);  displayPanel(-1);  //display.display();//displayCircularBuffer(false,false);
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
	void displayCompass(float rad, bool blDrawCircle = true) {
		// offset da impostare per allineare la barra sul display all'asse x del magnetometro
		// positivo in senso orario
		#define OFFSET PI/2

		#define CENTER_X 80
		#define CENTER_Y 38
		#define RADIUS 22
		if (blDrawCircle)
		{
			display.drawCircle(CENTER_X, CENTER_Y, RADIUS);

		}
		int dx = round(RADIUS* cos(-rad + OFFSET));
		int dy = round(RADIUS* sin(-rad + OFFSET));
		display.drawLine(CENTER_X, CENTER_Y, CENTER_X + dx, CENTER_Y + dy);
	}

	void setup_display(String title) {
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
		dbgD("# "+title+" #");
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
	#define DISPLAY_FRAME_RASPICAM	5
	#define DISPLAY_FRAME_MEMORY	6
	#define DISPLAY_WEBSERVER		7

	void  displayPanel(int frameNo = -1) {

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
			s = "Enc L: " + String((int)thisSystem.encL.count);
			display.drawString(0, DISPLAY_ROW(1), s);
			
			s = "Pin Enc L a: " + String(thisSystem.Pin_EncLa) + ", b " + String(thisSystem.Pin_EncLb);
			display.drawString(0, DISPLAY_ROW(2), s);



			s = "Enc R: " + String((int)thisSystem.encR.count);
			//s = "Enc R: " + String((int)encoderR.read());
			display.drawString(0, DISPLAY_ROW(3), s);

			s = "Enc R Pin a: " + String(thisSystem.Pin_EncRa) + ", b " + String(thisSystem.Pin_EncRb);
			display.drawString(0, DISPLAY_ROW(4), s);


			display.display();
			break;			


		case DISPLAY_FRAME_WIFI:
			//WIFI

			display.clear();
			displayFrameHeader("[WiFi Config]");
			//if (thisSystem.wifiStatus == wifiStatus_e::CONNECTED)
			if (WiFi.isConnected())
			{
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

			}
			else if (thisSystem.wifiStatus == wifiStatus_e::SMARTCONFIG)
			{

				s = "USE SMARTCONFIG!!" ;
				display.drawString(0, DISPLAY_ROW(1), s);
			}
			else //wifiStatus_e::DISCONNECTED
			{
				s ="Wifi NOT Connected";
				display.drawString(0, DISPLAY_ROW(1), s);

			}

	 


			//s = "MQTT:" + String(mqtt_server);
			//display.drawString(0, DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (2), s);

			display.display();
			break;
		case DISPLAY_FRAME_ROS:
			// ROS
			displayFrameHeader("[ROS]");

			s =thisSystem.rosConnected ? "[ROS CONNECTED]" :"[ROS NOT CONNECTED]";
			display.drawString(0, DISPLAY_ROW(1), s);
			s = "ROS URI:" + thisSystem.ros_master_uri;
			display.drawString(0, DISPLAY_ROW(2), s);
			s = "TCP PORT: " + String(thisSystem.ros_tcp_port);
			display.drawString(0, DISPLAY_ROW(3), s);



			display.display();
			break;

		case DISPLAY_FRAME_ODOM:
			// ROS
			displayFrameHeader("[ROS odometry]");

			s = "topic: " + thisSystem.ros_topic;
			display.drawString(0, DISPLAY_ROW(1), s);

			//s = "frame id: " + String(thisSystem.frame_id);
			//display.drawString(0, DISPLAY_ROW(2), s);


			s = "x: " +String(thisSystem.odom_x,3);
			display.drawString(0, DISPLAY_ROW(2), s);
			s = "y: " + String(thisSystem.odom_y, 3);
			display.drawString(0, DISPLAY_ROW(3), s);
			s = "r: " + String(thisSystem.odom_r, 3);
			display.drawString(0, DISPLAY_ROW(4), s);

			displayCompass(thisSystem.odom_r);


			display.display();
			break;

		case DISPLAY_FRAME_RASPICAM:
			// SERVO
			displayFrameHeader("[ROS Servo]");

			s = "sub: /raspicam_tilt_deg";
			display.drawString(0, DISPLAY_ROW(1), s);

			s = "Tilt deg: " + String(thisSystem.raspicam_tilt_deg);
			display.drawString(0, DISPLAY_ROW(4), s);



			displayCompass((90+thisSystem.raspicam_tilt_deg) * DEG_TO_RAD, false);
			display.drawCircle(CENTER_X, CENTER_Y, 2);

			display.display();
			break;
		case DISPLAY_FRAME_MEMORY:
			displayFrameHeader("[MEMORY]");
			//String((unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT));
			
			s = "Free mem: " +String(xPortGetFreeHeapSize());
			display.drawString(0, DISPLAY_ROW(1), s);

			display.display();
			break;


		case DISPLAY_WEBSERVER:
			displayFrameHeader("[WEBSERVER]");

			#if  OPT_WEBSERVER
				s = "Web server at";
				display.drawString(0, DISPLAY_ROW(1), s);
				s =	"http://"+ WiFi.localIP().toString() +"/";
				display.drawString(0, DISPLAY_ROW(2), s);

				webServer.handleClient();
			#else
				s = "NO Web server";
				display.drawString(0, DISPLAY_ROW(1), s);

			#endif //  OPT_WEBSERVER
			display.display();
			break;

		default:
			displayFrameHeader("[ROS Servo]");

			s = "## WRONG PANEL ##";
			display.drawString(0, DISPLAY_ROW(2), s);
			display.display();
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


	String ftoa(float number) {
	// dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
	
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
//		SERVO DRIVER
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region SERVO


	/***************************************************
	This is an example for our Adafruit 16-channel PWM & Servo driver
	http://www.adafruit.com/products/815
	****************************************************/
	//#include <Wire.h>
	#include <Adafruit_PWMServoDriver.h>

	// called this way, it uses the default address 0x40
	Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
	// you can also call it with a different address you want
	//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
	// you can also call it with a different address and I2C interface
	//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);



	// Depending on your servo make, the pulse width min and max may vary, you 
	// want these to be as small/large as possible without hitting the hard stop
	// for max range. You'll have to tweak them as necessary to match the servos you
	// have!
	//questi valori li ho trovati sperimentalmente
	#define SERVOMIN  150 //was150		 this is the 'minimum' pulse length count (out of 4096)
	#define SERVOMAX  580 //was 600		 this is the 'maximum' pulse length count (out of 4096)
	const int servonum = 0;//canale pwm da 0 a 16

	void setup_servo() {
		Serial.begin(115200);
		Serial.println("16 channel PWM test!");

		pwm.begin();


		pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

		delay(10);
		// if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
		// some i2c devices dont like this so much so if you're sharing the bus, watch
		// out for this!
		Wire.setClock(400000);
	}


	// you can use this function if you'd like to set the pulse length in seconds
	// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
	void setServoPulse(uint8_t n, double pulse) {
		double pulselength;

		pulselength = 1000000;   // 1,000,000 us per second
		pulselength /= 60;   // 60 Hz
		Serial.print(pulselength); Serial.println(" us per period");
		pulselength /= 4096;  // 12 bits of resolution
		Serial.print(pulselength); Serial.println(" us per bit");
		pulse *= 1000000;  // convert to us
		pulse /= pulselength;
		Serial.println(pulse);
		pwm.setPWM(n, 0, pulse);
	}


	// AngleDeg=0 >> tilt orizzontale; >0 verso l'alto, ; <0 tilt verso il basso
	void servoAngle(int angleDeg) {
		int x =map(angleDeg, -45, 90, SERVOMIN, SERVOMAX);
		if (x>0)
		{

			if (x > SERVOMAX) {
				String s = "! " + String(x) + "> Max Value " + String(SERVOMAX);
				dbgD(s)
					dbg(s);
			}
			else if (x < SERVOMIN) {
				String s ="! "+ String(x) + "< Min Value " + String(SERVOMIN);
				dbgD(s)
					dbg(s);
			}
			else
			{
				if (x < SERVOMIN) { x = SERVOMIN; };
				if (x > SERVOMAX) { x = SERVOMAX; };
				pwm.setPWM(servonum, 0, x);
				String s = "Servo " + String(servonum) + " : " + String(x) + "   @ "+ String(millis());
				//dbgD(s);
				dbg(s);

			}
		}	//if (x>0)

	}

	void servo_sweep() {
		// Drive each servo one at a time
		for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
			pwm.setPWM(servonum, 0, pulselen);
			String s = "Servo " + String(servonum) + " : " + String(pulselen) + "    ";
			dbgD(s);
			//display.drawString(0, 10, s); display.display();
			#ifdef ESP8266 
				yield();  // take a breather, required for ESP8266
			#endif
			delay(15); 		  //delay(dt);
		}

		delay(500);
		for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
			pwm.setPWM(servonum, 0, pulselen);
			String s = "Servo " + String(servonum) + " : " + String(pulselen) + "    ";
			//display.drawString(0, 10, s); display.display();

			dbgD(s);
			#ifdef ESP8266 
				yield();  // take a breather, required for ESP8266
			#endif
				delay(30); 		  //delay(dt);
		}
		delay(1500);

	}
#pragma endregion





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
	#include <std_msgs/Int32.h>
	#include <std_msgs/Int16.h>
	#include <std_msgs/Uint16.h>



	//--------------------------------
	// TF 
	//--------------------------------
	#include <tf/tf.h>
	#include <tf/transform_broadcaster.h>
	tf::TransformBroadcaster tfBr;
	geometry_msgs::TransformStamped t;

	WiFiClient tcpClient;




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
		#define ROSRATE(Hz)  1000 / Hz* portTICK_PERIOD_MS




	/// ////////////////////////////////////////////////////////////////////////////////////////////////////
	/// R O S
	/// C H A T T E R   S U B S C R I B E R
	/// /////////////////////////////////////////////////////////////////////////////////////////////////////	
#if 0
	void chatterCallback(const std_msgs::String& msg) {
		dbgD(msg.data);
		char  ros_info_msg[100];
		sprintf(ros_info_msg, "OK: [/message] = %d", msg.data); 	ROS_INFO(ros_info_msg);
	}
	ros::Subscriber<std_msgs::String> sub_chatter("/message", &chatterCallback);

#endif // 0



	/*
		AngleDEg =0  => orizzontale
	*/


	/// ////////////////////////////////////////////////////////////////////////////////////////////////////
	/// R O S
	/// S E R V O   S U B S C R I B E R
	/// /////////////////////////////////////////////////////////////////////////////////////////////////////	

#if 0
	void rosCbk_raspicam_tilt_deg(const std_msgs::Int32& cmd_msg) {
		//servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
		//digitalWrite(13, HIGH - digitalRead(13));  //toggle led  
		thisSystem.raspicam_tilt_deg = cmd_msg.data;
		servoAngle(thisSystem.raspicam_tilt_deg);
		char  ros_info_msg[100];
		sprintf(ros_info_msg, "OK: tilt: %d°", cmd_msg.data); 	ROS_INFO(ros_info_msg);
		dbgD(String(ros_info_msg));

	}
	ros::Subscriber<std_msgs::Int32> sub_raspicam_tilt_deg("/raspicam_tilt_deg", &rosCbk_raspicam_tilt_deg);


#endif // 0





	/// ////////////////////////////////////////////////////////////////////////////////////////////////////
	/// R O S
	/// O D O ME T R Y
	/// ////////////////////////////////////////////////////////////////	
	#pragma region ROS_ODOMETRY
		#include <nav_msgs/Odometry.h>
		nav_msgs::Odometry msg_odom;
		std_msgs::Int32 msg_encoderCountL;
		std_msgs::Int32 msg_encoderCountR;


		//ros::Publisher pub_odom("/odom", &msg_odom);

		#include <std_msgs/Int32.h>
		#include <geometry_msgs/PoseWithCovarianceStamped.h>		// per InitialPose

		// publisher -----------------------------
		ros::Publisher pub_odom("/odom", &msg_odom);
		ros::Publisher pub_encL("/encoder_left", &msg_encoderCountL);
		ros::Publisher pub_encR("/encoder_right", &msg_encoderCountR);




		void rosCbk_initialPose(const geometry_msgs::PoseWithCovarianceStamped& msg_initialPose) {
			dbg("\nROS] Ricevuto InitialPose");
		}
		ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_initialPose("/initialpose", &rosCbk_initialPose);





		#include "parameters.h"

		#define INTERVAL_ODOM_UPDATE_MSEC 100  /// ogni quanto aggiorno l'odom'. limitato da rosRate
		long g_last_robotOdom_update_time = 0;

		//Aggiorna thisSystem.odom_x ,y,z e thisSystem.vel_x ,y,z sulla base del valore incrementale di conteggio degli encoder

		//versione senza attesa per multitasking
		void odom_msg_update(nav_msgs::Odometry* msg_odom,  int32_t encL, int32_t encR, bool blCountIsAbsolute=true) {
			//Aggiorno i contatori totali
			if (blCountIsAbsolute)
			{
				thisSystem.encL.deltaCount = encL - thisSystem.encL.count;
				thisSystem.encR.deltaCount = encR - thisSystem.encR.count;
				thisSystem.encL.count = encL;
				thisSystem.encR.count = encR;

			}
			else
			{
				thisSystem.encL.deltaCount = encL ;
				thisSystem.encR.deltaCount = encR ;
				thisSystem.encL.count += encL;
				thisSystem.encR.count += encR;

			}
			

			if (abs(thisSystem.encL.deltaCount) + abs(thisSystem.encR.deltaCount) > 0) ///> 0 se si è mosso
			{

			//dbg("#")

				/// I delta sono calcolati rispetto a /base_link
				//				double deltaStepsForward = (float)((encL.count+encR.count)/2)*(float)ROBOT_MOTOR_STEPS_PER_ENCODERTICK ; ///somma 0 se ruota				
				//				double deltaStepsRotationCCW = 0.5*(encRcount-encLcount)*(float )ROBOT_MOTOR_STEPS_PER_ENCODERTICK ;
				double deltaStepsForward = (float)((thisSystem.encL.deltaCount + thisSystem.encR.deltaCount) / 2)*(float)thisSystem.g_enc2steps; ///somma 0 se ruota				
				double deltaStepsRotationCCW = 0.5*(thisSystem.encR.deltaCount - thisSystem.encL.deltaCount)*(float)thisSystem.g_enc2steps;
			//dbg("@1")

				//				double deltaForward_m = deltaStepsForward *(float)ROBOT_STEPS2M; 
				//				double deltaTheta_rad = (float )deltaStepsRotationCCW* ROBOT_STEPS2RAD /2;/// converto in angolo 
				double deltaForward_m = deltaStepsForward *(float)thisSystem.g_steps2m;
				double deltaTheta_rad = (float)deltaStepsRotationCCW* thisSystem.g_steps2rads;/// converto in angolo 

			//dbg("@2")


				// delta spostamento in metri rispetto al frame /odom
				double deltaOdom_x = deltaForward_m * cos(thisSystem.odom_r);
				double deltaOdom_y = deltaForward_m * sin(thisSystem.odom_r);

			//dbg("@3")
					
				///calculate current position of the robot vs /odom frame
				thisSystem.odom_x += deltaOdom_x;
				thisSystem.odom_y += deltaOdom_y;
				thisSystem.odom_r += deltaTheta_rad;
				if (thisSystem.odom_r > 2*PI)
				{
					thisSystem.odom_r -=2 * PI;
				}
					
			//dbg("@4")


				/// compute velocity
				//ros::Time now = ros::Time::now();
				//double dt = (now - g_last_robotOdom_update_time).toSec();
				long now = millis();
				long dt = now -g_last_robotOdom_update_time;
				g_last_robotOdom_update_time = now;

				float dtSec = dt / 1000;
				thisSystem.vel_x = deltaOdom_x / dtSec;
				thisSystem.vel_y = deltaOdom_y / dtSec;
				thisSystem.twist_z = deltaTheta_rad / dtSec;
			//dbg("@5")
				msg_odom->header.frame_id = "odom";
				msg_odom->pose.pose.position.x = thisSystem.odom_x;
				msg_odom->pose.pose.position.y = thisSystem.odom_y;
				msg_odom->pose.pose.orientation.z = thisSystem.odom_r;

				msg_odom->twist.twist.linear.x = thisSystem.vel_x;
				msg_odom->twist.twist.linear.y = thisSystem.vel_y;
				msg_odom->twist.twist.angular.z = thisSystem.twist_z;
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
		String thisNode = "/socket_node_wifi";


		//int ros_loadParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, int defaultValue) {
		int ros_loadParameterInt(ros::NodeHandle* nh, const char* parName, int defaultValue=0) {
			String parPath = thisNs + thisNode + "/" + parName;
			int parValue = defaultValue;
			//ESP.wdtFeed();
			if (nh->getParam(parPath.c_str(), &parValue))
			{
				printf("\n Loaded int   [%s]:\t %d", parName, parValue);
				char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded int param  [%s]:\t %d", parName, parValue); nh->loginfo(ros_info_msg);
				dbgD("Param " + String(parName) + "= " + String(parValue));

				return parValue;
			}
			else
			{
				char ros_info_msg[200]; 
				sprintf(ros_info_msg, "***Warning*** int param. [%s] not found. Using default value %f ", parPath.c_str()); nh->logwarn(ros_info_msg);
				sprintf(ros_info_msg,"  Using default value %d ", defaultValue); nh->logwarn(ros_info_msg);
				printf("***Warning*** Int param. [%s] not found. \n\t\t Using default value %d \n\n", parPath.c_str(), defaultValue);
				printf(parPath.c_str());
				dbgD("Param " + String(parName) + " not found");
				return defaultValue;
			}
		}

		float ros_loadParameterFloat(ros::NodeHandle* nh, const char* parName, float defaultValue) {

			String parPath = thisNs + thisNode + "/" + parName;
			float parValue = defaultValue;
			//ESP.wdtFeed();//necessario
			if (nh->getParam(parPath.c_str(), &parValue))
			{
				String parValueStr = ftoa(parValue, 4, 10);
				printf("Loaded float [%s]:\t %s", parName, parValueStr.c_str());  // prinf con %f non è supportato
				char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded float param [%s]:\t  %s", parName, parValueStr.c_str()); nh->loginfo(ros_info_msg);
				dbgD("Param " + String(parName) + "= " + parValueStr);
				return parValue;
			}
			else
			{
				char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** float param. [%s] not found. Using default value %s ", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str()); nh->logwarn(ros_info_msg);
				printf(" ***Warning*** float param. [%s] not found. \n\t \tUsing default value %s \n\n", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str());
				dbgD("Param " + String(parName) + " not found");
				return defaultValue;
			}
		}
		String ros_loadParameterStr(ros::NodeHandle* nh, const char* parName, String defaultValue) {

			String parPath = thisNs + thisNode + "/" + parName;
			char**  parValuePtr;
			defaultValue.toCharArray(*parValuePtr,10);

			if (nh->getParam(parName, parValuePtr))
			{
				printf("\n Loaded [%s ]: %s", parName, parValuePtr);
				String returnStr;
				dbgD("Param " + String(parName) + "= " + String(*parValuePtr));

				return String(*parValuePtr);
			}
			else
			{
				dbgD("Param " + String(parName) + " not found");
				printf("\n ***Warning***  [%s] not found. Using default value %s", parPath.c_str(), defaultValue.c_str());
				return defaultValue;
			}
		}


		// chiamare dopo:		 while(!nh.connected()) {nh.spinOnce();}
		void ros_loadParameters() {
			//thisSystem.frame_id = ros_loadParameterStr("odom_frame", "");
			thisSystem.odom_x = ros_loadParameterFloat(&nh, "x", (float)0.0);
			thisSystem.odom_y = ros_loadParameterFloat(&nh, "y", (float)0.0);
			thisSystem.odom_r = ros_loadParameterFloat(&nh, "r", (float)0.0);
		}


	#pragma endregion // ROS_PARAMETERS

/// ///////////////////////////////////////////////////////////////
/// ////////////////////////////////////////////////////////////////
#pragma region  ROS MACRO REGION

#if 0
		void ros_initialize(String rosServer) {
			IPAddress rosIp;
			rosIp.fromString(rosServer);
			thisSystem.rosConnected = false;
			bool blFirstTime = true;


			dbg2("ros_initialize()] Connecting to Ros core  ", rosServer);
			// Ros objects constructors   
			nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);
			nh.initNode();


			//subscribe topics--------------------------------------------
			nh.subscribe(sub_raspicam_tilt_deg);



			//wait for ROS connection--------------------------------------------
			while (!nh.connected()) { vTaskDelay(500 / portTICK_PERIOD_MS /* 10Hz */); nh.spinOnce(); }
			dbg("[ROS connected]");
			dbgD("[ROS connected]");
			thisSystem.rosConnected = true;




			// inizializzo tf
			tfBr.init(nh);


			//inizializzo i messaggi
			msg_odom.header.stamp = nh.now();
			msg_odom.header.frame_id = "odom";
			msg_odom.child_frame_id = "base_link";
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
				thisSystem.rosConnected = true;
				dbgD("ROS Connected");
				dbg("ros_initialize()] ROS Connected");

			}


			dbg("[ros_initialize()] End");
			ROS_INFO("[ros_initialize()] End");

		}



		void setup_ROS(String rosServer, bool firstTime = true) {
			thisSystem.ros_master_uri = ROS_MASTER_URI_DEFAULT;

			//attende la connessione wifi
			while (!thisSystem.wifiStatus == wifiStatus_e::CONNECTED) { yield(); }


			if (firstTime)
			{
				ros_initialize(rosServer);
			}


			//"no more advertise admitted...");
			while (!nh.connected()) {
				thisSystem.rosConnected = false;

				nh.spinOnce();
				yield();
				//BLINK(500);

				///gestione pagina per impostare i parametri ROS  
				#if  WEBSERVER
						webServer.handleClient();
				#endif

				displayPanel();

			}

			// qui ci deve arrivare solo quando si è connesso a ROS
			//LDSStatus = LDSStatus_e::ROSCONNECTED;
			thisSystem.rosConnected = true;
			dbgD("CONNESSO A ROS");
			nh.loginfo("## ESP CONNESSO A ROS ##");
			//delay(5);

//			ros_loadParameters();
		}
#endif // 0
		/// inzializza  odom 

#pragma region Connect_ROS

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





		/// ///////////////////////////////////////////////////////////////
		/// ////////////////////////////////////////////////////////////////
		// si connette a ROS , fa l'advertise dei vari messaggi, e carica i parametri
		bool setup_ROS(int coreAddress) {
			if ((coreAddress < 1) || (coreAddress>255))
			{
				dbgD("Not Valid Ip Addr");
				dbg2("Not valid Ip Address.192.168.0.", coreAddress);
				return false;
			}

			dbg("Connecting to Ros...");
			dbgD("Check WiFi node");
			dbgD("Web server at: ");
			dbgD(WiFi.localIP().toString());
			dbgD("Connecting to Ros core at");
			dbgD("192.168.0." + String(coreAddress));


			IPAddress ros_core_server(192, 168, 0, coreAddress);


			// Ros objects constructors   
			nh.getHardware()->setConnection(ros_core_server, ROS_TCP_CONNECTION_PORT);
			nh.initNode();

			// inizializzo tf
			tfBr.init(nh);


			// ======================================
			//nh.subscribe(____);
			//nh.advertise(____);
			nh.advertise(pub_odom);
			nh.advertise(pub_encL );
			nh.advertise(pub_encR );
			//=========================================

			// recupero parametri


			nh.spinOnce();
			//delay(3000);

			// imposto webserver per poter cambiare alcuni parametri
			// Set web server port number to 80
			webServerStart();


			//"no more advertise admitted...");

			SystemStatus = systemStatus_e::ROSCONNECTED;
			dbg("\nCONNESSO A ROS");
			dbgD("##CONNESSO A ROS##");
			nh.loginfo("OK CONNESSO A ROS");

			delay(5);

		}

#pragma endregion	// Setup_ROS


	#pragma endregion	// Setup_ROS


#pragma endregion ROS macro region

// inizia ad acquisire n[samples] con cadenza costante data da  [time_interval_sec]  (intervallo dato da dt)
// debug su seriale delle distanze








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
		TaskHandle_t h_rosSpin;
		TaskHandle_t h_display;
		TaskHandle_t h_wifi;
		TaskHandle_t h_webserver;




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
				thisSystem.wifiStatus = wifiStatus_e::DISCONNECTED;
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
					thisSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;

					while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
						dbgD("RUN SMARTCONFIG");
						dbgD("ON SMARTPHONE");
							 
						vTaskDelay(xDelay_wifi);

					}

				}

				thisSystem.wifiStatus = wifiStatus_e::CONNECTED;
				dbg("[WIFI CONNECTED]");
				// segnala a tsk_ros che la connessione c'è
				/* Set bit 8 in the notification value of the task referenced by xTask1Handle. */
				//xTaskNotify(h_ros, (1UL << 8UL), eSetBits);
				//xSemaphoreGive(sem_wifi);

				vTaskDelay(xDelay_wifi);


			}
			else// wifi non connesso
			{
				vTaskDelay(10 * xDelay_wifi);
			}
			//displayPanel(-1);
		}



	}


	//---------------------------------------------------------------------------------------
	// task ROS
	//---------------------------------------------------------------------------------------





		// cambio strategia: dichiaro tutte le variabili qui dentro
		void tsk_ros_bkp(void * parameter) {
			const TickType_t tsk_rate = ROSRATE(thisSystem.rosRate);
			dbg("\n[tsk_ros START...]");
			uint32_t ulNotifiedValue;

			//attende la connessione wifi--------------------------------------
			///while (!thisSystem.wifiStatus == wifiStatus_e::CONNECTED) { vTaskDelay(10 * xDelay_ros); }
			while (!WiFi.isConnected()) { vTaskDelay(10 * tsk_rate); }


			displayCurrentFrameIndex = DISPLAY_FRAME_ROS;
			thisSystem.rosConnected = false;


			//// publisher -----------------------------
			ros::Publisher pub_odom("/odom", &msg_odom);
			ros::Publisher pub_encL("/encoder_left", &msg_encoderCountL);
			ros::Publisher pub_encR("/encoder_right", &msg_encoderCountR);



			//-------------------------------------
			//inizializzo i messaggi
			//-------------------------------------

			//// inizializzo tf
			tfBr.init(nh);

			// init msg_odom
			msg_odom.header.stamp = nh.now();
			msg_odom.header.frame_id = "odom";
			msg_odom.child_frame_id = "base_link";
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

			
			msg_encoderCountL.data = thisSystem.encL.count; //contatori assoluti
			msg_encoderCountR.data = thisSystem.encR.count;
			//-------------------------------------

			// messaggi-----------------------------
			odom_msg_update(&msg_odom, encoderL.read(), encoderR.read(), true);



			// Ros objects constructors   
			nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);
			nh.initNode();

			nh.advertise(pub_odom);
			nh.advertise(pub_encL);
			nh.advertise(pub_encR); //contatori assoluti
			//subscribe topics--------------------------------------------
			nh.subscribe(sub_initialPose);
			//nh.subscribe(sub_raspicam_tilt_deg);
			 








			//wait for ROS connection   NO MORE ADVERTISE --------------------------------------------
			dbgD("Connecting ROS");
			while (!nh.connected()) { 
				vTaskDelay(500 / portTICK_PERIOD_MS /* 10Hz */); 
				nh.spinOnce(); //nh.spinOnce() chiama negoziateTopics che a sua volta pone conneted = true;
							   ///gestione pagina per impostare i parametri ROS  
				#if  WEBSERVER
					webServer.handleClient();
				#endif
				displayPanel();
			}


			dbg("[ROS connected]");
			dbgD("[ROS connected]");
			nh.loginfo("## ESP32ROSODOM CONNECTED ##");
			thisSystem.rosConnected = true;

			// carica i parametri (dopo la connessione)--------------------------
			ros_loadParameters();
			dbg("[ROS Loaded params]");
			dbgD("[ROS Loaded params]");


			/////////////////////////////
			/// LOOP 
			/////////////////////////////
			for (;;)//task Loop phase
			{


				odom_msg_update(&msg_odom, encoderL.read(), encoderR.read(), true);
				msg_encoderCountL.data = thisSystem.encL.count;
				msg_encoderCountR.data = thisSystem.encR.count;



				pub_odom.publish(&msg_odom);
				pub_encL.publish(&msg_encoderCountL);
				pub_encR.publish(&msg_encoderCountR);

				nh.spinOnce();

				vTaskDelay(tsk_rate);

			}


		}

		//---------------------------------------------------------------------------------------

		void tsk_ros(void * parameter) {
			const TickType_t tsk_rate = ROSRATE(thisSystem.rosRate);
			dbg("\n[tsk_ros START...]");
			uint32_t ulNotifiedValue;

			//attende la connessione wifi--------------------------------------
			///while (!thisSystem.wifiStatus == wifiStatus_e::CONNECTED) { vTaskDelay(10 * xDelay_ros); }
			while (!WiFi.isConnected()) { vTaskDelay(10 * tsk_rate); }


			displayCurrentFrameIndex = DISPLAY_FRAME_ROS;
			thisSystem.rosConnected = false;




			setup_ROS(thisSystem.rosCoreAddress);








			//-------------------------------------
			//inizializzo i messaggi
			//-------------------------------------
			#pragma region ros_msgs_init



			// init msg_odom
			msg_odom.header.stamp = nh.now();
			msg_odom.header.frame_id = "odom";
			msg_odom.child_frame_id = "base_link";
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


			msg_encoderCountL.data = thisSystem.encL.count; //contatori assoluti
			msg_encoderCountR.data = thisSystem.encR.count;
			//-------------------------------------

			// messaggi-----------------------------
			odom_msg_update(&msg_odom, encoderL.read(), encoderR.read(), true);


			#pragma endregion


			//// spostato in ros_setup 
			//nh.getHardware()->setConnection(rosIp, ROS_TCP_CONNECTION_PORT);
			//nh.initNode();

			//nh.advertise(pub_odom);
			//nh.advertise(pub_encL);
			//nh.advertise(pub_encR); //contatori assoluti
			//						//subscribe topics--------------------------------------------
			//nh.subscribe(sub_initialPose);
			////nh.subscribe(sub_raspicam_tilt_deg);









			////wait for ROS connection   NO MORE ADVERTISE --------------------------------------------
			//dbgD("Connecting ROS");
			//while (!nh.connected()) {
			//	vTaskDelay(500 / portTICK_PERIOD_MS /* 10Hz */);
			//	nh.spinOnce(); //nh.spinOnce() chiama negoziateTopics che a sua volta pone conneted = true;
			//				   ///gestione pagina per impostare i parametri ROS  
			//#if  OPT_WEBSERVER
			//	webServer.handleClient();
			//#endif
			//	displayPanel();
			//}


			dbg("[ROS connected]");
			dbgD("[ROS connected]");
			nh.loginfo("## ESP32ROSODOM CONNECTED ##");
			thisSystem.rosConnected = true;

			// carica i parametri (dopo la connessione)--------------------------
			ros_loadParameters();
			dbg("[ROS Loaded params]");
			dbgD("[ROS Loaded params]");


			/////////////////////////////
			/// LOOP 
			/////////////////////////////
			for (;;)//task Loop phase
			{


				odom_msg_update(&msg_odom, encoderL.read(), encoderR.read(), true);
				msg_encoderCountL.data = thisSystem.encL.count;
				msg_encoderCountR.data = thisSystem.encR.count;



				pub_odom.publish(&msg_odom);
				pub_encL.publish(&msg_encoderCountL);
				pub_encR.publish(&msg_encoderCountR);

				nh.spinOnce();

				vTaskDelay(tsk_rate);

			}


		}


	//---------------------------------------------------------------------------------------
	// task MONITOR
	//---------------------------------------------------------------------------------------
	#if 1
		unsigned long tsk_monitor_lastloop = 0;
		float tsk_monitor_interval_ms = 1000.0 / 0.5; //Herz al denom.

		const TickType_t xDelay_monitor = 3000 / portTICK_PERIOD_MS;/* Block for 500ms. */
																	//Se sono passati i millis, manda su seriale i valori di thisSystem.encL.coun
		void tsk_monitor_loop() {
			if (millis() - tsk_monitor_lastloop > tsk_monitor_interval_ms) {
				dbg2("@ms: ", millis());
				//format the output for printing 
				char buf[50];
				sprintf(buf, "encoderL.read(): %d", encoderL.read()); 	Serial.println(buf);
				sprintf(buf, "encoderR.read(): %d", encoderR.read()); 	Serial.println(buf);
				sprintf(buf, "thisSystem.encL.count: %d", thisSystem.encL.count); 	Serial.println(buf);
				sprintf(buf, "thisSystem.encR.count: %d", thisSystem.encR.count); 	Serial.println(buf);

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
	//Se sono passati i millis, manda su seriale i valori di thisSystem.encL.coun
	#if 1
		const TickType_t xDelay_display = 500 / portTICK_PERIOD_MS;/* Block for 500ms. */
			void tsk_display(void * parameter) {
				dbg("\n[tsk_display STARTED... ]");
				while (true) //task loop
				{

					displayPanel(-1);
					vTaskDelay(xDelay_display);

				}

			}

	#endif // 1
	//---------------------------------------------------------------------------------------


	//---------------------------------------------------------------------------------------
	// task tsk_webserver
	//---------------------------------------------------------------------------------------
	#if 1


		const TickType_t xDelay_webserver = 1000 / TSK_ROS_HZ* portTICK_PERIOD_MS;

		void tsk_webserver(void * parameter) {

			dbg("\n[tsk_webserver START...]");
			//thisSystem.rosConnected = false;

			// Attendi la connessione WiFi
			while (!WiFi.isConnected()) { vTaskDelay(10 * xDelay_webserver); }
			//displayCurrentFrameIndex = DISPLAY_FRAME_ROS;

			Serial.println(WiFi.localIP());


			setup_webserver();

			while (true)
			{
				webServer.handleClient();
				if (thisSystem.rosConnected)
				{
					vTaskDelay(10*xDelay_webserver);
				}
				else
				{
					vTaskDelay(xDelay_webserver);
				}
				

			}



		}

	#endif // 1


		//---------------------------------------------------------------------------------------

		// lancia i vari task paralleli
		static int Core0 = 0;
		static int Core1 = 1;


		void tsk__launch() {
			//  http://esp32.info/docs/esp_idf/html/dd/d3c/group__xTaskCreate.html

			//			xTaskCreate(	task,   name of task, Stack size , Parameter , Priority , Task handle)
			//xTaskCreatePinnedToCore(	task,   name of task, Stack size , Parameter , Priority , Task handle, core)

			xTaskCreatePinnedToCore(tsk_wifi,		"wifi",		3000,	NULL, 3, &h_wifi,		Core1);
			xTaskCreatePinnedToCore(tsk_ros,		"ros",		6000,	NULL, 3, &h_ros,		Core1);
//			xTaskCreatePinnedToCore(tsk_rosSpin,	"rosSpin",	3000,	NULL, 3, &h_rosSpin,	Core1);
			xTaskCreatePinnedToCore(tsk_display,	"display",	3000,	NULL, 3, &h_display,	Core1);
//			xTaskCreatePinnedToCore(tsk_monitor,	"monitor",	3000,	NULL, 3, &h_monitor,	Core1);
			xTaskCreatePinnedToCore(tsk_webserver, "webserver", 3000,	NULL, 3, &h_webserver, Core1);


			LED_OFF;
			dbg("\n\n  tsk__launch()] ---all task created ---");

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

		thisSystem.encL.count = 0;
		thisSystem.encR.count = 0;

		thisSystem.wifiStatus = wifiStatus_e::DISCONNECTED;


		thisSystem.ros_master_uri = ROS_MASTER_URI_DEFAULT;
		thisSystem.ros_tcp_port = ROS_TCP_CONNECTION_PORT;
		thisSystem.rosConnected = false;
		thisSystem.ros_topic = "/odom";
		thisSystem.frame_id = "base_link";
		thisSystem.rosRate = 50;

		thisSystem.odom_x = 0.0;
		thisSystem.odom_y = 0.0;
		thisSystem.odom_r = 0.0;


		thisSystem.g_steps2m = ROBOT_STEPS2M;
		thisSystem.g_steps2rads = ROBOT_STEPS2RAD;
		thisSystem.g_enc2steps = ROBOT_MOTOR_STEPS_PER_ENCODERTICK;


		//Servo control
		thisSystem.raspicam_tilt_deg = 0;

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

		setup_encoder();

		dbg("\nEnd setup HW... ");


	}

	#pragma region setup_wifi_smartconfig
		#include "WiFi.h"
		void setup_wifi_smartconfig() {
			thisSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;

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

			thisSystem.wifiStatus = wifiStatus_e::CONNECTED;

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
				thisSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;
				WiFi.mode(WIFI_AP_STA); //WiFi.mode(WIFI_STA);
				WiFi.beginSmartConfig();
				while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
					delay(5000);
				}

			}

			thisSystem.wifiStatus = wifiStatus_e::CONNECTED;

		}
		//classic way
		void setup_wifi(char* ssid, char* pwd) {

			displayCurrentFrameIndex = 3;
			WiFi.begin(ssid,pwd); 
			Serial.printf("\nsetup_wifi] Connecting to SSID %s  ...", ssid);
			delay(5000);
			thisSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;
			while (WiFi.status() != WL_CONNECTED) {
				delay(2000);
				Serial.print(".");
				displayPanel(-1);
			}

			Serial.println("\nsetup_wifi] Connected to the WiFi network");
			dbgD("WIFI Connected  ");

			//if (!WiFi.isConnected()){

			//	//Init WiFi as Station, start SmartConfig
			//	thisSystem.wifiStatus = wifiStatus_e::SMARTCONFIG;
			//	WiFi.mode(WIFI_AP_STA); //WiFi.mode(WIFI_STA);
			//	WiFi.beginSmartConfig();
			//	while (!WiFi.smartConfigDone()) {//Wait for SmartConfig packet from mobile
			//		delay(5000);
			//	}

			//}

			thisSystem.wifiStatus = wifiStatus_e::CONNECTED;

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
		SERIAL_DBG.setDebugOutput(true);	// to enable output from printf() function.  >>http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#timing-and-delays
		dbg("\n\n\n\n\n ########### Esp32RosOdom v2 ########### \n" );
	#endif

	EEPROM_load();
	init_globalVars(); //integration layer
	//---------------------------------------------------------------------------------------
	// setup HW
	dbg("\nsetup HW.. ");
	setup_HW();	//imposta I2C,Display,Interrupt, direzione GPIO


	//---------------------------------------------------------------------------------------
	// setup DISPLAY
	dbg("\nsetup_display..\n");
	displayCurrentFrameIndex = DISPLAY_FRAME_MESSAGGI; //display msgs
	setup_display("ESP32 Odom ROS");





	//---------------------------------------------------------------------------------------
	// setup WEBSERVER
	//---------------------------------------------------------------------------------------
	#if OPT_WEBSERVER

		dbgD("setup webserver.. ");
		dbg("\nsetup webserver.. ");
		setup_webserver();	//  webserver per permettere di impostare i parametri WIFI e ROS 



	#endif // OPT_WEBSERVER


	displayCurrentFrameIndex = DISPLAY_FRAME_WIFI; // DISPLAY_FRAME_ENCODER; //display san

	//fine setup  inizia loop
	LED_OFF;
	dbg("\n\n --[STARTING LOOP]--");
	tsk__launch();
	dbgD("[STARTING LOOP]");


}



void loop() { 
	nh.spinOnce();

}






