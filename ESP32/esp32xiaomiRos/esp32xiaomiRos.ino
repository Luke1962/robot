/////////////////////////////////////////////////////////////////////
////  NODO ROS CHE SI INTERFACCIA A XIAOMI LIDAR
//	WIFI MANAGER
// PARAMETRI CARICATI DA ROS
/////////////////////////////////////////////////////////////////////
/*
	launch file:
	<launch>
		<!--serial communication between arduino and pc via WIFI  per LDS laser Scan /-->
		<node ns="robot"  name="serial_node_wifi" pkg="rosserial_python" type="serial_node.py" output="screen">
			<param name="port" value="tcp" />
			<param name="/scan_samples" value="50"/>
			<param name="/scan_speed" value="3.0"/>
			<param name="/return_speed" value="7.0"/>
		</node>
	</launch>
*/


/////////////////////////////////////////////////////////////////////
// PARAMETRI DI COMPILAZIONE
/////////////////////////////////////////////////////////////////////
#pragma region ParametriDiCompilazione

#define ESP32 1

#define MBEDTLS_CONFIG_FILE "config.h"

//	#include <SPI.h>


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
// PROBLEMI NOTI
/////////////////////////////////////////////////////////////////////
// FILE (debug) :

/////////////////////////////////////////////////////////////////////


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

#pragma region EEPROM_CONFIGURATION
	#include <EEPROMAnything.h>
	#include <EEPROM.h>
	const int N_ANGLES = 360;                // # of angles (0..359)

	struct EEPROM_Config {
		byte id;
		char version[6];

		int motor_pwm_pin;
		int ros_tcp_port;
		String ros_master_uri;
		int scan_motor_rpm;


		double motor_rpm;		//rpm del motore fornito dal lidar
		double rpm_setpoint;          // desired RPM (uses double to be compatible with PID library)
		double rpm_min;
		double rpm_max;
		double pwm_max;              // max analog value.  probably never needs to change from 1023
		double pwm_min;              // min analog pulse value to spin the motor
		int sample_time;             // how often to calculate the PID values

									 // PID tuning values
		double Kp;
		double Ki;
		double Kd;

		boolean motor_enable;        // to spin the laser or not.  No data when not spinning
		boolean raw_data;            // to retransmit the seiral data to the USB port
		boolean show_dist;           // controlled by ShowDist and HideDist commands
		boolean show_rpm;            // controlled by ShowRPM and HideRPM commands
		boolean show_interval;       // true = show time interval, once per revolution, at angle=0
		boolean show_errors;         // Show CRC, signal strength and invalid data errors
		boolean aryAngles[N_ANGLES]; // array of angles to display

		// memorizza i dato di scan_msg per renderli disponibili al Display
		char* frame_id;
		float angle_min;
		float angle_max;
		float angle_increment;


	}
	LDS;



	void saveConfig() {
		EEPROM_writeAnything(0, LDS);
		dbg(F("Config Saved."));
	}
#pragma endregion



// ///////////////////////////////////////////////////////////////////////////////
///
//		NODEMCU HW
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region NODEMCU_HW


void isr_displayChangeWindow(); // --> sezione OLED Display




	#pragma region ESP32_PIN
/*
						GPIO		LOLIN 32	GPIO
								-----------
						36	--- |			| --  39
						25	--- |			| --  16	PIN_FLASHBUTTON
						26	--- |			| --   5	SDA
						8	--- |			| --   4	SCL
						11	--- |			| --   0
						7	--- |			| --   2
						6	--- |			| --  14
						3.3v	|			| --  12
							GND	|			| --  13
							5v	|			| --  15
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

		#define PIN_LED				13
		#define PIN_FLASHBUTTON		0 
		// I2C
		#define PIN_SDA				5		//D2 su ESP8266
		#define PIN_CK				4		//D1 su ESP8266
		// LED EXT.
		#define ESP_PIN_LED2				36
		#define INTERRUPT_PIN 12 // use pin 15 on ESP8266

		#define LED_ON digitalWrite(PIN_LED, 1);
		#define LED_OFF digitalWrite(PIN_LED, 0);

		#define BLINK  LED_ON; delay(5); LED_OFF;
		#define BLINK(ms)  LED_ON; delay(ms); LED_OFF;

		//LASER
		//#define writeFast1(gpIO)	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 4,  (1<<gpIO));
		//#define writeFast0(gpIO) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 8,  (1<<gpIO));
		//#define writeFast(gpIO,on) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + (on?4,8),  (1<<gpIO));
		//#define TOGGLE_LED			writeFast(PIN_LED, !digitalRead(PIN_LED));

		#define GETBUTTON  !digitalRead(PIN_FLASHBUTTON)	/*negato perchè ritorna 0 se premuto*/	
		#pragma  endregion


	//imposta I2C,Display,Interrupt, direzione GPIO
	void setup_HW() {
		Wire.begin(PIN_SDA, PIN_CK);
		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
		setup_display();

		printESPinfo();

		// LED
		pinMode(PIN_LED, OUTPUT);

		pinMode(PIN_FLASHBUTTON, INPUT_PULLUP);
		pinMode(INTERRUPT_PIN, INPUT_PULLUP);

		//Commuta il frame del display visualizzato
		attachInterrupt(INTERRUPT_PIN, isr_displayChangeWindow, RISING);
		interrupts();  // abilita gli interrupt



		//pinMode(D4, FUNCTION_4);
		// comandi che possono tornare utili
		///uart_set_debug(UART0);
		///pinMode(uart->tx_pin, FUNCTION_4);







	}


	// memoria libera ----
	// si usa così: uint32_t free = system_get_free_heap_size();
	//extern "C" {	
	//	#include "user_interface.h"	
	//}

#pragma endregion	// NODEMCU_HW

// ///////////////////////////////////////////////////////////////////////////////
///
//		FREE RTOS
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region RTOS_Tasks

	SemaphoreHandle_t sem_scan_msg = xSemaphoreCreateBinary();
	/* initialize binary semaphore */


#pragma endregion


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




	const char* ssid = "FASTWEB-CSRLCU";
	const char* password = "cesarini";
	#define ROS_TCP_CONNECTION_PORT 11411
	#define ROS_MASTER_URI_DEFAULT	"192.168.0.51"
	IPAddress rosIp;

	const uint16_t port = 80;
	const char * host = ROS_MASTER_URI_DEFAULT; // ip or dns

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

	// rientra quando si è connesso al WiFi
	bool setup_WiFi_classic(int maxRetries = 5) {
		//Init WiFi as Station, start SmartConfig
		WiFi.mode(WIFI_AP_STA);
		WiFi.beginSmartConfig();

		//Wait for SmartConfig packet from mobile
		Serial.println("Waiting for SmartConfig.");
		while (!WiFi.smartConfigDone()) {
			delay(500);
			Serial.print(".");
		}

		Serial.println("");
		Serial.println("SmartConfig received.");

		//Wait for WiFi to connect to AP
		Serial.println("Waiting for WiFi");
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			dbg(".");
		}
		dbg("WiFi Connected.");

		dbg("IP Address: ");
		dbg(WiFi.localIP());
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
		if (now - isr_displayLastTime > 200)
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
		dbgD("# Xiaomi LDS ROS  #");
		dbgD("###################");


	}

	void displayScan();

	inline void displayFrameHeader(String frameName) {
		display.clear();
		String s = String(displayCurrentFrameIndex) + "] " + frameName;
		display.drawString(0, 0, s); //titolo											 
		display.drawString(90, 0, String(millis() / 1000) + "''");//millisecondi trascorsi
	}


	// Se frameNo <0 => Visualizza il frame corrente (incrementato da interrupt o dal pulsante flash)
	
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
		case 0:
			//messaggi		
			display.clear();
			displayFrameHeader("Messaggi");
			displayCircularBufferMsgs(false, false);
			display.display();

			break;

		case 1:
			//Scan map
			display.clear();
			displayFrameHeader("[Scan map]");

			s = "RPM:" + String((int)LDS.motor_rpm);
			display.drawString(0, DISPLAY_ROW(1), s);
			displayScan();


			display.display();
			break;			
		case 2:
			// ROS
			displayFrameHeader("[ROS]");

			s = "ROS URI:" + LDS.ros_master_uri ;
			display.drawString(0, DISPLAY_ROW(1), s);
			s = "TCP PORT:" + LDS.ros_tcp_port ;
			display.drawString(0, DISPLAY_ROW(2), s);



			display.display();
			break;
		case 3:		
			//WIFI

			display.clear();
			displayFrameHeader("[WiFi Config]");

			//s = "Mac:" +String( WiFi.macAddress);
			s = "Mac:" + getStrMacAddress();
			display.drawString(0, DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (0), s);

			s = "IP:" + WiFi.localIP().toString();
			display.drawString(0, DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (1), s);



			//s = "MQTT:" + String(mqtt_server);
			//display.drawString(0, DISPLAY_HEADER_SIZE_Y + DISPLAY_INTERLINEA * (2), s);

			display.display();
			break;

		case 4:
			////////////////////////
			//Scan Parameters
			display.clear();
			displayFrameHeader("Scan param.");

			s = "frame_id: " + String(LDS.frame_id);
			display.drawString(0, DISPLAY_ROW(0), s);
			s = "angle_increment: " + String(LDS.angle_increment);
			display.drawString(0, DISPLAY_ROW(1), s);
			s = "angle_min: " + String(LDS.angle_min);
			display.drawString(0, DISPLAY_ROW(2), s);
			s = "angle_max: " + String(LDS.angle_max);
			display.drawString(0, DISPLAY_ROW(3), s);


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
		SCANNING,
		ENDSCAN,
		PUBLISHING,
		ENDPUBLISH,
		SLOWPUBLISH, // la pubblicazione termina dopo l'arrivo a HOME
		SLOWSCAN	//la scansione termina dopo l'arrivo a END

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
	#include <sensor_msgs/Range.h> //ultrasound
	//	#include <tf/transform_broadcaster.h>

	//--------------------------------
	// TF 
	//--------------------------------
	//#include <tf/tf.h>
	//#include <tf/transform_broadcaster.h>
	//tf::TransformBroadcaster broadcaster;

	WiFiClient client;

	class WiFiHardware {

	public:
		WiFiHardware() {};
		void init( ) {
			 
		}

		void init(IPAddress server, uint16_t port) {
			client.connect(server, port);
		}
		void init(const char *host, uint16_t port) {
			client.connect(host,  port);
		}


		// accetta una stringa del tipo "192.168.0.51:11411"
		void init( char *hostWithPort) {
			char host[16];
			uint16_t portNo;
			int pos; // posizione :

			String s;
			pos= s.indexOf(":");

			if (pos>0)
			{
				//estraggo l'host ip fino al :
				s.substring(1, pos).toCharArray(host ,16);
				//estraggo la porta dopo il  :
				portNo = (uint16_t)s.substring(pos + 1).toInt();
				dbg2("host: ",host);
				dbg2("port: ",port);


			}
			else  // porta non specificata, uso quella di default
			{
				//estraggo l'host ip fino al :
				s.toCharArray(host, 16);

				portNo = 11411;
				pos = s.length();
			}


			client.connect(host,  portNo);
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
			for (int i = 0; i<length; i++)
				client.write(data[i]);
		}

		// returns milliseconds since start of program
		unsigned long time() {
			return millis(); // easy; did this one for you
		}
	};




		void chatterCallback(const std_msgs::String& msg) {
			dbgD(msg.data);
			
		}

		ros::Subscriber<std_msgs::String> sub("message", &chatterCallback);

		//	ros::NodeHandle  nh;
		ros::NodeHandle_<WiFiHardware> nh;
		//ros::NodeHandle nh;		// con aggiunta in Ros.h di: 	typedef NodeHandle_<Esp8266Hardware, 25, 25, 512, 1024> NodeHandle;

								//--------------------------------
								//	geometry_msgs::TransformStamped t;
								//	tf::TransformBroadcaster broadcaster;

								//--------------------------------
		//std_msgs::String str_msg;
		//ros::Publisher chatter("chatter", &str_msg);
		////--------------------------------
		//sensor_msgs::Range rosmsg_range;
		//ros::Publisher pub_range("ultrasound", &rosmsg_range);

		//--------------------------------
		#define ROS_INFO(s) nh.loginfo(s);

		// laser data---------------








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



		sensor_msgs::LaserScan scan_msg;;
		ros::Publisher pub_Laser("/scan", &scan_msg);

		float scanSpeed = 300; //velocità di rotazione  in rpm; 300 = 5Hz
		int   LDSsamples;

		// Costanti indipendenti
		#define SCAN_SAMPLES_MAX 360	// dimensione dell'array
		#define LDSspeed_default PI		// PI rad/sec = 180�/sec 

		// Angolo massimo= un po' meno di 270° ( vanno tolti 3 step da 1.8° = 5.4°)
		// 270 - 5.4 = 264.6° =  264.6° = 4.618 rad
		//  (rad=  d*2Pi/360 )
		#define SCAN_ANGLE_MAX	2*M_PI		// 270° , SCAN_ANGLE_MAX_MAX - SCAN_ANGLE_MAX_MIN

		#define	SCAN_RANGE_MIN 0.05f
		#define	SCAN_RANGE_MAX 5.0f


		// Costanti derivate

		#define SCAN_ANGLE_MAX_MIN  0	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		#define SCAN_ANGLE_MAX_MAX   2*PI

		#define SCAN_ANGLE_MAX_INCREMENT  SCAN_ANGLE_MAX / LDSsamples;  // (SCAN_ANGLE_MAX_MAX - SCAN_ANGLE_MAX_MIN)/ LDSsamples 
		#define SCAN_TIME  0.01 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX_MAX-SCAN_ANGLE_MAX_MIN )/LDSspeed
		#define SCAN_TIME_INCREMENT_MS  SCAN_TIME/LDSsamples
		#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667

		//	float ranges[LDSsamples]; // max of 30 measurements
		//float intensities[SCAN_SAMPLES_MAX]; // buffer 
		//------------------------
		float* intensities = new float[SCAN_SAMPLES_MAX];
		float* ranges = new float[SCAN_SAMPLES_MAX];
		unsigned long scan_time;


		// inizializza il contenuto di scan_msg in base ai parametri 
		void setup_scan_msg(sensor_msgs::LaserScan* scan_msg, float scanAngle, float scanTime, float timeIncrement, int LDSsamples) {
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
			scan_msg->angle_min = 0;
			scan_msg->angle_max = M_TWOPI;
			scan_msg->range_min = SCAN_RANGE_MIN;
			scan_msg->range_max = SCAN_RANGE_MAX;

			// tempo di andata + ritorno
			scan_msg->scan_time = scanTime;	// 3.91 tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE_MAX / LDSspeed ;
			scan_msg->angle_increment = 1;
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

		#pragma endregion // ROS SCAN


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


		int loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, int defaultValue) {
		//int loadRosParameter(ros::NodeHandle* nh, const char* parName, int defaultValue) {
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
		float loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, float defaultValue) {
		//float loadRosParameter(ros::NodeHandle* nh, const char* parName, float defaultValue) {

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
			SERIAL_DBG.print("connecting to ");
			SERIAL_DBG.println(host);
			const int httpPort = 21;
			if (!client.connect(host, httpPort)) {
				SERIAL_DBG.println("connection failed");
				return;
			}
		}
		*/


		void ROS_loadParameters() {

			//LDSsamples =SCAN_SAMPLES_DEFAULT; //con 40 funziona  devono essere minore di  SCAN_SAMPLES_MAX

			//LDSsamples = loadRosParameter(&nh, "scan_samples", SCAN_SAMPLES_DEFAULT);
			LDS.scan_motor_rpm = loadRosParameter(&nh, "scan_speed_rpm", (int)300);

			
			//LDSsamples = 360; 
			//float scanAngle = 2*M_PI; //360°

			////time_increment = tempo di scansione / samples
			//float timeIncrement = (scanAngle / LDS.scan_motor_rpm) / LDSsamples;

			//// tempo tra due scansioni
			//float scanTime = scanAngle / LDS.scan_motor_rpm;	//
																				//float angleIncrement = scanAngle / LDSsamples;
			//setup_scan_msg(&scan_msg, scanAngle, scanTime, timeIncrement, LDSsamples);


		}


	#pragma endregion // ROS_PARAMETERS

	/// ///////////////////////////////////////////////////////////////
	/// ////////////////////////////////////////////////////////////////
	#pragma region Setup_ROS

		void setup_ROS(String rosServer, bool firstTime = true) {

			// verifica prima la connessione wifi------------------
			//if (WiFi.status() != WL_CONNECTED)
			//{
			//		dbg("wifi not connected...connecting...");
			//		WiFi.connect(host, ROS_TCP_CONNECTION_PORT);

			//}


			dbg2("\n[setup_ROS()] Connecting to Ros..", rosServer);

			if (firstTime)
			{

				IPAddress rosIp;
				rosIp.fromString(rosServer);

				/// imposta solamente i parametri di server e porta, non so connette 
				///nh.getHardware()->setConnection(ros_core_server, ROS_TCP_CONNECTION_PORT);


				client.connect(rosIp, ROS_TCP_CONNECTION_PORT); //client.connect(server, ROS_TCP_CONNECTION_PORT);
				nh.initNode();


				if (client.connected()	)
				{
					dbgD("Cilent Connected");
				}

				/// recupero parametri-------------------
				///motor_rpm = loadRosParameter(&nh,"scan_speed_rpm", 300);

				/// esegue gli advertise----------
				//nh.advertise(chatter);




				LDSsamples = SCAN_SAMPLES_MAX;
				float scanAngle = 2 * M_PI; //360°

				//time_increment = tempo di scansione / samples
				float timeIncrement = (scanAngle / LDS.scan_motor_rpm) / LDSsamples;

				// tempo tra due scansioni
				float scanTime = scanAngle / LDS.scan_motor_rpm;	//


//spostato in setup				setup_scan_msg(&scan_msg,360,0.5,0.005,360);

				nh.advertise(pub_Laser);
				dbg("\n[setup_ROS()] End");
				//nh.spinOnce();
				//delay(3000);
				ROS_INFO("[setup_ROS()] End");
			}





			//"no more advertise admitted...");
			while (!nh.connected()) {

				nh.spinOnce();
				delay(1000);
				//	BLINK(500);

				///gestione pagina per impostare i parametri ROS  
				#if  WEBSERVER
						server.handleClient();
				#endif

				displayFrame();

			}


			//LDSStatus = LDSStatus_e::ROSCONNECTED;
			dbgD("CONNESSO A ROS");
			nh.loginfo("## ESP CONNESSO A ROS ##");
			delay(5);

			//ROS_loadParameters();




		}

	#pragma endregion	// Setup_ROS


#pragma endregion ROS macro region

// inizia ad acquisire n[samples] con cadenza costante data da  [time_interval_sec]  (intervallo dato da dt)
// debug su seriale delle distanze





// ///////////////////////////////////////////////////////////////////////////////
///
//		XIAOMI DRIVER
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region XIAOMI_DRIVER




///#include <sensor_msgs/LaserScan.h>

	//xv_11_laser_driver::XV11Laser laser(&SerialLidar);

	bool scanAvailable = false; // true quando l'array contiene una scansione completa


	//---------------------------------------------------------------
	const int SHOW_ALL_ANGLES = N_ANGLES;    // value means 'display all angle data, 0..359'


	const byte EEPROM_ID = 0x07;   // used to validate EEPROM initialized

	double pwm_val = 500;          // start with ~50% power
	double pwm_last;
//	double motor_rpm;  //velocità del motore in rpm dal lidar   spostato in LDS.motor_rpm
//	unsigned long now;
	unsigned long motor_check_timer = millis();
	unsigned long motor_check_interval = 200;
	unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
	unsigned int rpm_err = 0;
	unsigned long curMillis;
	unsigned long lastMillis = millis();

	const unsigned char COMMAND = 0xFA;        // Start of new packet
	const int INDEX_LO = 0xA0;                 // lowest index value
	const int INDEX_HI = 0xF9;                 // highest index value

	const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
	const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

	 // Offsets to bytes within 'Packet'
	const int OFFSET_TO_START = 0;
	const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
	const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
	const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
	const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
	const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
	const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
	const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet
													// Offsets to the (4) elements of each of the (4) data quads
	const int OFFSET_DATA_DISTANCE_LSB = 0;
	const int OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
	const int OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
	const int OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

	int Packet[PACKET_LENGTH];                 // an input packet
	int ixPacket = 0;                          // index into 'Packet' array
	const int VALID_PACKET = 0;
	const int INVALID_PACKET = VALID_PACKET + 1;
	const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"




	/* REF: https://github.com/Xevel/NXV11/wiki
	The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
	It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
	only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
	data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
	interrupted by the supports of the cover) .
	*/
	const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
	/*
	The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
	This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
	size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
	reflections (glass... ).
	*/
	const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);






	const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
	const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet

	int eState = eState_Find_COMMAND;


	//PID rpmPID(&motor_rpm, &pwm_val, &LDS.rpm_setpoint, LDS.Kp, LDS.Ki, LDS.Kd, DIRECT);

	uint8_t inByte = 0;  // incoming serial byte
	uint8_t motor_rph_high_byte = 0;
	uint8_t motor_rph_low_byte = 0;
uint16_t motor_rph = 0;
	uint16_t aryDist[N_DATA_QUADS] = { 0, 0, 0, 0 };   // thre are (4) distances, one for each data quad
													   // so the maximum distance is 16383 mm (0x3FFF)
	uint16_t aryQuality[N_DATA_QUADS] = { 0, 0, 0, 0 }; // same with 'quality'
	
	uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

													 /*
													 processIndex - Process the packet element 'index'
													 index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
													 (packet 89, readings 356 to 359).
													 Enter with: N/A
													 Uses:       Packet
													 ledState gets toggled if angle = 0
													 ledPin = which pin the LED is connected to
													 ledState = LED on or off
													 LDS.show_dist = true if we're supposed to show distance
													 curMillis = milliseconds, now
													 lastMillis = milliseconds, last time through this subroutine
													 LDS.show_interval = true ==> display time interval once per revolution, at angle 0
													 Calls:      digitalWrite() - used to toggle LED pin
													 SERIAL_DBG.print
													 Returns:    The first angle (of 4) in the current 'index' group
													 */
	
	
	boolean ledState = LOW;

	
	
	uint16_t processIndex() {
		uint16_t angle = 0;
		uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
		angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4
		if (angle == 0) {
			if (ledState) {
				ledState = LOW;

			}
			else {
				ledState = HIGH;
			}
			

			#if OPT_USE_SERIAL_DBG
				if (LDS.show_rpm) {
						SERIAL_DBG.print(F("R,"));
						SERIAL_DBG.print((int)LDS.motor_rpm);
						SERIAL_DBG.print(F(","));
						SERIAL_DBG.println((int)pwm_val);
				}

				curMillis = millis();
				if (LDS.show_interval) {
					SERIAL_DBG.print(F("T,"));                                // Time Interval in ms since last complete revolution
					SERIAL_DBG.println(curMillis - lastMillis);
				}
			#endif
			lastMillis = curMillis;

		} // if (angle == 0)
		return angle;
	}
	/*
	processSpeed- Process the packet element 'speed'
	speed is two-bytes of information, little-endian. It represents the speed, in 64th of RPM (aka value
	in RPM represented in fixed point, with 6 bits used for the decimal part).
	Enter with: N/A
	Uses:       Packet
	angle = if 0 then enable display of RPM and PWM
	LDS.show_rpm = true if we're supposed to display RPM and PWM

	*/
	void processSpeed() {
		motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
		motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
		motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
		LDS.motor_rpm = float((motor_rph_high_byte << 8) | motor_rph_low_byte) / 64.0;
	}
	/*
	Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
	byte 0 : <distance 7:0>
	byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
	byte 2 : <signal strength 7:0>
	byte 3 : <signal strength 15:8>
	*/



	/*
	processDistance- Process the packet element 'distance'
	Enter with: iQuad = which one of the (4) readings to process, value = 0..3
	Uses:       Packet
	dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
	so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
	Calls:      N/A
	Exits with: 0 = okay
	Error:      1 << 7 = INVALID_DATA_FLAG is set
	1 << 6 = STRENGTH_WARNING_FLAG is set
	*/
	byte processDistance(int iQuad) {
		uint8_t dataL, dataM;
		aryDist[iQuad] = 0;                     // initialize
		int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
		// byte 0 : <distance 7:0> (LSB)
		// byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
		dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
		if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
			return dataM & BAD_DATA_MASK;        // ...then return non-zero
		dataL = Packet[iOffset];               // LSB of distance data
		aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
		return 0;                              // okay
	}
	/*
	processSignalStrength- Process the packet element 'signal strength'
	Enter with: iQuad = which one of the (4) readings to process, value = 0..3
	Uses:       Packet
	quality[] = signal quality
	Calls:      N/A
	*/
	void processSignalStrength(int iQuad) {
		uint8_t dataL, dataM;
		aryQuality[iQuad] = 0;                        // initialize
		int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
		dataL = Packet[iOffset];                  // signal strength LSB
		dataM = Packet[iOffset + 1];
		aryQuality[iQuad] = dataL | (dataM << 8);
	}


	/*
	eValidatePacket - Validate 'Packet'
	Enter with: 'Packet' is ready to check
	Uses:       CalcCRC
	Exits with: 0 = Packet is okay
	Error:      non-zero = Packet is no good
	*/
	byte eValidatePacket() {
		unsigned long chk32;
		unsigned long checksum;
		const int bytesToCheck = PACKET_LENGTH - 2;
		const int CalcCRC_Len = bytesToCheck / 2;
		unsigned int CalcCRC[CalcCRC_Len];

		byte b1a, b1b, b2a, b2b;
		int ix;

		for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
			CalcCRC[ix] = 0;

		// Perform checksum validity test
		for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
			CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

		chk32 = 0;
		for (ix = 0; ix < CalcCRC_Len; ix++)
			chk32 = (chk32 << 1) + CalcCRC[ix];
		checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
		checksum &= 0x7FFF;
		b1a = checksum & 0xFF;
		b1b = Packet[OFFSET_TO_CRC_L];
		b2a = checksum >> 8;
		b2b = Packet[OFFSET_TO_CRC_M];
		if ((b1a == b1b) && (b2a == b2b))
			return VALID_PACKET;                       // okay
		else
			return INVALID_PACKET;                     // non-zero = bad CRC
	}

	// legge i dati lidar e li trasferisce in scan_msg
	// ritorna true quando è disponibile una scansione completa
	bool lidar2scan(sensor_msgs::LaserScan* scan_msg ) {
		boolean blScancomplete = false;

		byte aryInvalidDataFlag[N_DATA_QUADS] = { 0, 0, 0, 0 }; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

		int waitingBytes = SERIAL_LDS.available();
		if (waitingBytes> 0) {                  // read byte from LIDAR and relay to USB
												//printf("\nbytes in:%d", waitingBytes);

												////ESP.wdtFeed();
			for (int i = 0; i < waitingBytes; i++)	// per ogni byte in attesa
			{


				uint8_t inByte = SERIAL_LDS.read();				// get incoming byte:




				if (LDS.raw_data) { dbg(inByte); }				// relay
																// Switch, based on 'eState':
																// State 1: We're scanning for 0xFA (COMMAND) in the input stream
																// State 2: Build a complete data packet
				if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
					if (inByte == COMMAND) {
						eState++;                                 // switch to 'build a packet' state
						Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
					}
				}
				else {                                            // eState == eState_Build_Packet
					Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
					if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
						if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
							startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
							processSpeed();                             // process the speed
																		// process each of the (4) sets of data in the packet


							for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
								aryInvalidDataFlag[ix] = processDistance(ix);

							for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
								aryQuality[ix] = 0;
								if (aryInvalidDataFlag[ix] == 0)
									processSignalStrength(ix);
							}




							// PER OGNI BYTE DEL GRUPPO DI 4 BYTE DEL PACCHETTO
							for (int ix = 0; ix < N_DATA_QUADS; ix++) {
								if (LDS.aryAngles[startingAngle + ix]) {             // if we're supposed to display that angle
									if (aryInvalidDataFlag[ix] & BAD_DATA_MASK) {  // if LIDAR reported a data error...

									}
									else {



										///###################################
										// Travaso in scan
										///###################################
										//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
										// startingAngle va a multipli di 4 

										/*										// se l'indice di ranges aumenta come l'indice startingAngle + ix , una rotazione CCW del sistema
										// genera una rotazione dell'immagine nello stesso senso, quando invece dovrebbe essere in controfase
										scan_msg->ranges[(startingAngle + ix)% SCAN_SAMPLES_MAX] = (float)aryDist[ix] / 1000;
										scan_msg->intensities[(startingAngle + ix) % SCAN_SAMPLES_MAX] = aryQuality[ix];
										*/

										// così invece riempio ranges[] al contrario
										scan_msg->ranges[(SCAN_SAMPLES_MAX - 1) - (startingAngle + ix) % SCAN_SAMPLES_MAX] = (float)aryDist[ix] / 1000;
										scan_msg->intensities[(SCAN_SAMPLES_MAX - 1) - (startingAngle + ix) % SCAN_SAMPLES_MAX] = aryQuality[ix];


									}
								}  // if (LDS.aryAngles[startingAngle + ix])
							}  // for (int ix = 0; ix < N_DATA_QUADS; ix++)

						}  // if (eValidatePacket() == 0



						// initialize a bunch of stuff before we switch back to State 1
						for (int ix = 0; ix < N_DATA_QUADS; ix++) {
							aryDist[ix] = 0;
							aryQuality[ix] = 0;
							aryInvalidDataFlag[ix] = 0;
						}


						for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
							Packet[ixPacket] = 0;
						ixPacket = 0;
						eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte

					}  // if (ixPacket == PACKET_LENGTH)
				}  // if (eState == eState_Find_COMMAND)


			} //for each waitingBytes


			  /// Scansione completa ?
			if (startingAngle >= 356)
			{
				blScancomplete = true;
			}


		}  // if (SERIAL_LDS.available() > 0)








		return blScancomplete;


	}
	bool lidar2scan_debug(sensor_msgs::LaserScan* scan_msg) {
		boolean blScancomplete = false;

		byte aryInvalidDataFlag[N_DATA_QUADS] = { 0, 0, 0, 0 }; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set
		
		int waitingBytes = SERIAL_LDS.available();
		if (waitingBytes> 0) {                  // read byte from LIDAR and relay to USB
												//printf("\nbytes in:%d", waitingBytes);

												////ESP.wdtFeed();
			for (int i = 0; i < waitingBytes; i++)	// per ogni byte in attesa
			{


				uint8_t inByte = SERIAL_LDS.read();				// get incoming byte:




				if (LDS.raw_data) { dbg(inByte); }				// relay
																// Switch, based on 'eState':
																// State 1: We're scanning for 0xFA (COMMAND) in the input stream
																// State 2: Build a complete data packet
				if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
					if (inByte == COMMAND) {
						eState++;                                 // switch to 'build a packet' state
						Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
					}
				}
				else {                                            // eState == eState_Build_Packet
					Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
					if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
						if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
							startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
							processSpeed();                             // process the speed
																		// process each of the (4) sets of data in the packet


							for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
								aryInvalidDataFlag[ix] = processDistance(ix);

							for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
								aryQuality[ix] = 0;
								if (aryInvalidDataFlag[ix] == 0)
									processSignalStrength(ix);
							}




							// PER OGNI BYTE DEL GRUPPO DI 4 BYTE DEL PACCHETTO
							for (int ix = 0; ix < N_DATA_QUADS; ix++) {
								if (LDS.aryAngles[startingAngle + ix]) {             // if we're supposed to display that angle
									if (aryInvalidDataFlag[ix] & BAD_DATA_MASK) {  // if LIDAR reported a data error...


#if OPT_USE_SERIAL_DBG
										if (LDS.show_errors) {                           // if we're supposed to show data errors...
											SERIAL_DBG.print(F("A,"));
											SERIAL_DBG.print(startingAngle + ix);
											SERIAL_DBG.print(F(","));
											if (aryInvalidDataFlag[ix] & INVALID_DATA_FLAG)
												SERIAL_DBG.println(F("I"));
											if (aryInvalidDataFlag[ix] & STRENGTH_WARNING_FLAG)
												SERIAL_DBG.println(F("S"));
										}
#endif

									}
									else {
										//###################################
										// show clean data
										//###################################
#if OPT_USE_SERIAL_DBG
										if (LDS.show_dist) {
											SERIAL_DBG.print(F("A,"));
											SERIAL_DBG.print(startingAngle + ix);
											SERIAL_DBG.print(F(","));
											SERIAL_DBG.print(int(aryDist[ix]));
											SERIAL_DBG.print(F(","));
											SERIAL_DBG.println(aryQuality[ix]);
										}
#endif
										//###################################


										///###################################
										// Travaso in scan
										///###################################
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
										// startingAngle va a multipli di 4 
										
/*										// se l'indice di ranges aumenta come l'indice startingAngle + ix , una rotazione CCW del sistema 
										// genera una rotazione dell'immagine nello stesso senso, quando invece dovrebbe essere in controfase
										scan_msg->ranges[(startingAngle + ix)% SCAN_SAMPLES_MAX] = (float)aryDist[ix] / 1000;
										scan_msg->intensities[(startingAngle + ix) % SCAN_SAMPLES_MAX] = aryQuality[ix];
*/

										// così invece riempio ranges[] al coontrario
										scan_msg->ranges[(SCAN_SAMPLES_MAX -1)- (startingAngle + ix)% SCAN_SAMPLES_MAX] = (float)aryDist[ix] / 1000;
										scan_msg->intensities[(SCAN_SAMPLES_MAX - 1) - (startingAngle + ix) % SCAN_SAMPLES_MAX] = aryQuality[ix];


									}
								}  // if (LDS.aryAngles[startingAngle + ix])
							}  // for (int ix = 0; ix < N_DATA_QUADS; ix++)

						}  // if (eValidatePacket() == 0
#if OPT_USE_SERIAL_DBG 
						else if (LDS.show_errors) {                                // we have encountered a CRC error
							SERIAL_DBG.println(F("C,CRC"));
						}
#endif

						// initialize a bunch of stuff before we switch back to State 1
						for (int ix = 0; ix < N_DATA_QUADS; ix++) {
							aryDist[ix] = 0;
							aryQuality[ix] = 0;
							aryInvalidDataFlag[ix] = 0;
						}


						for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
							Packet[ixPacket] = 0;
						ixPacket = 0;
						eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte

					}  // if (ixPacket == PACKET_LENGTH)
				}  // if (eState == eState_Find_COMMAND)


			} //for each waitingBytes


			  /// Scansione completa ?
			if (startingAngle >= 356)
			{
				blScancomplete = true;
			}


		}  // if (SERIAL_LDS.available() > 0)








		return blScancomplete;

	}

	/*
	initEEPROM
	*/

	void setup_lidar() {		// was initEEPROM
		LDS.id = 0x07;
		strcpy(LDS.version, "1.4.0");


		LDS.motor_pwm_pin = 5;  // pin connected N-Channel Mosfet


		LDS.motor_rpm = 0;
		LDS.rpm_setpoint = 200;  // desired RPM
		LDS.rpm_min = 200;
		LDS.rpm_max = 300;
		LDS.pwm_min = 100;
		LDS.pwm_max = 1023;
		LDS.sample_time = 20;
		LDS.Kp = 2.0;
		LDS.Ki = 1.0;
		LDS.Kd = 0.0;

		LDS.motor_enable = false; // true;//
		LDS.raw_data = false; // true;//
		LDS.show_dist =false; //true;// 
		LDS.show_rpm =true;	// false; //  		457/ su 500
		LDS.show_interval = true;// false;//
		LDS.show_errors =false; // true;//
		for (int ix = 0; ix < N_ANGLES; ix++) 		LDS.aryAngles[ix] = true;

		setup_scan_msg(&scan_msg, 360, 0.5, 0.005, 360);

//		EEPROM_writeAnything(0, LDS);

	}





#pragma endregion	//XIAOMI DRIVER




	// ///////////////////////////////////////////////////////////////////////////////
	///
	//		LAYER DI INTEGRAZIONE FRA I VARI COMPONENTI
	///
	// ///////////////////////////////////////////////////////////////////////////////
#pragma region IntegrationLayer

	#pragma region setup_wifi_smartconfig
		#include "WiFi.h"
		void setup_wifi_smartconfig() {

			/* Set ESP32 to WiFi Station mode */
			WiFi.mode(WIFI_AP_STA);
			/* start SmartConfig */
			WiFi.beginSmartConfig();

			/* Wait for SmartConfig packet from mobile */
			dbgD("Waiting SmartConfig.");
			while (!WiFi.smartConfigDone()) {
				delay(500);
				BLINK(5);
			}
			//Serial.println("");
			Serial.println("SmartConfig done.");
			dbgD("SmartConfig done.");
			/* Wait for WiFi to connect to AP */
			Serial.println("Waiting for WiFi");
			while (WiFi.status() != WL_CONNECTED) {
				delay(500);
				Serial.print(".");
			}
			Serial.println("WiFi Connected.");
			Serial.print("IP Address: ");
			Serial.println(WiFi.localIP());
			dbgD("WiFi Connected.");
		}

		// ritorna true se riesce a connettersi al wifi
		bool setup_WiFi(int maxRetries = 5)
		{

			bool connected = false;


			WiFi.begin(ssid, password);

			String mac = WiFi.macAddress();


			uint8_t i = 0;
			while (WiFi.status() != WL_CONNECTED && i++ < maxRetries) delay(1000);



			if (WiFi.status() != WL_CONNECTED) {
				connected = false;
				dbg2("Could not connect to: ", ssid);
				dbgD("Cant connect: "+String( ssid));


				// LED flash per 3 sec...
				unsigned long t0 = millis();
				while ((millis() - t0) < 3000)
				{
					BLINK(200);	delay(500);
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




				//		SystemStatus = systemStatus_e::WIFICONNECTED;
				delay(2000);
			}

			return connected;
		}



	#pragma endregion

	#define TSK_ROS_HZ 5.0f //frequenza alla quale deve girare il task

	void tsk_ros(void * parameter) {
		//Setup phase
		dbg("\ntsk_ros:setup WiFi... \n");
		dbgD("setup WiFi... \n");
		//connectToWiFi(); //wifiManager non funziona con ESP32
		setup_WiFi();
		//setup_wifi_smartconfig();


		unsigned long tsk_ros_lastloop = 0;
		float taskPeriod = 1000.0 / TSK_ROS_HZ;
		dbgD("WIFI CONNESSO \n");



		dbgD("ROS URI: " + LDS.ros_master_uri);
		setup_ROS(LDS.ros_master_uri); //fa il setup dei vari nodi e legge i parametri

		//Loop phase
		while (true)
		{
			if (nh.connected())
			{
				if (millis()-tsk_ros_lastloop >taskPeriod)
				{
					xSemaphoreTake(sem_scan_msg, portMAX_DELAY);
					pub_Laser.publish(&scan_msg);
					xSemaphoreGive(sem_scan_msg);

				}

			}
			else
			{
				// ROS non più connesso

			}

		}

	}

	// lancia i vari task paralleli
	void setup_tasks() {


		xTaskCreate(
			tsk_ros,          /* Task function. */
			"ros",        /* String with name of task. */
			10000,            /* Stack size in words. */
			NULL,             /* Parameter passed as input of the task */
			1,                /* Priority of the task. */
			NULL);            /* Task handle. */

							  // inizia
		LED_OFF;
		dbg("\n\n --- Start /scan loop ---");

	}

#pragma endregion  //IntegrationLayer


#pragma region webServer
	#include <WiFi.h>
	#include <FS.h>
	#include <AsyncTCP.h>
	#include <ESPAsyncWebServer.h>

	//const char* ssid = "yourNetworkSSID";
	//const char* password = "yourNetworkPassword";

	AsyncWebServer server(80);

	const char HTML[] PROGMEM = "<form onSubmit=\"event.preventDefault()\"><label class=\"label\">Network Name</label><input type=\"text\" name=\"ssid\"/><br/><label>Password</label><input type=\"text\" name=\"pass\"/><br/><input type=\"submit\" value=\"Submit\"></form>";

	void setup_webserver() {
		//Serial.begin(115200);

		//WiFi.begin(ssid, password);

		//while (WiFi.status() != WL_CONNECTED) {
		//	delay(1000);
		//	Serial.println("Connecting to WiFi..");
		//}

		//Serial.println(WiFi.localIP());

		server.on("/html", HTTP_GET, [](AsyncWebServerRequest *request) {
			request->send(200, "text/html", HTML);
		});

		server.begin();
	}
#pragma endregion

// ///////////////////////////////////////////////////////////////////////////////
///
//		S E T U P
///
// ///////////////////////////////////////////////////////////////////////////////

void setup() {
	bool blSkipConnections = false;
	if (GETBUTTON == 1) /*se tengo premuto inizialmente salto il setup wifi*/
	{
		blSkipConnections = true;
	}

	#if OPT_USE_SERIAL_DBG 
		SERIAL_DBG.begin(SERIAL_SPEED);
		SERIAL_DBG.setDebugOutput(true);// to enable output from printf() function.  >>http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#timing-and-delays
		dbg("\n######## ### Xiaomi driver v0.1 #####\n\n\n\n\n");
	#endif
	SERIAL_LDS.begin(SERIAL_SPEED);






	setup_display();

	delay(2000);
	LDS.ros_master_uri = ROS_MASTER_URI_DEFAULT;
	setup_HW();	//imposta I2C,Display,Interrupt, direzione GPIO


	setup_lidar(); //imposta LDS rpm, pwm, parametri PID,scan_msg e se fare debug dei dati dal LIDAR

	//dbgD("setup WiFi... \n");
	////connectToWiFi(); //wifiManager non funziona con ESP32
	
	if (!blSkipConnections ) /*se tengo premuto inizialmente salto il setup wifi*/
	{
		setup_WiFi();
		dbgD("WIFI CONNESSO \n");

	}
	else
	{
		dbgD("SKIP WiFi ");
	}




	setup_webserver();
	if (!blSkipConnections) /*se tengo premuto inizialmente salto il setup wifi*/
	{
		dbgD("ROS URI: " + LDS.ros_master_uri);
		setup_ROS(LDS.ros_master_uri); //fa il setup dei vari nodi e legge i parametri
	}
	else
	{
		dbgD("SETUP ROS SKIPPED");

	}



	// test era qui-------------


	//setup_tasks();

	// inizia
	LED_OFF;
	dbg("\n\n --- Start /scan loop ---");
	dbgD("--[STARTING LOOP]--");

	displayCurrentFrameIndex = 1; //display san


	// test-------------
#if 0
	unsigned long nextMsg_time;
	dbg("\nTEST MODE------");
	dbgD("TEST MODE------");
	uint8_t buff[100];
	while (true)
	{
		while (!scanAvailable)// attendo sulla seriale una scansione completa
		{
			//ESP.wdtFeed();
			//dbg("\n*");
			scanAvailable = lidar2scan(&scan_msg);



			// Aggiorna i dati sul display ogni tot-----------------------------
			if (millis() > nextMsg_time + 1000)
			{
				BLINK(2);

				// ---spostato all'interno di displayFrame()----------------
				//if (GETBUTTON) /*negato perchè ritorna 0 se premuto*/

				//{

				//	displayCurrentFrameIndex += 1;
				//	if (displayCurrentFrameIndex >= DISPLAY_FRAMES) { displayCurrentFrameIndex = 0; }
				//	dbgD("Boot Pressed @" + String(millis()));
				//	
				//}

				displayFrame();
				nextMsg_time = millis();
			}
			//---------------------------------------------------------------------

		}
		dbgD("SCAN @" + String(millis()));


		scanAvailable = false;		// resetto

	}


#endif // 0

}





unsigned long nextMsg_time = 0;
long publishTime = 0;		/// tempo impiegato per pubblicare
long publishStartTime = 0;  ///inizio pubblicazione
long loopcnt = 0;
#define DISPLAY_SCAN_MAX_RAY 20 /*dimensione massima del cechio che rappresenta lo scan alla massima distanza*/
void displayScan() {
	float rescaledDist;
	const int center_x= 60;
	const int center_y = 35;
	int x1, y1;
	display.setColor(WHITE);
	float range2pixel = (float)DISPLAY_SCAN_MAX_RAY / 5; // 5 pixel per metro
//	dbg2("range2pixel:", range2pixel);
	for (int i = 0; i < 360; i++)
	{
		//for test
		//rescaledDist =min(50, 10*random(millis())) ;
		rescaledDist = range2pixel * scan_msg.ranges[i];// map(value, val_min,val_max, new_min,new_max)
		x1 = center_x + (int)round((cos((float)i*DEG_TO_RAD) * rescaledDist)) ;
		y1 = center_y + (int)round((sin((float)i*DEG_TO_RAD) * rescaledDist)) ;


		display.setPixel( x1,y1);  //non va in chash se x,y, sono fuori display

		//}
		
	}

	//rescaledDist = map(scan_msg.ranges[0], 0, 5, 0, 100);
	//display.drawProgressBar(1, 50, 120, 5, rescaledDist);

}



void loop() { // loop di test


	while (!scanAvailable)// attendo sulla seriale una scansione completa
	{
		scanAvailable = lidar2scan_debug(&scan_msg);

		/*//DEBUG ONLY
				if (SERIAL_LDS.available()>0)
				{
					Serial.println(SERIAL_LDS.read(), HEX); 
				}
		*/

		/// Aggiorna periodicamente i dati sul display ogni tot-----------------------------
		if (millis() > nextMsg_time + 1000)
		{
			//if ( GETBUTTON 	) /*negato perchè ritorna 0 se premuto*/
			// 
			//{
			//	displayCurrentFrameIndex += 1;
			//	if (displayCurrentFrameIndex >= DISPLAY_FRAMES) { displayCurrentFrameIndex = 0; }
			//	dbgD("Boot Pressed @"+ String(millis()));
			//	ROS_INFO("Boot btn Pressed");
			//}

			displayFrame();
			nextMsg_time = millis();
		}
		//---------------------------------------------------------------------

	} // ripeti finchè non ho una scansione completa
	BLINK(2);
	char dsplyTxt[20];	sprintf(dsplyTxt, "Scan @ %d ms", millis());
	scanAvailable = false;

	dbgD(dsplyTxt);
//	dbg(dsplyTxt);

	//pub_Laser.publish(&scan_msg);
	ROS_INFO(dsplyTxt);
}




