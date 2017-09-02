// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
// WIFI: porta 11411, connessione SERIALE: porta 11311 
#define ROS_TCP_CONNECTION_PORT 11411
#define ROS_BRIDGE_CONNECTION_PORT 9090

// PORTA ALLA QUALE SI CONNETTE IL WIFI
#define TCP_PORT ROS_BRIDGE_CONNECTION_PORT

#define SERIAL_SPEED 57600
#define DEBUG_ON 1

//#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Float64.h>
//#include <Servo.h>






#define LED_ON digitalWrite(LED_BUILTIN, 0);
#define LED_OFF digitalWrite(LED_BUILTIN, 1);
#define LED_PULSE LED_ON;delay(100);LED_OFF ;delay(300);



/*
#include <ESP8266WiFi.h>

//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = "FASTWEB-CSRLCU";
const char* password = "cesarini";


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
	for(int i=0; i<length; i++)
	  client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
	 return millis(); // easy; did this one for you
  }
};



//ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi()
{
  WiFi.begin(ssid, password);
//  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
	//Serial.print("Could not connect to"); Serial.println(ssid);
	  Serial.print(-1); // segnala fail 
	while (1) {
		LED_ON		delay(200);
		LED_OFF		delay(200);
	}
  }
  else //ok connesso
  {
		// led acceso per 2 secondi
	  LED_ON	  delay(200); 	  LED_OFF
	  LED_ON	  delay(200); 	  LED_OFF
	  LED_ON	  delay(200); 	  LED_OFF
		//Serial.print("Ready! Use ");
		//Serial.print(WiFi.localIP());
		//Serial.println(" to access client");
		//Serial.print(1); //ok connected
  }
}

*/


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
#define dbgV(__VA_ARGS__) Serial.printf(__VA_ARGS__)

#else
#define dbg(s) 
#define dbg2(s,v)
#endif

void ledSeq1(int ms, int times = 3) {
	pinMode(LED_BUILTIN, OUTPUT);
	for (int i = 0; i < times; i++)
	{
		digitalWrite(LED_BUILTIN, 0); delay(ms);
		digitalWrite(LED_BUILTIN, 1); delay(ms);

	}

}
#pragma endregion


#pragma region WIFI

//////////////////////
// WiFi Definitions //
//////////////////////

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
const char* ssid = "FASTWEB-CSRLCU";
const char* password = "cesarini";


IPAddress server(192, 168, 0, 51); // ip of your ROS server
								   //IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

/*

class WiFiHardware {

public:
	WiFiHardware() {};

	void init() {
		// do your initialization here. this probably includes TCP server/client setup
		client.connect(server, TCP_PORT);
		dbg2("Client.connect to port", TCP_PORT);
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
*/
void setup_WiFi()
{

	WiFi.mode(WIFI_STA);


	String mac = WiFi.macAddress();
	dbg2("MAC ADDR:", mac);
	dbg2("\nConnecting to ", ssid);
	WiFi.begin(ssid, password);

	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
	if (i == 21) {

		dbg2("Could not connect to: ", ssid);

		while (1) {
			// LED flash
			ledSeq1(200, 10);
			delay(1000);

		}
	}
	else //ok connesso
	{
		dbg2("Ready to use  IP: ", WiFi.localIP());
		ledSeq1(500, 3);


		//debug only
		//Serial.println("WiFi Connected"); Serial.println(ssid);
		//Serial.print("Ready! Use ");
		//Serial.print(WiFi.localIP());
		//Serial.println(" to access client");
		//Serial.print(1); //ok connected

	}
}

#pragma endregion







void setup() {
	Serial.begin(SERIAL_SPEED);
	pinMode(LED_BUILTIN, OUTPUT);
	LED_OFF


	setup_WiFi();


	// connessione TCP
	if (!client.connect(server, TCP_PORT)) {
		Serial.println("client connection failed");
		return;
	}

 }

char strVal[10];
int cnt = 0;



uint8_t buf1[1024];
uint8_t i1 = 0;

uint8_t buf2[1024];
uint8_t i2 = 0;
uint16_t lastTimeWifiCheck = 0;
void hbWiFi() {
	if (millis()-lastTimeWifiCheck > 5000)
	{

		if (WiFi.isConnected())
		{
			LED_PULSE
		}
		else
		{
			dbg("Lost WiFi connection!!")
			setup_WiFi();
		}
		lastTimeWifiCheck = millis();
	}

}

void loop() {
	if (client.connected())
	{
	   // here we have a connected client
		if (client.available()) {
			while (client.available()) {
				buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
				if (i1<1023) i1++;
				//digitalWrite(LED_BUILTIN, 1);
				//digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			}
			// now send to UART:
			Serial.write(buf1, i1);
			i1 = 0;
		}
		if (Serial.available()) {
			while (Serial.available()) {
				buf2[i2] = (char)Serial.read(); // read char from UART
				if (i2<1023) i2++;
				digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
				//digitalWrite(LED_BUILTIN, 0);
			}
			// now send to WiFi:
			client.write((char*)buf2, i2);
			i2 = 0;
		}

	}
	else
	{
		hbWiFi();

	}
}
