// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
#define ROS_TCP_CONNECTION_PORT 11411

#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
//#include <Servo.h>

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


/*
Servo s;
int i;

void chatterCallback(const std_msgs::String& msg) {
  i = atoi(msg.data);
  s.write(i);
}

ros::Subscriber<std_msgs::String> sub("message", &chatterCallback);
*/
ros::NodeHandle_<WiFiHardware> nh;

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
		digitalWrite(LED_BUILTIN, 1);		delay(200);
		digitalWrite(LED_BUILTIN, 0); 		delay(200);
	}
  }
  else //ok connesso
  {
		// led acceso per 2 secondi
		digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
		digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
		digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
		digitalWrite(LED_BUILTIN, 1);	
		//Serial.print("Ready! Use ");
		//Serial.print(WiFi.localIP());
		//Serial.println(" to access client");
		//Serial.print(1); //ok connected
  }
}
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
void setup() {
	Serial.begin(57600);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, 0);

	setupWiFi();

 	nh.initNode();
 }

char strVal[10];
int cnt = 0;



uint8_t buf1[1024];
uint8_t i1 = 0;

uint8_t buf2[1024];
uint8_t i2 = 0;


void loop() {
   // here we have a connected client
 	if (client.available()) {
		while (client.available()) {
			buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
			if (i1<1023) i1++;
			//digitalWrite(LED_BUILTIN, 1);
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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
