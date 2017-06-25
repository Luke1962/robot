// ESP8266 WiFi <-> UART Bridge
// from  RoboRemo // www.roboremo.com


// config: ///////////////////////////////////////

#define UART_BAUD 115200
#pragma region WiFi-------------------------------------------
	// ESP WiFi mode:
	#include <ESP8266WiFi.h>
	#include <WiFiClient.h>
   
 		const char *ssid = "FASTWEB-CSRLCU";  // Your ROUTER SSID
		const char *pw = "cesarini"; // and WiFi PASSWORD
		const int port = 11411;
		// You must connect the phone to the same router,
		// Then somehow find the IP that the ESP got from router, then:
		// menu -> connect -> Internet(TCP) -> [ESP_IP]:9876
 
 	IPAddress rosServer(192, 168, 0, 51); // ip of your ROS rosServer
//	IPAddress ip_address;
	int status = WL_IDLE_STATUS;

	WiFiClient client;

	class WiFiHardware {

	public:
		WiFiHardware() {};

		void init() {
			// do your initialization here. this probably includes TCP rosServer/client setup
			client.connect(rosServer, port);
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

#pragma endregion









//////////////////////////////////////////////////


 

 

uint8_t buf1[1024];
uint8_t i1=0;

uint8_t buf2[1024];
uint8_t i2=0;

void setupWiFi()
{
	WiFi.begin(ssid, pw);
	Serial.print("\nConnecting to "); Serial.println(ssid);
	uint8_t i = 0;
	while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
	if (i == 21) {
		Serial.print("Could not connect to"); Serial.println(ssid);
		while (1) delay(500);
	}
	Serial.print("Ready! Use ");
	Serial.print(WiFi.localIP());
	Serial.println(" to access client");

	//todo restituire un codice di ok sulla seriale
 


 

}

#pragma region ROS
//--mia aggiunta ROS ------------------------------
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

ros::NodeHandle_<WiFiHardware> nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void publish_chatter(char* charVal) {


	//str_msg.data = strcat("ultra sound:", charVal);
	str_msg.data = charVal;
	chatter.publish(&str_msg);
	nh.spinOnce();

}



#pragma endregion

//-------------------------------------------------
void setup() {
	Serial.begin(UART_BAUD);
	setupWiFi();

	delay(2000);

	nh.initNode();

	nh.advertise(chatter);
	publish_chatter("ESPwifi connected");

	delay(500);
  

}


void loop() {

 

  // here we have a connected client

  if(client.available()) {
	while(client.available()) {
	  buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
	  if(i1<1023) i1++;
	}
	// now send to UART:
	Serial.write(buf1, i1);
	i1 = 0;
  }

  if(Serial.available()) {
	while(Serial.available()) {
	  buf2[i2] = (char)Serial.read(); // read char from UART
	  if(i2<1023) i2++;
	}
	// now send to WiFi:
	client.write((char*)buf2, i2);
	i2 = 0;
  }
  
}

