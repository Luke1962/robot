#include <SoftwareSerial.h>
#define SSID "ASUS"      //your wifi ssid here
#define PASS "cesarini"   //your wifi wep key here
#define DST_IP "192.168.0.3" //baidu.com
#define HOST "arduino.com"
 
//SoftwareSerial dbgSerial(11, 10); // RX, TX
 #define dbgSerial Serial1
char myChar;
 
void setup()
{
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
 
  //blink led13 to indicate power up
  for(int i = 0; i<15; i++)
  {
   digitalWrite(13,HIGH);
   delay(50);
   digitalWrite(13,LOW);
   delay(50);
  }
 
  // Open serial communications for WiFi module:
  dbgSerial.begin(9600);
  // Set time to wait for response strings to be found
  dbgSerial.setTimeout(5000);
 
  //Open software serial for debugging - must use serial to usb (FTDI) adapter
  //dbgSerial.begin(19200); //can't be faster than 19200 for softserial
  Serial.begin(38400);   //38400 softserial works fine for me
 
  Serial.println("ESP8266 Demo");
  delay(100);
 
  //test if the module is ready
  dbgSerial.println("AT+RST");    
  if(dbgSerial.find("ready"))
  {
    Serial.println("Module is ready");
    delay(1000);
    //connect to the wifi
    boolean connected=false;
    for(int i=0;i<5;i++)
    {
      if(connectWiFi())
      {
        connected = true;
        break;
      }
    }
    if (!connected)
    {
      //die
      while(1);
    }
 
    delay(5000);
    //set the single connection mode
    dbgSerial.println("AT+CIPMUX=0");
  }
  else
  {
    Serial.println("Module didn't respond.");
    delay(100);
   
    //serial loop mode for diag
    while(1){
      while (Serial.available()) {
        myChar = Serial.read();
        dbgSerial.print(myChar);
      }
 
      while (dbgSerial.available()) {
        myChar = dbgSerial.read();
        delay(25);
        Serial.print(myChar);
      }
    }
  }
}
 
void loop()
{
  String cmd = "AT+CIP\"TCP\",\"";
  cmd += DST_IP;
  cmd += "\",21";
  dbgSerial.println(cmd);
  Serial.println(cmd);
  if(dbgSerial.find("Error")) return;
  cmd = "Transmitting data well and clear! ";
  cmd += HOST;
  cmd += "\r\n\r\n";
 
  dbgSerial.print("AT+CIPSEND=");
  dbgSerial.println(cmd.length());
  if(dbgSerial.find(">"))
  {
    Serial.print(">");
  }
  else
  {
    dbgSerial.println("AT+CIPCLOSE");
    Serial.println("connect timeout");
    delay(1000);
    return;
  }
 
  dbgSerial.print(cmd);
 
  delay(500);
  //Serial.find("+IPD");
  while (dbgSerial.available())
  {
   char c = dbgSerial.read();
   Serial.write(c);
   if(c=='\r') Serial.print('\n');
   delay(50);
  }
 
 Serial.println();
 Serial.println("====");
 delay(1000);
}
 
 
boolean connectWiFi()
{
  dbgSerial.println("AT+CWMODE=1");
  String cmd="AT+CWJAP=\"";
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASS;
  cmd+="\"";
  Serial.println(cmd);
  dbgSerial.println(cmd);
  delay(2000);
  if(dbgSerial.find("OK"))
  {
    Serial.println("OK, Connected to WiFi.");
    return true;
  }
  else
  {
    Serial.println("Can not connect to the WiFi.");
    return false;
  }            
}