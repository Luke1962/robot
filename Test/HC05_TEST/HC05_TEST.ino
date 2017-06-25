// ================================================================
// ===               BLUETOOTH MODULE HC06		                ===
// ================================================================
#include <Arduino.h>
#include <HC05.h>
#define BT_ENABLEPIN 42
#define BT_STATEPIN 40
#define LED_CMDBT 13
#define BT_BAUD  115200
// default Passkey: “1234”

HC05 btserial = HC05(BT_ENABLEPIN, BT_STATEPIN);  // cmd, state

void setup()
{
  pinMode(LED_CMDBT, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
//  digitalWrite(BT_ENABLEPIN, HIGH); //attiva HC05 pin  KEY
   digitalWrite(LED_CMDBT, LOW);
  
  Serial.begin(BT_BAUD);

   Serial.println("Enter  cmd before pairing :");
//    btserial.setBaud(BT_BAUD); //default is 9600
  Serial.println("Waiting BT is connected..."); 

   unsigned long B = btserial.findBaud();
// btserial.getStatus();
  digitalWrite(BT_ENABLEPIN, HIGH);
Serial1.write("AT\r\n");

delay(500);
 Serial.write(Serial1.read());

 while (!btserial.connected());
  
  Serial.println("OK BT Connected...Entering loop");
 digitalWrite(LED_CMDBT, HIGH);
}

void loop()
{

  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (btserial.available())
    Serial.write(btserial.read());

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
  
	  btserial.write(Serial.read());
    
}





