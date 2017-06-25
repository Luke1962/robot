/*
  Serial Call and Response
 Language: Wiring/Arduino
*/
#define SerialIN Serial2
#define SerialOUT Serial
#include "arduino.h"

//
//
int inByte = 0;         // incoming serial byte

void setup()
{
	SerialIN.begin(9600);
  // start serial port at 9600 bps:
	SerialOUT.begin(9600);
	SerialOUT.println( "Echo su seriale dei dati ricevuti su Serial2" );


}

void loop()
{

	if (SerialIN.available() > 0) {
		// get incoming byte:
		inByte = SerialIN.read();
     
		if (inByte<255)
		{
			SerialOUT.print("[");
			SerialOUT.print(inByte);
			SerialOUT.println("]");

		} else
		{
			//Serial1.write(inByte);
			SerialOUT.write(inByte);
		}
	}    
	if (SerialOUT.available() > 0) {
	// get incoming byte:
	inByte = SerialOUT.read();
     
	// Serial.write(inByte);
	SerialIN.write(inByte);
	}
}





