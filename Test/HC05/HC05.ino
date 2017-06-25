/*
 * echo - echo characters back to bluetooth device
 *
 * Waits for a connection and then echos each charater received.
 *
 * Debugging is enabled, so if you open the 'Serial Monitor' you can see
 * the search for the HC05 baud and the wait for the BT connection.
 */
#include <Arduino.h>
#include "HC05.h"

HC05 btSerial = HC05(7, 6);  // cmd, state

void setup()
{
//  DEBUG_BEGIN(115200);
  btSerial.findBaud();
  Serial.begin(115200);
  Serial.println("Echo Server- type something");

}

void loop()
{
    // if we get a valid byte, read analog ins:
  if (Serial.available() > 0)
  {

    if (btSerial.connected())
    {
        btSerial.write(Serial.read());

    }
    else 
    {
        Serial.write("BT non connected");
    }
  }
   else 
    {
        Serial.write("Serial not available");
    }
  
}



