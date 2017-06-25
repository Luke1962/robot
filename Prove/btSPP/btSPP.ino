/*
 Keyboard Controller HID Example

 Shows the output of a USB Keyboard connected to the USB
 controller of an Arduino Due Board.

 created 8 Oct 2012
 by Cristian Maglie
 */

// Require keyboard control library
#include <KeyboardController.h>
#include <Usb.h>
#include "BTD.h"
#include "usbhub.h"
// Require keyboard control library
//#include <SPI.h>

#include "SPP.h";

#include "BTHID.h"

// Attach keyboard controller to USB
//-KeyboardController keyboard(usb);

/*
 Example for the RFCOMM/SPP Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */



//// Satisfy IDE, which only needs to see the include statement in the ino.
//#ifdef dobogusinclude
//#include <spi4teensy3.h>
//#endif



// Initialize USB Controller
USBHost Usb;

BTD Btd(&Usb); //WAS BTD Btd(&Usb); You have to create the Bluetooth Dongle instance like so
//SPP SerialBT(&Btd); // This will set the name to the defaults: "Arduino" and the pin to "0000"
SPP SerialBT(&Btd, "BTArduino", "1234"); // You can also set the name and pin like so

boolean firstMessage = true;
	

void setup()
{ 
/* You can create the instance of the class in two ways */


	Serial.begin(115200);
  Serial.println("Program started");
  delay(200);

  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  //if (usb.init() == -1) //WAS   if (Usb.Init() == -1) { ARDUINO DUE MOD
  //{
  //  Serial.print(F("\r\nOSC did not start"));
  //  while (1); //halt
  //}
  Serial.println(F("\r\nSPP Bluetooth Library Started"));

}

void loop(){
 Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (SerialBT.connected) {
    if (firstMessage) {
      firstMessage = false;
      SerialBT.println(F("Hello from Arduino")); // Send welcome message
    }
    if (Serial.available())
      SerialBT.write(Serial.read());
    if (SerialBT.available())
      Serial.write(SerialBT.read());
  }
  else
    firstMessage = true;
    
}
