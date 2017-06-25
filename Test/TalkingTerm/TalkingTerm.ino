// TalkingTerm.pde / TalkingTerm.ino - Talking terminal demo for SpeakJet Shield TTS
// written by Galen Raben / www.droidbuider.com
// original sketch written 8-1-2009, last modified 5-15-2010

// Demonstrates "serial pass-thru" where text you type at your computer
// is passed thru the Arduino and spoken by the SpeakJet Shield TTS.

// set up a new serial port for Speakjet Shield
// rxPin: the pin on which to receive serial data from TTS256
// txPin: the pin on which to transmit serial data to the TTS256
#include <SoftwareSerial.h>
#define txPin 2
#define rxPin 3
#define busyPin 4

// set the data rate for the Speakjet Shield SoftwareSerial port
SoftwareSerial speakJet =  SoftwareSerial(rxPin, txPin);

char sayThis[256]; // string (char array) that holds bytes of incoming string

// read a string from the serial and store it in an array
// this bit of code adapted from WilsonSerialIO.pde 
// http://userwww.sfsu.edu/~swilson/
void readSerialString (char *strArray) {
 int i = 0;
 if(Serial.available()) {    
   //Serial.print("reading Serial String: "); //optional: for confirmation
   while (Serial.available()){            
     strArray[i] = Serial.read();
     i++;
     Serial.print(strArray[(i-1)]); // for confirmation
   }
   strArray[i] = '\0'; // append a proper null to end of string
   Serial.println(); // for confirmation
 }
}

void setup(){
 // initialize the serial communications with PC:
 Serial.begin(9600);
 // initialize the serial communications with the SpeakJet-TTS256
 pinMode(rxPin, INPUT);
 pinMode(txPin, OUTPUT);
 speakJet.begin(9600);
 // make sure we are ready to start talking!
 delay(200); // this delay minimizes random spoken text in TTS256 buffer
 speakJet.println("Talking term ready");
 SJBusy(); // wait for speakjet buffer to empty
 sayThis[0] = '\0'; // clear speakjet buffer
}

void loop()
{
 // get string sent from PC
 readSerialString(sayThis);
 // speak it!
 speakJet.println(sayThis);
 SJBusy(); // wait for speakjet buffer to empty
 sayThis[0] = '\0'; // clear speakjet buffer before getting next string
}

void SJBusy(){
 delay(20); // wait 12ms minimum before checking SpeakJet busy pin
 while(digitalRead(busyPin)){
   delay(250); // wait here while SpeakJet is busy (pin 4 is true)
 }
 delay(250); // a bit more delay after busyPin goes false
}
