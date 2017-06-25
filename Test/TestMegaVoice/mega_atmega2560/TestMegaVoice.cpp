#define SPEECH_SERIAL Serial


#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	

#include <digitalWriteFast.h>
//#include <FrequencyTimer2\FrequencyTimer2.h>	
 

#include <SP0256-AL2\SP0256-AL2.h>
#include "arduino.h"

//
//
char SpeakBuffer[INPUTCHARARRAYSIZE];

 void setup()
{
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println("TestMegaVoice: test controllo speech module con Arduino Mega");
	#pragma region Configura Speaker
	
	SpeakerInit();
		
	SpeakerReset();
	 
	#pragma endregion Configura Speaker

 
	strcpy(SpeakBuffer, " Hello  arduino  sono  io  ");
	speakString(SpeakBuffer);
 }
#define T 1000

void loop()
{
	String s = Serial.readString();

	s.toCharArray(SpeakBuffer, INPUTCHARARRAYSIZE);
	speakString(SpeakBuffer);

}


