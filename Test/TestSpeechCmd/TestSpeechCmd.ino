/*
test del modulo voce via seriale
*/
#include "Arduino.h"
#include <digitalWriteFast\digitalWriteFast.h>
#define ser Serial3
#define PIN_SPEECH_BUSY 11

void setup()
{
	ser.begin(9600);
	Serial.begin(115200);
	Serial.println("TestSpeechCmd");
  /* add setup code here */
	pinMode(13, OUTPUT);//LED
	pinMode(PIN_SPEECH_BUSY, INPUT);

}
bool speechIsBusy() {
	return (bool)digitalReadFast(PIN_SPEECH_BUSY);
}
 void speech(String  wrd) {
	//attendo il carattere di pronto 
	while (speechIsBusy()) {
		Serial.print(" busy");
		delay(500);
	}
	Serial.print("speech ..");
	Serial.println(wrd);
	ser.println(wrd);   // send

}
void loop()
{
	digitalWrite(13, !digitalRead(13));
	speech("uno ");
	speech("due ");
	speech("tre ");

	delay(500);

}
  /* add main program code here */


/**
*
* Author: Domenico Monaco - hack@kiuz.it
*
* Description:
*   Simple Handshake Three Way with Arduino over Serial communication
*
* License: GNU v2 2014
*


int SYNIN = 0;
int SYNOUT = 0;
bool SYN_CHECK = false;
String SYNID = "SYN";

bool SYN_RECIVED = false;

int ACKIN = 0;
int ACKOUT = 0;
bool ACK_CHECK = false;
String ACKID = "ACK";
bool ACK_RECIVED = false;

String MESIN = "";
String MESOUT = "";
bool MES_CHECK = false;
String MESID = "MES";
bool MES_RECIVED = false;

int incomingByte = 0;    // for incoming serial data

long int timeStartCom = 0;
long int timeEndCom = 0;

long int charsIN = 0;
bool waiting = false;

String line = "";

void setup() {
	Serial.begin(115200);
}

void loop() {
	if (SYN_RECIVED == true && ACK_RECIVED == true && MES_RECIVED == true) {
		readLine();
	}
	else {
		readLine();
		handeShake();
	}
}

void handeShake() {

	if (line.length() != 0 && waiting == true) {
		int n_SYN = line.indexOf("SYN");
		//Serial.println(n_SYN);

		int n_ACK = line.indexOf("ACK");
		//Serial.println(n_ACK);

		int n_MES = line.indexOf("MES");
		//Serial.println(n_MES);   

		int n_end = line.indexOf(";");
		//Serial.println(n_end);

		SYN_CHECK = false;

		if (n_SYN == 0 && n_ACK == -1 && n_MES == -1 && n_end == 7) {
			Serial.println("Detected SYN message");

			check_SYN();

			if (SYN_CHECK == true) {

				SYNIN = line.substring(4, 7).toInt();
				SYNOUT = SYNIN + 1;

				Serial.println("Checked valid Syn;");

				SYN_RECIVED = true;

				ACKOUT = random(100, 900);

				String output = SYNID + " " + SYNOUT + " " + ACKID + " " + ACKOUT + ";";
				Serial.println(output);

			}
			else {
				SYN_RECIVED = false;
				Serial.print("Checked not valid Syn;");
			}


		}
		else if (n_SYN == 0 && n_ACK == 8 && n_MES == -1 && n_end == 15 && check_SYN() == true && SYN_RECIVED == true) {

			Serial.println("Detected ACK message");

			check_ACK();

			if (ACK_CHECK == true) {
				Serial.println("Checked valid ACK;");
				ACK_RECIVED = true;

				MESOUT = "OK";

				String output = SYNID + " " + SYNOUT + "," + ACKID + " " + ACKIN + "," + MESID + " " + MESOUT + ";";
				Serial.println(output);

			}
			else {
				ACK_RECIVED = false;
				SYN_RECIVED = false;
				Serial.print("Checked not valid ACK;");
			}


		}
		else if (n_SYN == 0 && n_ACK == 8 && n_MES == 16 && n_end>20 && check_ACK() == true && ACK_RECIVED == true) {

			Serial.println("Detected MES message");
			check_MES();

			if (MES_CHECK == true) {
				Serial.println("Checked valid MES;");
				MES_RECIVED = true;

				Serial.println("Are you allowed to communicate with me! ");
			}
			else {
				Serial.println("Checked not valid MES;");
			}

		}
		else {
			Serial.println("Not valid message.");
		}
	}
}


void readLine() {

	// send data only when you receive data:
	if (Serial.available() > 0) {



		if (charsIN == (long int)0) {

			//line = "";

			timeStartCom = millis();
			timeEndCom = 0;

			waiting = false;
		}
		charsIN++;

		incomingByte = Serial.read();
		line = line + (char)incomingByte;

		delay(1); //Delay of Serial transfer
	}
	else {

		if (waiting == false) {
			waiting = true;

			timeEndCom = millis();

			///PRINT PREVUOUS MESSAGE 
			Serial.print("Message: ");
			Serial.println(line);

			Serial.print("Recived ");
			Serial.print(charsIN);
			Serial.print("chars in ");
			Serial.print(timeEndCom - timeStartCom);
			Serial.print("millis; ");
			Serial.print((float)(timeEndCom - timeStartCom) / charsIN);
			Serial.print(" mills for chars;");
			Serial.print(" ");
			Serial.print((float)charsIN / (timeEndCom - timeStartCom));
			Serial.println("char/millis;");


			timeStartCom = 0;
			charsIN = (long int)0;

			Serial.println("");
			Serial.println("Waiting for message...");
			Serial.println("");
		}
		else {

			//in a seconde cicle reset line
			line = "";
		}
	}
}


bool check_SYN() {
	for (int i = 4; i <= 6; i++) {
		if (isDigit(line.charAt(i)) == true) {
			SYN_CHECK = true;
		}
		else {
			SYN_CHECK = false;
			break;
		}
	}
	return SYN_CHECK;
}

bool check_ACK() {
	for (int i = 12; i <= 14; i++) {
		if (isDigit(line.charAt(i)) == true) {
			ACK_CHECK = true;
		}
		else {
			ACK_CHECK = false;
			break;
		}
	}
	if (ACK_CHECK == true) {
		SYNIN = line.substring(4, 7).toInt();
		ACKIN = line.substring(12, 15).toInt();

		if (SYNIN != (SYNOUT) && ACKOUT != (ACKIN - 1)) {
			ACK_CHECK = false;
		}
		else if (SYNIN == (SYNOUT) && ACKOUT == (ACKIN - 1)) {
			ACK_CHECK = true;
		}
		else {
			ACK_CHECK = false;
		}
	}
	return ACK_CHECK;
}

bool check_MES() {

	SYNIN = line.substring(4, 7).toInt();
	ACKIN = line.substring(12, 15).toInt();
	MESIN = line.substring(20, line.length() - 1);

	if (SYNIN != (SYNOUT) && ACKOUT != (ACKIN - 1) && MESIN != MESOUT) {
		MES_CHECK = false;
	}
	else if (SYNIN == (SYNOUT) && ACKOUT == (ACKIN - 1) && MESIN == MESOUT) {
		MES_CHECK = true;
	}
	else {
		MES_CHECK = false;
	}

	return MES_CHECK;
}*/