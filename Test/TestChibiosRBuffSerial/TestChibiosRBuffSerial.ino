/*
*  Manage strings by using a generic, dynamic queue data structure.
*
*  Copyright (C) 2010  Efstathios Chatzikyriakidis (contact@efxa.org)
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#define SERIAL_OUT Serial
#define SERIAL_IN Serial1

//////////////////////////////////////////////////////////////////////////////////
// LIBRERIE                                    ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <ChibiOS_AVR.h>
#include <ChibiOS_AVR/utility/chstreams.h>

#include <digitalWriteFast.h>
#include <buzzer\buzzer.h>
#define DEBUG_ON
#include <MyRobotLibs\dbg.h>
#include  <MyRobotLibs\stringlib.h> //Funzioni di manipolazione stringhe di tipo CHARARRAY

#include <SoftwareSerial.h>
//SoftwareSerial SwSerialSpeech(42, Pin_SpeechSerialTX); // RX, TX
#include <MyRobotLibs\SpeakSerialInterface.h>

//#include <QueueList\QueueList.h>
#include <TinyGPSplus\TinyGPS++.h>	//il linker s'incazza se non c'è
//////////////////////////////////////////////////////////////////////////////////
// PARAMETRIZZAZIONE DEL SISTEMA               ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



//#define ROBOT_SERIAL Serial			// riceve i comandi dal modulo robot
//#define ROBOT_SERIAL_BAUD_RATE 9600
//////////////////////////////////////////////////////////////////////////////////
//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//char SpeakBuffer[INPUTCHARARRAYSIZE];
volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

// Fifo definitions-------------------------
#pragma region FiFoSpeech setup
#define MSGSTRING_SIZE 50 //dimensione di ciascuna stringa voce
#define RING_BUFFER_SIZE 3 //quante stringe possono essere in coda
#include <RingBuf/RingBuf.h>
struct myMsgStruct
{
	char* msgStr=msgString;  // usato da thdSerialIn2FifoB
	int index;
	char msgString[MSGSTRING_SIZE]; // usato da thdSerialIn2Fifo
	int msgSize;
	unsigned long long timestamp;
};



// Create a RinBuf object designed to hold  10 of mystructs
RingBuf *myRingBuf = RingBuf_new(sizeof(struct myMsgStruct), RING_BUFFER_SIZE);


#pragma endregion


//------------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS  /////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


#pragma region // BLINK LED
#define PIN_LED  13

// ///////////////////////////////////////////////////////////////////////////////
//  blinking LED       ///////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////

// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waFlashLed, 64);
static THD_FUNCTION(thdFlashLed, arg) {
	// Flash led every 200 ms.
	pinMode(PIN_LED, OUTPUT);		digitalWrite(PIN_LED, 0);	// led superiore

	while (1) {
		// Turn LED on.
		digitalWriteFast(PIN_LED, HIGH);
		// Sleep for 50 milliseconds.
		chThdSleepMilliseconds(40);

		// Turn LED off.
		digitalWriteFast(PIN_LED, LOW);

		// Sleep for 150 milliseconds.
		chThdSleepMilliseconds(960);
	}
}
#pragma endregion // BLINK LED----------------------------------------------------

/// ///////////////////////////////////////////////////////////////////////////////
// ALIMENTA LA FIFO IN AUTOMATICO (TEST)   /////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo di test: Alimenta la FIFO ogni X secondi
static THD_WORKING_AREA(waFeedFifo, 64);
static THD_FUNCTION(thdFeedFifo, arg) {
	// definizione di Fifohead spostato a livello globale in quanto usata da altri thread
	dbg("thdFeedFifo started")


	char tmpBuff[MSGSTRING_SIZE];


	// Create element I want to add
	struct myMsgStruct msgOutVoice;

	// alloca la memoria per msgOutVoice 
	memset(&msgOutVoice, 0, sizeof(struct myMsgStruct));



	while (true)
	{
		//TEST creo il testo del messaggio in buf[] (millis. in formato stringa)
		ltoa(millis(), tmpBuff, MSGSTRING_SIZE);

		playSingleNote(NOTE_A7, 80);


		if (!myRingBuf->isFull(myRingBuf))
		{
			// metto il messaggio in ring_buf----------
 			for (size_t i = 0; i < 10; i++)
			{
				msgOutVoice.msgString[i] = tmpBuff[i];
			}
			//-----------------------------------------


			myRingBuf->add(myRingBuf, &msgOutVoice);
			//-----------------------------------------

			dbg2("msgString out :", msgOutVoice.msgString)
				dbg2("myRingBuf->elem:", myRingBuf->elements)

				chThdSleepMilliseconds(500);//	chThdYield();

		}
		else //full
		{
			dbg2("myRingBuf->elem:", myRingBuf->elements)
				dbg("........myRingBuf full")
				//dbg2("........FreeSram", getFreeSram())
				//attendo più a lungo per permettere di svuotare il buffer
				chThdSleepMilliseconds(4000);//	chThdYield();
		}








	}

}

#pragma endregion 


#if 0
/// ///////////////////////////////////////////////////////////////////////////////
// ALIMENTA LA FIFO DA SERIALE /////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo di test: Alimenta la FIFO da seriale usando char* anzichè array di caratteri
static THD_WORKING_AREA(waSerialIn2FifoB, 64);
static THD_FUNCTION(thdSerialIn2FifoB, arg) {
	// definizione di Fifohead spostato a livello globale in quanto usata da altri thread
	dbg("thdSerialIn2Fifo started")

		char* tmpBuff;

	// Create element I want to add
	struct myMsgStruct msgOutVoice;

	// alloca la memoria per msgOutVoice 
	memset(&msgOutVoice, 0, sizeof(struct myMsgStruct));

	int inWaiting = 0; //caratteri in attesa nella seriale

	while (true)
	{
		inWaiting = SERIAL_IN.available();
		if (inWaiting > 0)
		{

			// limito la lettura alla dimensione del messaggio msgOutVoice.msgString[]
			if (inWaiting > MSGSTRING_SIZE) { inWaiting = MSGSTRING_SIZE; };


			if (!myRingBuf->isFull(myRingBuf))
			{
				//playSingleNote(NOTE_A7, 80);

				// metto il messaggio in ring_buf ----------
				SERIAL_IN.readBytesUntil('\n', msgOutVoice.msgStr, MSGSTRING_SIZE);

				// così non funziona!! -> strcpy(tmpBuff, msgOutVoice.msgString);
				//-----------------------------------------
				// metto il messaggio nella FIFO ----------
				myRingBuf->add(myRingBuf, &msgOutVoice);
				//-----------------------------------------

				//dbg2("msgString out :", msgOutVoice.msgString)
				dbg2("\n\t myRingBuf->elem+:", myRingBuf->elements)

					chThdSleepMilliseconds(500);//	chThdYield();

			}
			else //full
			{
				dbg2("\t myRingBuf full :", myRingBuf->elements)
					//attendo più a lungo per permettere di svuotare il buffer
					chThdSleepMilliseconds(4000);//	chThdYield();
			}

		}
	}

}

#pragma endregion   
#endif // 0


/// ///////////////////////////////////////////////////////////////////////////////
// ALIMENTA LA FIFO DA SERIALE /////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo di test: Alimenta la FIFO da seriale
static THD_WORKING_AREA(waSerialIn2Fifo, 64);
static THD_FUNCTION(thdSerialIn2Fifo, arg) {
	// definizione di Fifohead spostato a livello globale in quanto usata da altri thread
	
	char tmpBuff[MSGSTRING_SIZE];

	// Create element I want to add
	struct myMsgStruct msgOutVoice;

	// alloca la memoria per msgOutVoice 
	memset(&msgOutVoice, 0, sizeof(struct myMsgStruct));

	int inWaiting = 0; //caratteri in attesa nella seriale


	while (true)
	{
		inWaiting = SERIAL_IN.available();
		if (inWaiting>0)
		{

			// limito la lettura alla dimensione del messaggio msgOutVoice.msgString[]
			if (inWaiting > MSGSTRING_SIZE){
				playSingleNote(NOTE_A3, 60);

				inWaiting = MSGSTRING_SIZE;			};


			if (!myRingBuf->isFull(myRingBuf))
			{
				playSingleNote(NOTE_A7, 80);

				// metto il messaggio in ring_buf (funziona) ----------
				for (int i = 0; i < inWaiting; i++)
				{
					msgOutVoice.msgString[i] = SERIAL_IN.read(); 
					if (msgOutVoice.msgString[i] == ';')
					{
						inWaiting = i+1;
						break;

					}
				}
				/*msgOutVoice.msgString[inWaiting] = '0';*/
				msgOutVoice.msgSize = inWaiting;
				//-----------------------------------------
 				// metto il messaggio nella FIFO ----------
				myRingBuf->add(myRingBuf, &msgOutVoice);
				//-----------------------------------------

				//dbg2("msgString out :", msgOutVoice.msgString)
	 
 				dbg2("                                                      Buf+:", myRingBuf->elements)

				chThdSleepMilliseconds(500);//	chThdYield();

			}
			else //full
			{
				playSingleNote(NOTE_A2, 100);
 				dbg2("                                                      myRingBuf full :", myRingBuf->elements)
 				//attendo più a lungo per permettere di svuotare il buffer
				chThdSleepMilliseconds(4000);//	chThdYield();
			}

		}
	}

}

#pragma endregion 

/// ///////////////////////////////////////////////////////////////////////////////
// FiFo ->  Speech     ///////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo: FiFo -> SERIAL_OUT   (ex Speech)
// il formato stringhe ricevuto via Blutetooth è *...# dove ... è il testo inviato
static THD_WORKING_AREA(waFifoToSerialOut, 64);
static THD_FUNCTION(thdFifoToSerialOut, arg) {

 
	struct myMsgStruct  myVoiceMsg;// variabile di appoggio per estarre i messaggi da Buffer
	dbg("End Setup thdFifoToSerialOut")

	while (1) {

		if (myRingBuf->elements > 0)
		{
			dbg2("                                                      Buf-:", myRingBuf->elements)
			// estrai in myVoiceMsg il messaggio (should be same as msgOutVoice) 
			myRingBuf->pull(myRingBuf, &myVoiceMsg);
 
			//uno dei due in base al thread che alimenta la fifo
			//SERIAL_OUT.print(myVoiceMsg.msgString);
			for (size_t i = 0; i < myVoiceMsg.msgSize; i++)
			{
				SERIAL_OUT.write(myVoiceMsg.msgString[i]);
			}
			//SERIAL_OUT.print(myVoiceMsg.msgStr);
		}

		if (myRingBuf->elements >8)
		{
			chThdSleepMilliseconds(200);//	chThdYield();//	

		}
		else
		{
			chThdSleepMilliseconds(2000);//	chThdYield();//	

		}

	}
}

#pragma endregion



// FINE PROCESSI CHIBIOS ////////////////////////////////////////////////////////////////////////////////

#pragma region [CHIBIOS RTOS]

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;



//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void chSetup() {
	// fill pool with PoolObject array

	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 10, thdFlashLed, NULL);
	chThdCreateStatic(waFifoToSerialOut, sizeof(waFifoToSerialOut), NORMALPRIO + 2, thdFifoToSerialOut, NULL);
	dbg("started thdFifoToSerialOut...")
	//	chThdCreateStatic(waFeedFifo, sizeof(waFeedFifo), NORMALPRIO + 2, thdFeedFifo, NULL);
	//dbg("started	thdFeedFifo")
		chThdCreateStatic(waSerialIn2Fifo, sizeof(waSerialIn2Fifo), NORMALPRIO + 2, thdSerialIn2Fifo, NULL);
	dbg("started thdSerialIn2Fifo...")
		//chThdCreateStatic(waSerialManager, sizeof(waSerialManager), NORMALPRIO + 2, SerialManager, NULL);
		//chThdCreateStatic(waSerialPcToFifoVoice, sizeof(waSerialPcToFifoVoice), NORMALPRIO + 2, SerialPcToFifoVoice, NULL);

}
void setup()
{
	SERIAL_IN.begin(115200);
	SERIAL_OUT.begin(115200);

	SERIAL_OUT.println("TestChibios");
	SERIAL_OUT.setTimeout(5000);


#pragma region RingBuf Setup
	if (!myRingBuf)
	{
		Serial.println("Not enough memory SYSTEM HALT!");
		while (1);
	}
	//// Create element I want to add
	//struct myMsgStruct msgTmp;

	//// Zero both of them out
	//memset(&msgTmp, 0, sizeof(struct myMsgStruct));
	////memset(&myVoiceMsg, 0, sizeof(struct myMsgStruct));

	//msgTmp.index = 1;
	//msgTmp.timestamp = millis();

	//// Copy msgOutVoice into the ring_buf
	//myRingBuf->add(myRingBuf, &msgTmp);

#pragma endregion



#pragma region test swSerial

#if 0
	pinMode(Pin_SpeechSerialBusy, INPUT);
	for (size_t i = 0; i < 3; i++)
	{

		SERIAL_OUT.print("io ");
		delay(700);
	}

#endif // 0

#pragma endregion

	
	chBegin(chSetup);	while (1) {}
}
void loop() {
	// not used
}

#pragma endregion
