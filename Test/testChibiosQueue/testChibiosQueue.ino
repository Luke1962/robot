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



#define ROBOT_SERIAL Serial			// riceve i comandi dal modulo robot
#define ROBOT_SERIAL_BAUD_RATE 9600
//////////////////////////////////////////////////////////////////////////////////
//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//char SpeakBuffer[INPUTCHARARRAYSIZE];
volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

// Fifo definitions-------------------------
#pragma region FiFoSpeech setup
#define MAX_STRING_SIZE_VOICE 10 //dimensione di ciascuna stringa voce
#define MAX_BUFFER_SIZE_VOICE 10 //quante stringe possono essere in coda
#include <RingBuf/RingBuf.h>
struct myMsgStruct
{
	int index;
	char msgString[MAX_STRING_SIZE_VOICE];
	unsigned int mycrap;
	unsigned long long timestamp;
};
//struct myMsgStruct
//{
//		char msgString[MAX_STRING_SIZE_VOICE];
//};


// Create a RinBuf object designed to hold  10 of mystructs
RingBuf *myRingBuf = RingBuf_new(sizeof(struct myMsgStruct), MAX_BUFFER_SIZE_VOICE);


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
// ALIMENTA LA FIFO  (TEST)   /////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////

#pragma region Processo di test: Alimenta la FIFO ogni X secondi
static THD_WORKING_AREA(waFeedFifo, 64);
static THD_FUNCTION(thdFeedFifo, arg) {
	// definizione di Fifohead spostato a livello globale in quanto usata da altri thread
	dbg("thdFeedFifo started")
		char tmpBuff[MAX_STRING_SIZE_VOICE];


	// Create element I want to add
	struct myMsgStruct msgOutVoice;

	// alloca la memoria per msgOutVoice 
	memset(&msgOutVoice, 0, sizeof(struct myMsgStruct));



	while (true)
	{
		//TEST creo il testo del messaggio in buf[] (millis. in formato stringa)
		ltoa(millis(), tmpBuff, MAX_STRING_SIZE_VOICE);

		playSingleNote(NOTE_A7, 80);


		if (!myRingBuf->isFull(myRingBuf))
		{
			// metto il messaggio in ring_buf----------
			// metto il messaggio in ring_buf----------
			for (size_t i = 0; i < 10; i++)
			{
				msgOutVoice.msgString[i] = tmpBuff[i];
			}
			//-----------------------------------------
			// così non funziona!! -> strcpy(tmpBuff, msgOutVoice.msgString);

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
				dbg2("........FreeSram", getFreeSram())
				//attendo più a lungo per permettere di svuotare il buffer
				chThdSleepMilliseconds(4000);//	chThdYield();
		}








	}

}

#pragma endregion 




/// ///////////////////////////////////////////////////////////////////////////////
// FiFo ->  Speech     ///////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo: FiFo -> Speech
// il formato stringhe ricevuto via Blutetooth è *...# dove ... è il testo inviato
static THD_WORKING_AREA(waFifoToSpeech, 64);
static THD_FUNCTION(thdFifoToSpeech, arg) {

	SwSerialSpeech.begin(9600);

	struct myMsgStruct  myVoiceMsg;

	while (1) {

		if (myRingBuf->elements > 0)
		{
			dbg2("				myRingBuf->elements:", myRingBuf->elements)
				// estrai in myVoiceMsg il messaggio (should be same as msgOutVoice) 
				myRingBuf->pull(myRingBuf, &myVoiceMsg);

			dbg2("				msgIn:", myVoiceMsg.msgString)
			
				SwSerialSpeech.print(myVoiceMsg.msgString);
		}

		chThdSleepMilliseconds(2000);//	chThdYield();//	

	}
}

#pragma endregion

#pragma region Processo di MONITOR
static THD_WORKING_AREA(waThreadMonitor, 64);
static THD_FUNCTION(ThreadMonitor, arg) {

	while (true)
	{
		//Serial.print(F("    waFeedFifo unused stack: "));
		//Serial.println(chUnusedStack(waFeedFifo, sizeof(waFeedFifo)));
		//Serial.print(F("    overrun errors: "));
		//Serial.println( OverrunErrorCount);
		count++;
		dbg2("mon.cnt:", count)
			//FIFO_SPEAK.push(int2str(count));
			uint32_t t = micros();
		// yield so other threads can run
		chThdYield();
		t = micros() - t;
		if (t > maxDelay) maxDelay = t;
		dbg2("FreeSram: ", getFreeSram())

			chThdSleepMilliseconds(1000);//	chThdYield();//	
	}

}

#pragma endregion 

// FINE PROCESSI CHIBIOS ////////////////////////////////////////////////////////////////////////////////

#pragma region [CHIBIOS RTOS]

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
	uint8_t newVariable;
	// heap is empty, use bss as start memory address 
	if ((uint16_t)__brkval == 0)
		return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
	// use heap end as the start of the memory address 
	else
		return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void chSetup() {
	// fill pool with PoolObject array

	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 10, thdFlashLed, NULL);
	chThdCreateStatic(waFifoToSpeech, sizeof(waFifoToSpeech), NORMALPRIO + 2, thdFifoToSpeech, NULL);
	dbg("started..waFifoToSpeech")
		chThdCreateStatic(waFeedFifo, sizeof(waFeedFifo), NORMALPRIO + 2, thdFeedFifo, NULL);
	dbg("started	waFeedFifo")
		//chThdCreateStatic(waSerialManager, sizeof(waSerialManager), NORMALPRIO + 2, SerialManager, NULL);
		//chThdCreateStatic(waSerialPcToFifoVoice, sizeof(waSerialPcToFifoVoice), NORMALPRIO + 2, SerialPcToFifoVoice, NULL);
		//		chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 2, ThreadMonitor, NULL);
}
void setup()
{
	SERIAL_PC.begin(SERIAL_PC_BAUD_RATE);
	SERIAL_PC.println("TestChibios");
	SwSerialSpeech.begin(9600);
	SwSerialSpeech.setTimeout(5000);


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

#if 1
	pinMode(Pin_SpeechSerialBusy, INPUT);
	for (size_t i = 0; i < 3; i++)
	{

		SwSerialSpeech.print("io ");
		delay(700);
	}

#endif // 0

#pragma endregion
	//halInit();
	//chSysInit();


	chBegin(chSetup);	while (1) {}
}
void loop() {
	// not used
}

#pragma endregion
