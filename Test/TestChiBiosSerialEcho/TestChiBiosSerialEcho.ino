// con un tread legge una seriale (Serial1) e con l'altro scrive su Serial
#define DEBUG_ON
#define SERIAL_IN Serial1
#define SERIAL_OUT Serial
#define SERIAL_IN_BAUD_RATE 115200
#define SERIAL_OUT_BAUD_RATE 115200

/////////////////////////////////////////////////////////////////////////////////
// LIBRERIE                                   ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <ChibiOS_AVR.h>
#include <ChibiOS_AVR/utility/chstreams.h>

#include <digitalWriteFast.h>
#include <buzzer\buzzer.h>
#include <MyRobotLibs\dbg.h>

#include <SoftwareSerial.h>
#include <TinyGPSplus\TinyGPS++.h>	//il linker s'incazza se non c'è

//#include <QueueList\QueueList.h>
//////////////////////////////////////////////////////////////////////////////////
// PARAMETRIZZAZIONE DEL SISTEMA               ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//#define ROBOT_SERIAL Serial			// riceve i comandi dal modulo robot
//#define ROBOT_SERIAL_BAUD_RATE 115200

//////////////////////////////////////////////////////////////////////////////////
//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//char SpeakBuffer[INPUTCHARARRAYSIZE];
volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

// Fifo definitions-------------------------
#pragma region FiFoSpeach setup
//// index of record to be filled
//size_t fifoWriteIndex_Speech = 0;
//// count of overrun errors
//int OverrunErrorCount = 0;
//// dummy data
//int count = 0;
//
//
//// size of fifo in records
//const size_t FIFO_SIZE_Speech = 5;
//
//// count of data records in fifo
//SEMAPHORE_DECL(fifoData_Speech, 0);
//
//// count of free buffers in fifo
//SEMAPHORE_DECL(fifoSpace_Speech, FIFO_SIZE_Speech);
#pragma endregion

 //------------------------------------------------------------------------------
#pragma region MAILBOX VOICE
				// mailbox size and memory pool object count
const size_t MBOX_CAPACITY_VOICE = 6;
///#define INPUTCHARARRAYSIZE 50
// type for a memory pool object
struct PoolObject_t {
	char* strMsg;
///	char str[INPUTCHARARRAYSIZE];
	int size;
};
// array of memory pool objects
PoolObject_t PoolObject[MBOX_CAPACITY_VOICE];

// memory pool structure
MEMORYPOOL_DECL(memPoolVoice, MBOX_CAPACITY_VOICE, 0);

// slots for mailbox messages
msg_t msgPool[MBOX_CAPACITY_VOICE];

// mailbox structure
MAILBOX_DECL(mailVoice, &msgPool, MBOX_CAPACITY_VOICE);

void sendMsg(mailbox_t mailVoice, msg_t p) {	
	// send message
	msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
	if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

}

#pragma endregion
//------------------------------------------------------------------------------

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
		chThdSleepMilliseconds(1960);
	}
}
#pragma endregion // BLINK LED----------------------------------------------------

//////////////////////////////////////////////////////////////////////////////////
// ALIMENTA LA FIFO    ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#pragma region Processo di test: Alimenta la FIFO ogni X secondi
static THD_WORKING_AREA(waFeedFifo, 64);
static THD_FUNCTION(thdFeedFifo, arg) {
	// definizione di Fifohead spostato a livello globale in quanto usata da altri thread
	while (true)
	{
		// get object from memory pool
		PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPoolVoice);
		if (!p) {Serial.println("chPoolAlloc failed from thdFeedFifo");	while (1);	}


		// form message
		p->strMsg = " permesso che devo passare";		// (char*)strSpeech;
		//p->strSpeech = " imbecille spostati che devo passare";		// (char*)strSpeech;
		//strcpy(p->str, "oi");
		//p->size = 2;
		//dbg2("thdFeedFifo send: ",p->strSpeech)
		
		// send message
		// va passato l'indirizzo di mailVoice
		msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
		if (s != MSG_OK) {	Serial.println("chMBPost failed");	while (1);	}

		playSingleNote(NOTE_A5, 40);

		chThdSleepMilliseconds(1000);//	chThdYield();


	}

}

#pragma endregion 

/// ///////////////////////////////////////////////////////////////////////////////
// SERIAL_IN -> FiFo       ///////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo: Serial -> FiFo

static THD_WORKING_AREA(waSerialPcToFifoVoice, 64);
static THD_FUNCTION(SerialPcToFifoVoice, arg) {
	SERIAL_IN.begin(SERIAL_IN_BAUD_RATE);
	SERIAL_IN.setTimeout(5000);
	dbg("S")
	int i = 0;
	//// inizializza il buffer d caratteri
	//char str[INPUTCHARARRAYSIZE];
	//for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { str[i] = 0; }
	char* inBuf;
	while (1) {

		// attendo il testo sulla seriale
		while (!SERIAL_IN.available()) {chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')

		//alloca lo spazio per il puntatore dalla memory pool
		PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPoolVoice);
		if (!p) { Serial.println("chPoolAlloc failed from SerialPcToFifoVoice");	while (1); }


		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

			dbg("[")
			noInterrupts();

			p->size = SERIAL_IN.available();

			 SERIAL_IN.readBytesUntil('\n',inBuf, p->size);
			 
			 p->strMsg=inBuf ;
			dbg("]")
		#pragma endregion


		interrupts();


		// send message
		sendMsg(mailVoice, (msg_t)p);

		chThdSleepMilliseconds(100);//	chThdYield();


	}

}
#pragma endregion




/// ///////////////////////////////////////////////////////////////////////////////
// FiFo ->  SerialOut     ///////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region Processo: FiFo -> SerialOut

static THD_WORKING_AREA(waFifoToSerialOut, 64);
static THD_FUNCTION(thdFifoToSerialOut, arg) {
	SERIAL_OUT.begin(9600);


	PoolObject_t *pVoice;
	SERIAL_OUT.print("FiFo -> SerialOut");

	while (1) {

		// get mailVoice
		chMBFetch(&mailVoice, (msg_t*)&pVoice, TIME_INFINITE);  // il cast di pVoice deve essere sempre di tipo msg_t
		playSingleNote(600, 40);

		SERIAL_OUT.print(pVoice->strMsg);

		// put memory back into pool
		chPoolFree(&memPoolVoice, pVoice);

		chThdSleepMilliseconds(200);//	chThdYield();//	
	}
}

#pragma endregion
/// ///////////////////////////////////////////////////////////////////////////////


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
	for (size_t i = 0; i < MBOX_CAPACITY_VOICE; i++) {
		chPoolFree(&memPoolVoice, &PoolObject[i]);
	}

	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 10, thdFlashLed, NULL);
	chThdCreateStatic(waFifoToSerialOut, sizeof(waFifoToSerialOut), NORMALPRIO + 2, thdFifoToSerialOut, NULL);
	dbg("started..waFifoToSerialOut")
	chThdCreateStatic(waFeedFifo, sizeof(waFeedFifo), NORMALPRIO + 2, thdFeedFifo, NULL);
	dbg("started..waFeedFifo")
	//chThdCreateStatic(waSerialManager, sizeof(waSerialManager), NORMALPRIO + 2, SerialManager, NULL);
	//chThdCreateStatic(waSerialPcToFifoVoice, sizeof(waSerialPcToFifoVoice), NORMALPRIO + 2, SerialPcToFifoVoice, NULL);
 }
void setup()
{
	SERIAL_PC.begin(SERIAL_PC_BAUD_RATE);
	SERIAL_PC.println("TestChibios");
	SERIAL_OUT.begin(9600);
	SERIAL_OUT.setTimeout(5000);

	#pragma region test swSerial

		#if 1
			pinMode(Pin_SpeechSerialBusy, INPUT);
			for (size_t i = 0; i < 3; i++)
			{

				SERIAL_OUT.print("io ");
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
