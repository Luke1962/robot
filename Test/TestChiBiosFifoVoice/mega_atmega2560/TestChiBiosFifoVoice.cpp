
//////////////////////////////////////////////////////////////////////////////////
// LIBRERIE                                    ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#include <ChibiOS_AVR.h>
#include <digitalWriteFast.h>
#include <SP0256-AL2\SP0256-AL2.h>
//#include <QueueList\QueueList.h>


//////////////////////////////////////////////////////////////////////////////////
// PARAMETRIZZAZIONE DEL SISTEMA               ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#define T 1000
#define SPEECH_SERIAL Serial1
#define SPEECH_SERIAL_BAUD_RATE 115200


#define ROBOT_SERIAL Serial			// riceve i comandi dal modulo robot
#define ROBOT_SERIAL_BAUD_RATE 9600

//////////////////////////////////////////////////////////////////////////////////
//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#include "arduino.h"

static THD_FUNCTION(FlashLed, arg);
static THD_FUNCTION(FifoFeed, arg);
static THD_FUNCTION(Serial1ToFifo, arg);
static THD_FUNCTION(SerialManager, arg);
static THD_FUNCTION(FifoEchoSpeech, arg);
static THD_FUNCTION(ThreadMonitor, arg);
uint16_t getFreeSram();
void chSetup();
//
//
char SpeakBuffer[INPUTCHARARRAYSIZE];
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
const size_t MB_COUNT = 6;

// type for a memory pool object
struct PoolObject_t {
	char* name;
	char str[INPUTCHARARRAYSIZE];
	int size;
};
// array of memory pool objects
PoolObject_t PoolObject[MB_COUNT];

// memory pool structure
MEMORYPOOL_DECL(memPool, MB_COUNT, 0);

// slots for mailbox messages
msg_t letter[MB_COUNT];

// mailbox structure
MAILBOX_DECL(mail, &letter, MB_COUNT);

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
static THD_FUNCTION(FlashLed, arg) {
	// Flash led every 200 ms.
	pinMode(PIN_LED, OUTPUT);		digitalWrite(PIN_LED, 0);	// led superiore

	while (1) {
		// Turn LED on.
		digitalWriteFast(PIN_LED, HIGH);
		// Sleep for 50 milliseconds.
		chThdSleepMilliseconds(50);

		// Turn LED off.
		digitalWriteFast(PIN_LED, LOW);

		// Sleep for 150 milliseconds.
		chThdSleepMilliseconds(950);
	}
}
#pragma endregion // BLINK LED----------------------------------------------------

//////////////////////////////////////////////////////////////////////////////////
// ALIMENTA LA FIFO    ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#pragma region Processo di test: Alimenta la FIFO ogni X secondi
static THD_WORKING_AREA(waFifoFeed, 64);
static THD_FUNCTION(FifoFeed, arg) {
	// definizione di Fifohead spostato a livello globale in quanto usata da altri thread
	while (true)
	{

		// get object from memory pool
		PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
		if (!p) {Serial.println("chPoolAlloc failed");	while (1);	}


		// form message
		p->name = "oi";		// (char*)name;
		strcpy(p->str, "oi");
		p->size = 2;
		
		// send message
		msg_t s = chMBPost(&mail, (msg_t)p, TIME_IMMEDIATE);
		if (s != MSG_OK) {	Serial.println("chMBPost failed");	while (1);	}


		chThdSleepMilliseconds(5000);//	chThdYield();


	}

}

#pragma endregion 

//////////////////////////////////////////////////////////////////////////////////
//SerialBT -> FiFo       ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo: Serial -> FiFo

static THD_WORKING_AREA(waSerial1ToFifo, 64);
static THD_FUNCTION(Serial1ToFifo, arg) {
	SPEECH_SERIAL.begin(SPEECH_SERIAL_BAUD_RATE);
	SPEECH_SERIAL.setTimeout(5000);
	dbg("S")
	int i = 0;
	// inizializza il buffer d caratteri
	char str[INPUTCHARARRAYSIZE];
	for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { str[i] = 0; }

	while (1) {

		// attendo il testo sulla seriale
		while (!SPEECH_SERIAL.available()) {chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')

		// get object from memory pool
		PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
		if (!p) { Serial.println("chPoolAlloc failed");	while (1); }


		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

			dbg("[")
			noInterrupts();
			int i = 0; char c='\0';
			while (SPEECH_SERIAL.available() > 0 && i < INPUTCHARARRAYSIZE && c!='#')
			{
				// mette i dati nella FIFO------------------
				char c =SPEECH_SERIAL.read();
				p->str[i] = c;
				dbg(c)
				switch (c)
				{
				case '*': break;//salto il primo carattere * e non incremento i
				case '#': p->str[i] = '\0'; i++; break; // sostituisco con una pausa
				default: i++; 				break;
				}

				//dbg(char(str[i])) messa qui fa casino
			}
			p->size = i;

		#pragma endregion

		// pulisce il resto del buffer
		for (size_t j = i; j < INPUTCHARARRAYSIZE; j++) { p->str[i] = '\0'; }

		dbg("]")
		interrupts();


		// send message
		msg_t s = chMBPost(&mail, (msg_t)p, TIME_IMMEDIATE);
		if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

		chThdSleepMilliseconds(100);//	chThdYield();


	}

}
#pragma endregion
//////////////////////////////////////////////////////////////////////////////////
//  SerialManager      ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo: SerialManager se voce -> FiFoVoice

static THD_WORKING_AREA(waSerialManager, 64);
static THD_FUNCTION(SerialManager, arg) {
	ROBOT_SERIAL.begin(ROBOT_SERIAL_BAUD_RATE);
	ROBOT_SERIAL.setTimeout(5000);
	dbg("S")
		int i = 0;
	// inizializza il buffer d caratteri
	char str[INPUTCHARARRAYSIZE];
	for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { str[i] = 0; }

	while (1) {

		// attendo il testo sulla seriale
		while (!ROBOT_SERIAL.available()) { chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')

		// get object from memory pool
		PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
		if (!p) { Serial.println("chPoolAlloc failed");	while (1); }


		dbg("[")
		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

			//noInterrupts();
			int i = 0; char c = '\0';
			while (ROBOT_SERIAL.available() > 0 && i < INPUTCHARARRAYSIZE && c != '#')
			{
				// mette i dati nella FIFO------------------
				char c = ROBOT_SERIAL.read();
				p->str[i] = c;
				dbg(c)
					switch (c)
					{
					case '*': break;//salto il primo carattere * e non incremento i
					case '#': p->str[i] = '\0'; i++; break; // sostituisco con una pausa
					default: i++; 				break;
					}

				//dbg(char(str[i])) messa qui fa casino
			}
			p->size = i;

		#pragma endregion
		dbg("]")

		// pulisce il resto del buffer
		for (size_t j = i; j < INPUTCHARARRAYSIZE; j++) { p->str[i] = '\0'; }

		//	interrupts();


		// send message
		msg_t s = chMBPost(&mail, (msg_t)p, TIME_IMMEDIATE);
		if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

		chThdSleepMilliseconds(100);//	chThdYield();


	}

}
#pragma endregion


//////////////////////////////////////////////////////////////////////////////////
// FiFo ->  Speech     ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo: FiFo -> Speech
// il formato stringhe ricevuto via Blutetooth � *...# dove ... � il testo inviato
static THD_WORKING_AREA(waFifoEchoSpeech, 64);
static THD_FUNCTION(FifoEchoSpeech, arg) {
	SPEECH_SERIAL.begin(SPEECH_SERIAL_BAUD_RATE);
	SPEECH_SERIAL.setTimeout(5000);
	#pragma region [Configura Speaker]

		SpeakerInit();

		SpeakerReset();

	#pragma endregion 

	#pragma region  [test Iniziale speak]
		strcpy(SpeakBuffer, " Hello  ");
		speakString(SpeakBuffer);
		delay(500);
		strcpy(SpeakBuffer, " okei ");
		speakString(SpeakBuffer);

		SpeakPhoneme(0x1f);
		SpeakPhoneme(0x00);
		delay(500);
		SpeakPhoneme(0x17);
		SpeakPhoneme(0x00);
		delay(500);
		SpeakPhoneme(0x1f);
		SpeakPhoneme(0x00);
		delay(500);
		SpeakPhoneme(0x17);
		SpeakPhoneme(0x00);
		delay(500);
	#pragma endregion  

	
	while (1) {

		PoolObject_t *p;

		// get mail
		chMBFetch(&mail, (msg_t*)&p, TIME_INFINITE);


		// parla
		speakString(p->str);
 

		// put memory back into pool
		chPoolFree(&memPool, p);

		chThdSleepMilliseconds(200);//	chThdYield();//	
	}
}

#pragma endregion

#pragma region Processo di MONITOR
static THD_WORKING_AREA(waThreadMonitor, 64);
static THD_FUNCTION(ThreadMonitor, arg) {

	while (true)
	{
		//Serial.print(F("    waFifoFeed unused stack: "));
		//Serial.println(chUnusedStack(waFifoFeed, sizeof(waFifoFeed)));
		//Serial.print(F("    overrun errors: "));
		//Serial.println( OverrunErrorCount);

			count++;
			//FIFO_SPEAK.push(int2str(count));
			uint32_t t = micros();
			// yield so other threads can run
			chThdYield();
			t = micros() - t;
			if (t > maxDelay) maxDelay = t;

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
	for (size_t i = 0; i < MB_COUNT; i++) {
		chPoolFree(&memPool, &PoolObject[i]);
	}

	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 2, ThreadMonitor, NULL);
	chThdCreateStatic(waFifoFeed, sizeof(waFifoFeed), NORMALPRIO + 2, FifoFeed, NULL);
	chThdCreateStatic(waSerialManager, sizeof(waSerialManager), NORMALPRIO + 2, SerialManager, NULL);
	chThdCreateStatic(waSerial1ToFifo, sizeof(waSerial1ToFifo), NORMALPRIO + 2, Serial1ToFifo, NULL);
	chThdCreateStatic(waFifoEchoSpeech, sizeof(waFifoEchoSpeech), NORMALPRIO + 2, FifoEchoSpeech, NULL);
	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 10, FlashLed, NULL);
	dbg("started..")
}
void setup()
{
	Serial.begin(ROBOT_SERIAL_BAUD_RATE);
	Serial.println("TestChibios");

	chBegin(chSetup);	while (1) {}
}
void loop() {
	// not used
}

#pragma endregion


