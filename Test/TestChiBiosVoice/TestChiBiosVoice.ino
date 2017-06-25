
//////////////////////////////////////////////////////////////////
// attende sulla porta definita come SPEECH_SERIAL stringhe
// che poi vengono ripetute dal modulo vocale
#include <ChibiOS_AVR.h>
#include <digitalWriteFast.h>
#include <SP0256-AL2\SP0256-AL2.h>




#define T 1000
#define SPEECH_SERIAL Serial
#define SERIAL_BAUD_RATE 9600
#define SPEECH_SERIAL_BAUD_RATE 115200

 
//char SpeakBuffer[INPUTCHARARRAYSIZE];



#pragma region SerialEchoSpeech
static THD_WORKING_AREA(waSerialEchoSpeech, 164);
static THD_FUNCTION(SerialEchoSpeech, arg) {
	SPEECH_SERIAL.begin(SPEECH_SERIAL_BAUD_RATE);
	SPEECH_SERIAL.setTimeout(5000);
	dbg("1")
	#pragma region Configura Speaker

		SpeakerInit();

		SpeakerReset();

	#pragma endregion Configura Speaker

	#pragma region //test speak
		strcpy(SpeakBuffer, " Hello Luca ");
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
	#pragma endregion test speak

	// inizializza il buffer di caratteri
	char str[INPUTCHARARRAYSIZE];
	for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { str[i] = 0; }

	while (1) {
		
		// attendo dei byte su seriale
			while (!SPEECH_SERIAL.available()) {dbg("s") chThdSleepMilliseconds(500); }	//chThdYield();delay(50);dbg('.')

		dbg("!")
		noInterrupts();
		int i = 0;
		// legge la seriale byte per byte perchè readString non funziona
		// il formato è *...# dove ... è il testo inviato

#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]
		while (SPEECH_SERIAL.available() > 0 || i < INPUTCHARARRAYSIZE)
		{
			str[i] = SPEECH_SERIAL.read();

			switch (str[i])
			{
			case '*': break;//salto il primo carattere * e non incremento i
			case '#': str[i] = '\0'; i++; break; // sostituisco con una pausa
			default: i++; 				break;
			}

			//dbg(char(str[i])) messa qui fa casino
		}
#pragma endregion		
		
		// pulisce il resto del buffer
		for (size_t j = i; j < INPUTCHARARRAYSIZE; j++) { str[j] = '\0'; }
		//dbg('|')
		speakString(str);
		interrupts();



		delay(500);
	}
}

#pragma endregion


//------------------------------------------------------------------------------


#pragma region  MONITOR
static THD_WORKING_AREA(waThreadMonitor, 64);
static THD_FUNCTION(ThreadMonitor, arg) {

	while (true)
	{
		Serial.print("Free Mem:");Serial.println(getFreeSram());

		chThdSleepMilliseconds(1000);//	chThdYield();
	}

}

#pragma endregion 

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
	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor),		NORMALPRIO + 2, ThreadMonitor, NULL);
//	chThdCreateStatic(waFifoFeed, sizeof(waFifoFeed), NORMALPRIO + 2, FifoFeed, NULL);
	chThdCreateStatic(waSerialEchoSpeech, sizeof(waSerialEchoSpeech),		NORMALPRIO + 2, SerialEchoSpeech, NULL);
	dbg("started..")
		SPEECH_SERIAL.begin(SPEECH_SERIAL_BAUD_RATE);
	SPEECH_SERIAL.setTimeout(5000);
#if 0
#pragma region Configura Speaker

	SpeakerInit();

	SpeakerReset();

#pragma endregion Configura Speaker

#pragma region //test speak
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
#pragma endregion test speak

#endif // 0

}
void setup()
{
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println("TestChibios");

	chBegin(chSetup);	while (1) {	}
}
void loop() {
 // not used
}


