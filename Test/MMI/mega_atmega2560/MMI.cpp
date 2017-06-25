
//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA 
				//#define dbg(t) Serial.println(t);

//#define T 1000
#define PC_SERIAL Serial		// riceve i comandi dal modulo robot
#define PC_SERIAL_BAUD_RATE 115200


#define SPEECH_SERIAL Serial1		// seriale del BlueTooth
#define SPEECH_SERIAL_BAUD_RATE 115200

/// messaggio di test
/// 31,1,1,1,1,300,100,220,1,1,1,1,1,1,70;
#define ROBOT_SERIAL Serial2		// ROBOT_SERIAL=Serial2  modulo Speak UP di riconoscimento vocale
#define ROBOT_SERIAL_BAUD_RATE 115200


#define dbg(cha)  // PC_SERIAL.println(cha);	 PC_SERIAL.println(cha);
#define dbg2(t,cha)  //  PC_SERIAL.print(t);PC_SERIAL.println(cha);//  SPEECH_SERIAL.print(t);SPEECH_SERIAL.println(cha);  
#pragma endregion

//////////////////////////////////////////////////////////////////////////
//																		//
//						 O P Z I O N I   R O B O T						//
//																		//
//////////////////////////////////////////////////////////////////////////

#define OPT_SERVOSONAR 1	//INCLUDI FUNZIONALITA' SERVO-SONAR
#define OPT_ENCODERS  0	//INCLUDI ENCODERS
#define OPT_BT 0	//bluetooth
#define OPT_SPEECH 1
//Interfaccia con modulo del parlato

//////////////////////////////////////////////////////////////////////////////////
// LIBRERIE                                    ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Librerie
#include <ChibiOS_AVR.h>
#include <TinyGPSplus\TinyGPS++.h>	//se manca non compila a causa del robot.cpp nella stessa cartella di robot\Commands_Enum.h
#include <digitalWriteFast.h>
#include <SP0256-AL2\SP0256-AL2.h>
//#include <CmdMessenger/CmdMessenger.h>
//#include <robot\Commands_Enum.h>
#include <string.h> 
#include "stringlib.h"
#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
//#include <servo/src/Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <robot/robotModel/robotModel.h>
#include "arduino.h"

int  GetCommandIndex();
bool  blAttentionWordFound();
int  GetCommandParamValue();
void processVoiceCommand();
static THD_FUNCTION(BtVoiceCommandInterpreter, arg);
static THD_FUNCTION(SerialManager, arg);
static THD_FUNCTION(FifoEchoSpeech, arg);
static THD_FUNCTION(FlashLed, arg);
void drawGaugeHoriz(int left, int top, int width, int height, int value, int min, int max, int valueColor);
void drawGauge(int left, int top, int v, int color);
static THD_FUNCTION(ThreadMonitorLCD, arg);
static THD_FUNCTION(ThreadMonitor, arg);
uint16_t getFreeSram();
void chSetup();
//
//
struct robotModel_c robotModel;

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger2/CmdMessenger2.h>
static CmdMessenger2 cmdRobot = CmdMessenger2(ROBOT_SERIAL);
void appendSpeech(char inStr[]);

#include <robot/RobotInterfaceCommandsMMI.h>



#pragma endregion

#pragma region LCD LIBRARY & OBJECT

#include <SPFD5408/SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408/SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
//#include <SPFD5408_TouchScreen.h>
// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:


// Color definitions - in 5:6:5
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF
#define TEST            0x1BF5
#define JJCOLOR         0x1CB6
#define JJORNG          0xFD03







Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;


#pragma endregion

//////////////////////////////////////////////////////////////////////////////////
//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//char SpeakBuffer[INPUTCHARARRAYSIZE];	//buffer per il parlato in SP0256-AL2.h
volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

//------------------------------------------------------------------------------
#pragma region DEFINIZIONE MAILBOX VOICE
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
MAILBOX_DECL(mailVoice, &letter, MB_COUNT);

#pragma endregion



bool isNumeric(const char *ch);
int moveEfondo(char array[], int size);
void ConvLetter(const char cifreIn[], int size);



// ///////////////////////////////////////////////////////////////////////////////
//  METTE IN CODA LA STRINGA DA PRONUNCIARE      ///////////////////////////////// 
// ///////////////////////////////////////////////////////////////////////////////

void appendSpeech( char inStr[]) {
	dbg2(">>appendSpeech riceve", inStr)

	// get object from memory pool
	PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
	if (!p) { Serial.println("chPoolAlloc failed");	while (1); }

	//p->size = inStr.length();
	//inStr.toCharArray(p->str, INPUTCHARARRAYSIZE);

	// passa la stringa nel buffer 
	p->size = strlen(inStr);
	dbg2(">> p->size", p->size)



//	strncpy(inStr, p->str, p->size);		//http://www.cplusplus.com/reference/cstring/strncpy/?kw=strncpy
	//strcpy(inStr, p->str );			//http://www.cplusplus.com/reference/cstring/strcpy/?kw=strcpy

	// -----------------------------------
	// mette i dati nella FIFO------------------
	
	for (int i = 0; i < p->size; i++)
	{
		p->str[i] = inStr[i];
	}
	for (int i = p->size; i < INPUTCHARARRAYSIZE; i++)	{		p->str[i] = '\0';	}
	dbg2(">> p->str", p->str)

	// send message
	msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
	if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

}


// ///////////////////////////////////////////////////////////////////////////////
//  PROCESS VOICE COMMANDS      //////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
// Interpreta il comando ricevuto ed invia a Robot il comando corrispondente

#pragma region PROCESS VOICE COMMANDS  
#define VOICECOMMANDMAXLEN 40	//Massima lunghezza della stringa in ingresso contenente il comando
#define WORDCOMMANDMAXLEN 10			// massima lunghezza di ciascun comando
#define ATTENTIONCOMMAND ROBOT	//indice del comando vocale che attiva il robot
#define COMMANDSCOUNT 10			// numero di comandi riconosciuti
#define ATTENTIONCOMMANDSCOUNT 2	//numero di comandi di attivazione

char AttentionWords[ATTENTIONCOMMANDSCOUNT][WORDCOMMANDMAXLEN] = { "ROBOT\0", "ARDUINO\0", }; //elenco comandi accettati come attivazione
char Vocabolary[COMMANDSCOUNT][WORDCOMMANDMAXLEN] = { "NIL\0",  "AVANTI\0", "INDIETRO\0", "DESTRA\0","SINISTRA\0", "FERMA\0" , "SONO\0" , "LUCA\0", "ANGELICA\0", "VINICIA\0",};
enum e_VoiceCommands			   { NIL,   AVANTI,   INDIETRO,   DESTRA,  SINISTRA,   FERMA , SONO , LUCA, ANGELICA, VINICIA};
static char voiceCommandCharArray[VOICECOMMANDMAXLEN] = "\0"; //array contenente la stringa vocale da riconoscere
enum e_cmdProcessingStatus { cmdstatus_IDLE, cmdStatus_WAITCMD, cmdStatus_WAITPARAM };
//estrae dalla stringa la prima parola e se coincide con un comando ritorna l'indice del comando
// ritorna 0 se non riconosce alcun comando
int  GetCommandIndex() {

	dbg2("@GetCommandIndex elabora: ", voiceCommandCharArray)


	char *cmdCandidate;
	int cmdId = 0;	// id del comando da restituire
	int i = 1;
	bool found = false;

	// estraggo il primo token-------------------------------------------
	char *rest ;	//= voiceCommandCharArray
	const char sep[] = " ";
	cmdCandidate = strtok_r(voiceCommandCharArray, sep, &rest);		//http://www.mkssoftware.com/docs/man3/strtok_r.3.asp#MULTITHREAD_SAFETY_LEVEL
	dbg2("@ cmdCandidate = ", cmdCandidate)
	


	//	strstr()		//http://www.cplusplus.com/reference/cstring/strstr/
	//	cmdCandidate = s.substring(0, firstSpacePos);

	// confronto con ogni voce del vocabolario
	while (!found && (i < COMMANDSCOUNT))
	{
		dbg2("@ confronto con Vocabolary[i]", Vocabolary[i]);
		//if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
 		if (strcmp(cmdCandidate, Vocabolary[i]) == 0)
		{
			found = true;
			cmdId = i;	//trovato >
			dbg2("@ trovato   cmdId=", cmdId);
		}
		i++;
	}

	if (cmdId>0)
	{

		// assegno la string rimanente a voiceCommandCharArray
		strncpy(voiceCommandCharArray, rest, strlen(rest));
		for (int i = strlen(rest); i < strlen(voiceCommandCharArray); i++)
		{
			voiceCommandCharArray[i] = '\0';
		}

		dbg2("@  GetCommandIndex   ritorna   cmdId=", cmdId);
		dbg2("@  Stringa rimanente", voiceCommandCharArray);

	}
	return cmdId;
}
//////////////////////////////////////////////////////////////////////////////////
// blAttentionWordFound		//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
///cerca nella stringa la parola di attenzione (es "ROBOT") 
///Se presente ritorna true
/// rimuove tutta la parte che precede la parola di attivazione, questa inclusa
bool  blAttentionWordFound() {
	// copio la stringa in una di servizio
	//String cha = *InputString;
	char  *cmdCandidate;
	int cmdId = 0;	// id del comando da restituire
	int i = 0;

	// elimino eventuali spazi all'inizio e alla fine stringa
	strtrim(voiceCommandCharArray);
	dbg2("#AttentionWordFound elabora: ", voiceCommandCharArray)

	// localizzo la parola di attenzione 
	// estraggo il primo token-------------------------------------------
	char *rest;		// = voiceCommandCharArray;  //tringa rimanente
	const char sep[] = " ";
	cmdCandidate = strtok_r(voiceCommandCharArray, sep, &rest);		//http://www.mkssoftware.com/docs/man3/strtok_r.3.asp#MULTITHREAD_SAFETY_LEVEL

	dbg2("# testing token:", cmdCandidate)
	dbg2("# string rimanente rest:", rest)

	bool found = false;
	//while ((cmdId = NIL) && (i < COMMANDSCOUNT))
	while (!found && (i < ATTENTIONCOMMANDSCOUNT) )
	{
		dbg2("# confronto con:", AttentionWords[i])
			//if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
			if (strcmp(cmdCandidate, AttentionWords[i]) == 0)		//http://www.cplusplus.com/reference/cstring/strcmp/
			{
				found = true;
				cmdId = i;	//trovato
				dbg("# Trovato")
			}
		i++;
	}



		//rimuovo il comando dalla stringa copiando rest in voiceCommandCharArray
		//s.remove(0, firstSpacePos);
		dbg2("#  Stringa rest:", rest);
		dbg2("#  len of rest:", strlen(rest));

		// assegno la string rimanente a voiceCommandCharArray
		strncpy(voiceCommandCharArray, rest, strlen(rest));
		for (int i =strlen(rest); i < strlen(voiceCommandCharArray); i++)
		{			voiceCommandCharArray[i] = '\0';		}


		dbg2("#  Stringa rimanente in uscita:", voiceCommandCharArray);



	return found;
}


//////////////////////////////////////////////////////////////////////////////////
// GetCommandParamValue		//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//estrae dalla stringa un valore numerico
// ritorna -1 se non riconosce alcun comando
#define COMMANDVALUEINVALID -999
int  GetCommandParamValue() {
	// copio la stringa in una di servizio
	//String cha = *InputString;
	char *strCandidateValue;
	int CandidateValue = COMMANDVALUEINVALID;	// id del comando da restituire
	int i = 0;
	// travaso str in cha

	// estraggo il primo token-------------------------------------------
	char *rest = voiceCommandCharArray;
	const char sep[] = " ";
	strCandidateValue = strtok_r(rest, sep, &rest);		//http://www.mkssoftware.com/docs/man3/strtok_r.3.asp#MULTITHREAD_SAFETY_LEVEL

 	dbg2("strCandidateValue", strCandidateValue)
	//provo a convertire
	sscanf(strCandidateValue, "%d", &CandidateValue);
	//CandidateValue = strCandidateValue.toInt();


	//rimuovo i caratteri dalla stringa
	if (CandidateValue != -999)
	{
		//rimuovo il comando dalla stringa copiando rest in voiceCommandCharArray
		//s.remove(0, firstSpacePos);
		memcpy(voiceCommandCharArray, rest, sizeof rest);
 
	}
	dbg2("GetCommandValue return:", CandidateValue)

	return CandidateValue;
}

										 
//////////////////////////////////////////////////////////////////////////////////
// processVoiceCommand		//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#define CMDPROCESSING_INITIAL_STATUS cmdStatus_WAITCMD
static e_cmdProcessingStatus cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; //0=idle,1:attn;2:cmd;3:cmdvalue
static int currentCmdId = -1;
static e_VoiceCommands currentCmd = NIL; // comand corrente da elaborare
#define processVoiceCommandReset 	cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; currentCmdId = -1; currentCmd = NIL;
//CMDPROCESSING_INITIAL_STATUS
// 1 non utilizza il comando di attenzione 0= utilizza il comando di attenzione
void processVoiceCommand() {
	boolean blEndVoiceCmdProcessing = false;
	//String InputString = "";
	int cmdValue = COMMANDVALUEINVALID;
	int cmdId = 0;
	dbg2("> processVoiceCommand riceve: ", voiceCommandCharArray);

	while (sizeof(voiceCommandCharArray)>0 && !blEndVoiceCmdProcessing)
	{
		dbg2("> stringa rimanente da elabore:", voiceCommandCharArray);
		switch (cmdProcessingStatus)
		{
		case 0: // idle
				// si attende la parola di attenzione
				// ritorna 0 se non riconosce alcun comando
			dbg(">*  cmdProcessingStatus = 0")
				strtrim(voiceCommandCharArray);
				//cha->trim();

			if (blAttentionWordFound())
			{
				dbg(">AttentionWordFound");
				cmdProcessingStatus = cmdStatus_WAITCMD;
			}
			else
			{
				dbg("> OI OI");
				appendSpeech("OI OI NON CAPITO");	//appendSpeech("OI OI");

				
				blEndVoiceCmdProcessing = true;
				processVoiceCommandReset
			}

			break;
		case cmdStatus_WAITCMD: // attende il comando
			dbg("> *       cmdProcessingStatus = 1")
				//cha->trim();
				

			cmdId = GetCommandIndex();
			currentCmd = (e_VoiceCommands)cmdId;

			switch (currentCmd)
			{
			case AVANTI:
				cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
				dbg(">* OKEY AVANTI");
 
				 appendSpeech("OKEY AVANTI");
				break;

			case INDIETRO:
				cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
				dbg(">* OKEY INDIETRO");
				appendSpeech("OKEY INDIETRO");
				break;

			case DESTRA:
				cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
				dbg(">* OKEY DESTRA");
				appendSpeech("OKEY DESTRA");
				break;

			case SINISTRA:
				cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
				dbg(">* OKEY SINISTRA");
				appendSpeech("OKEY SINISTRA");
				break;
			case SONO:
				cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un'altra parola chiave
				//dbg(">* OKEY SONO");
				//appendSpeech("CIAO LUCA");
				break;

			case STOP:
				// wait su semaforo di commandQueue


				//todo: esegue comando di stop

				cmdRobot.sendCmdStart(CmdRobotStopMoving);

				cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
				appendSpeech("OKEY STOP");

				dbg("\n OK Mi fermo")
				blEndVoiceCmdProcessing = true;
				processVoiceCommandReset
					break;



			default:// comando  riconosciuto ma non gestito
				cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
				dbg("cmd riconosciuto  ma non gestito");

				blEndVoiceCmdProcessing = true;

				break;
			}
			break;

		case cmdStatus_WAITPARAM: //si aspetta un parametro
			dbg("*       cmdProcessingStatus = cmdStatus_WAITPARAM")
				//cha->trim();
				strtrim(voiceCommandCharArray);

			cmdValue = GetCommandParamValue();
			dbg2("cmdValue:", cmdValue)
				if (cmdValue == COMMANDVALUEINVALID) {
					cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
					dbg("  Valore non valido")

				}
				else // Valore numerico ok
				{

					// converto in stringa
					char inputChNumber[5];
					itoa(cmdValue, inputChNumber, 10);
					appendSpeech(inputChNumber);
					//appendSpeech(voiceCommandCharArray);

					dbg("*****   invia il comando ******")
					dbg2("Valore comando:", cmdValue)
						// esegue il comando
						switch (currentCmd)
						{
						case AVANTI:
							cmdRobot.sendCmdStart(CmdRobotMoveCm);
							cmdRobot.sendCmdArg(cmdValue);
							cmdRobot.sendCmdEnd();
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
							break;

						case INDIETRO:
							cmdRobot.sendCmdStart(CmdRobotMoveCm);
							cmdValue *= -1;
							cmdRobot.sendCmdArg(cmdValue);
							cmdRobot.sendCmdEnd();
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
							break;

						case DESTRA:
							cmdRobot.sendCmdStart(CmdRobotRotateDeg);
							cmdRobot.sendCmdArg(cmdValue);
							cmdRobot.sendCmdEnd();
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command

							break;
						case SINISTRA: 
							cmdRobot.sendCmdStart(CmdRobotRotateDeg);
							cmdRobot.sendCmdArg(-cmdValue);
							cmdRobot.sendCmdEnd();
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
							break;

						case LUCA: 
							cmdRobot.sendCmdStart(CmdRobotRotateDeg);
							cmdRobot.sendCmdArg(-cmdValue);
							cmdRobot.sendCmdEnd();
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
							break;

						default:
							appendSpeech("OI OI");
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
							break;
						} //end switch
				}
			blEndVoiceCmdProcessing = true;
			processVoiceCommandReset

				break;
		case 3: //si aspetta un parametro non numerico
		
			cmdId = GetCommandIndex();
			currentCmd = (e_VoiceCommands)cmdId;

			switch (currentCmd)
			{
			case LUCA:
				cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
				dbg(">* LUCA");

				appendSpeech("CIAO CAPO");
				// todo disattivare il sensore di movimento
				break;

			case ANGELICA:
				cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
				dbg(">* ANGELICA ");

				appendSpeech("CIAO ANGELICA");
				// todo disattivare il sensore di movimento
				break;
			case VINICIA:
				cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
				dbg(">* OKEY AVANTI");

				appendSpeech("CIAO ANGELICA");
				// todo disattivare il sensore di movimento
				break;

			default:// persona sconosciuta
				cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
				appendSpeech("NON TI CONOSCO  MA PIACERE");

				break;
			}
			break;
		default: // non dovrebbe mai arrivare qui
			cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
			blEndVoiceCmdProcessing = true;

			break;

		}// end switch cmdProcessingStatus

	}//end while

	 //	blEndVoiceCmdProcessing = false;//ripristina lo stato iniziale

}


#pragma endregion



//------------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS  /////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
//RICEVE DA BT E INTERPRETA I COMANDI VOCALI      ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo: BtVoiceCommandInterpreter e comando riconosciuto su Voice FiFo
// gestisce i comandi vocali
// Monitora la SPEECH_SERIAL e mette quello che riceve in voiceCommandCharArray[]
// poi elabora il comando
static THD_WORKING_AREA(waBtVoiceCommandInterpreter, 200);//was 64
static THD_FUNCTION(BtVoiceCommandInterpreter, arg) {
	SPEECH_SERIAL.begin(SPEECH_SERIAL_BAUD_RATE);
	SPEECH_SERIAL.setTimeout(5000);
	//dbg("S")
	int i = 0;
	// inizializza il buffer d caratteri

	for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { voiceCommandCharArray[i] = 0; }

	while (1) {

		// attendo il testo sulla seriale
		while (!SPEECH_SERIAL.available()) { chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')
	

		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

		//noInterrupts();
		int i = 0; char c = '\0';
		while (SPEECH_SERIAL.available() > 0 && i < VOICECOMMANDMAXLEN && c != '#')
		{
			// legge il carattere dalla seriale
			char c = SPEECH_SERIAL.read();

				// l'app Android invia * all'inizio e # alla fine del testo
				switch (c)
				{
				case '*': break;//salto il primo carattere * e non incremento i
				case '#': voiceCommandCharArray[i]  = '\0'; i++; break; // sostituisco con una pausa
				default:	
					voiceCommandCharArray[i] = toupper(c);
					//if ((c >= 'a') && (c <= 'z'))
					//	voiceCommandCharArray[i] =c + ( 'A' - 'a');
					 
					i++; 				
					break;
				}

		}

		#pragma endregion

		// pulisce il resto del buffer
		for (size_t j = i; j < VOICECOMMANDMAXLEN; j++) {	voiceCommandCharArray[i] = '\0' ;	}
		

		// elaboro il contenuto della stringa voiceCommandCharArray
		processVoiceCommand();


		chThdSleepMilliseconds(500);//	chThdYield();
	}

}
#pragma endregion


//////////////////////////////////////////////////////////////////////////////////
// SERIAL MANAGER ( ROBOT INTERFACE  )    ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo: SerialManager se voce -> FiFoVoice

static THD_WORKING_AREA(waSerialManager, 64);
static THD_FUNCTION(SerialManager, arg) {
	ROBOT_SERIAL.begin(ROBOT_SERIAL_BAUD_RATE);
	ROBOT_SERIAL.setTimeout(5000);
	dbg("S")
		int i = 0;
	// inizializza il buffer di caratteri
	char strSerial[INPUTCHARARRAYSIZE];
	for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { strSerial[i] = 0; }

	// Setup CommandMessenger -----------------------------------------------------
	cmdRobot.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdRobot);// Attach my application's user-defined callback methods

	while (1) {
		//robotModel.status.sensors.bumper.center = !robotModel.status.sensors.bumper.center;
		//robotModel.status.sensors.analog[0] = random(0, 1023);
		//robotModel.status.sensors.analog[1] = random(0, 1023);
		dbg(">");
		cmdRobot.feedinSerialData();

/*		// attendo il testo sulla seriale
		while (!ROBOT_SERIAL.available()) { chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')

		//// get object from memory pool
		//PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
		//if (!p) { Serial.println("chPoolAlloc failed");	while (1); }


		dbg("[")
		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

				//noInterrupts();
				int i = 0; char c = '\0';
				while (ROBOT_SERIAL.available() > 0 && i < INPUTCHARARRAYSIZE && c != '#')
				{
					// mette i dati nella FIFO------------------
					char c = ROBOT_SERIAL.read();
					//p->str[i] = c;
					strSerial[i] = c;
					dbg(strSerial[i])
						switch (c)
						{
						case '*': break;//salto il primo carattere * e non incremento i
						case '#': //ultimo carattere, lo sostituisco con fine stringa
							//p->str[i] = '\0'; 
							strSerial[i]='\0'; 
							i++; 
							break; // sostituisco con una pausa
						default:
							i++; 				
							break;
						}

					//dbg(char(str[i])) messa qui fa casino
				}
				//p->size = i;

		#pragma endregion
		dbg("]")

			// pulisce il resto del buffer
			for (size_t j = i; j < INPUTCHARARRAYSIZE; j++) { strSerial[i] = '\0';} // p->str[i] = '\0'; 

		//	interrupts();


		// send message
		appendSpeech(strSerial);

		//msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
		//if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }
*/
		chThdSleepMilliseconds(500);//	chThdYield();
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

#pragma region  [speakTest Iniziale speak]
	strcpy(SpeakBuffer, "VOCE OKEI ");
	speakString(SpeakBuffer);

#if dbg
	speakNumber(1);
	delay(500);
	speakNumber((uint16_t)437);
	delay(500);
	strcpy(SpeakBuffer, "126");
	speakNumberStr(SpeakBuffer);
	delay(500);
#endif // dbg

#pragma endregion  


	while (1) {

		PoolObject_t *p;

		// get mailVoice
		chMBFetch(&mailVoice, (msg_t*)&p, TIME_INFINITE);


		// parla
		speakString(p->str);


		// put memory back into pool
		chPoolFree(&memPool, p);

		chThdSleepMilliseconds(200);//	chThdYield();//	
	}
}

#pragma endregion

// ///////////////////////////////////////////////////////////////////////////////
//  blinking LED       ///////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
#pragma region // BLINK LED
#define PIN_LED  13
#define LCD_LED_POS_X 230
#define LCD_LED_POS_Y 10

// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waFlashLed, 64);
static THD_FUNCTION(FlashLed, arg) {
	// Flash led every 200 ms.
	pinMode(PIN_LED, OUTPUT);		digitalWrite(PIN_LED, 0);	// led superiore

	while (1) {
		// Turn LED on.
		digitalWriteFast(PIN_LED, HIGH);
		tft.fillCircle(LCD_LED_POS_X, LCD_LED_POS_Y, 5, BLACK);
		// Sleep for 50 milliseconds.
		chThdSleepMilliseconds(200);

		// Turn LED off.
		digitalWriteFast(PIN_LED, LOW);
		tft.fillCircle(LCD_LED_POS_X, LCD_LED_POS_Y, 5, GREEN);

		// Sleep for 150 milliseconds.
		chThdSleepMilliseconds(800);
	}
}
#pragma endregion // BLINK LED----------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////
//  L C D  M O N I T O R	   ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo di gestione LCD MONITOR
// x lato corto
#define TFT_ROWSPACING 20	//Altezza in pixel di una riga di testo
#define TFTROW(r) r*TFT_ROWSPACING	// asse y
#define TFTDATACOL 100 // colonna dei dati

//////////////////////////////////////////////////////////////////////////////////
// Procedure grafiche 
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// disegna un rettangolo e lo riempie in proporzione al valore 
void drawGaugeHoriz(int left, int top, int width, int height, int value, int min, int max, int valueColor) {
#define GAUGE_COLOR_BCKGROUND BLACK
	int v;
	v = map(value, min, max, 0, width); //map(value, fromLow, fromHigh, toLow, toHigh).
	//tft.drawRect(left, top,   width, height, BLUE);
	tft.fillRect(left  , top,        v, height, valueColor);
	tft.fillRect(left+v, top,  width-v, height, GAUGE_COLOR_BCKGROUND);
}


void drawGauge(int left, int top, int v, int color) {
	drawGaugeHoriz( left, top, 100, 10, v, 0, 1023, color);
}

void drawLedRect(int left, int top, bool value, int col = RED) {
#define LEDSIZE 5
#define LEDCOLOR_ON RED
#define LEDCOLOR_OFF BLACK

	if (value)
	{
		tft.fillRect(left, top, LEDSIZE, LEDSIZE, col);
	}
	else
	{
		tft.fillRect(left, top, LEDSIZE, LEDSIZE, BLACK);
	}


}

static THD_WORKING_AREA(waThreadMonitorLCD, 100);
static THD_FUNCTION(ThreadMonitorLCD, arg) {
	tft.begin(0x9341); // SDFP5408

	tft.setRotation(0); // Need for the Mega, please changed for your choice or rotation initial
	tft.fillScreen(BLACK);
	tft.setTextSize(2);
	tft.setTextColor(GREEN);
	tft.setCursor(10, 10);
		
	tft.printAt(1, TFTROW(1), "FREE RAM=");
	tft.printAt(1, TFTROW(2), "SATS:");	
	tft.printAt(1, TFTROW(3), "LAT:");  
	tft.printAt(1, TFTROW(4), "LONG:"); 
	tft.printAt(1, TFTROW(5), "Pot:"); 
	tft.printAt(1, TFTROW(6), "Vbat:"); 

	drawLedRect(TFTDATACOL, TFTROW(7),1);
	bool HbLed = 0; //stato del led che visualizza l'ttivit� di questo Thread

	while (true)
	{
 //Serial.print(chUnusedStack(waThreadMonitorLCD, sizeof(waThreadMonitorLCD)));
		HbLed = !HbLed;
		drawLedRect(180, 10, HbLed, BLUE);

		tft.printAt(TFTDATACOL, TFTROW(1), getFreeSram());
		tft.printAt(TFTDATACOL, TFTROW(2), robotModel.status.sensors.gps.sats); // Number of satellites in use (u32)
		///tft.printAt(TFTDATACOL, TFTROW(3), dtostrf( robotBaseModel.status.gps.lat, 20, 8, s));
		///tft.printAt(TFTDATACOL, TFTROW(4), dtostrf(robotBaseModel.status.gps.lng, 20, 8, s));
		tft.printAt(TFTDATACOL, TFTROW(3),0);
		tft.printAt(TFTDATACOL, TFTROW(4), 0);
		
		drawGauge(TFTDATACOL, TFTROW(5), robotModel.status.sensors.analog[0], GREEN);
		drawGauge(TFTDATACOL, TFTROW(6), robotModel.status.sensors.analog[1], YELLOW);

	
		chThdSleepMilliseconds(1000);//	chThdYield();//	
	}

}

#pragma endregion 
//////////////////////////////////////////////////////////////////////////////////
// M O N I T O R	   ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo di MONITOR
static THD_WORKING_AREA(waThreadMonitor, 64);
static THD_FUNCTION(ThreadMonitor, arg) {
	while (true)
	{
		dbg("M.")
		//Serial.print(F("    waFifoFeed unused stack: "));
		//Serial.println(chUnusedStack(waFifoFeed, sizeof(waFifoFeed)));
		//Serial.print(F("    overrun errors: "));
		//Serial.println( OverrunErrorCount);
		Serial.print(F("Ram: "));
		Serial.println(getFreeSram());

		//count++;
		////FIFO_SPEAK.push(int2str(count));
		//uint32_t t = micros();
		//// yield so other threads can run
		//chThdYield();
		//t = micros() - t;
		//if (t > maxDelay) maxDelay = t;

		chThdSleepMilliseconds(2000);//	chThdYield();//	
	}

}

#pragma endregion 

// FINE PROCESSI CHIBIOS ////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////
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
	dbg("Starting all chThreads...")

	// fill pool with PoolObject array
	for (size_t i = 0; i < MB_COUNT; i++) {
		chPoolFree(&memPool, &PoolObject[i]);
	}

	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 2, FlashLed, NULL);
//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO , ThreadMonitorLCD, NULL);

	chThdCreateStatic(waThreadMonitorLCD, sizeof(waThreadMonitorLCD), NORMALPRIO+2 , ThreadMonitorLCD, NULL);
	chThdCreateStatic(waSerialManager, sizeof(waSerialManager), NORMALPRIO+2 , SerialManager, NULL);
	chThdCreateStatic(waBtVoiceCommandInterpreter, sizeof(waBtVoiceCommandInterpreter), NORMALPRIO, BtVoiceCommandInterpreter, NULL);
	chThdCreateStatic(waFifoEchoSpeech, sizeof(waFifoEchoSpeech), NORMALPRIO , FifoEchoSpeech, NULL);
	dbg("...all chThd started")
}
void setup()
{
	PC_SERIAL.begin(PC_SERIAL_BAUD_RATE);
	dbg("MMI");

	chBegin(chSetup);	while (1) {}
}
void loop() {
	// not used
}

#pragma endregion


