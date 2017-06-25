//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//#define dbg(t) Serial.println(t);

//#define T 1000
#define SPEECH_SERIAL Serial		// seriale del BlueTooth
#define SPEECH_SERIAL_BAUD_RATE 115200


#define ROBOT_SERIAL Serial			// riceve i comandi dal modulo robot
#define ROBOT_SERIAL_BAUD_RATE SPEECH_SERIAL_BAUD_RATE

#define appendSpeak(s) SPEECH_SERIAL.print("Voce>> ");SPEECH_SERIAL.println(s)
#define dbg(s) SPEECH_SERIAL.println(s);
#define dbg2(t,s) SPEECH_SERIAL.print(t);SPEECH_SERIAL.println(s);

//////////////////////////////////////////////////////////////////////////////////
// LIBRERIE                                    ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Librerie
//#include <ChibiOS_AVR.h>
#include <digitalWriteFast.h>
//#include <SP0256-AL2\SP0256-AL2.h>
//#include <dbg\Dbg.h>
#include <CmdMessenger/CmdMessenger.h>
#include <robot\Commands_Enum.h>
#pragma endregion

//////////////////////////////////////////////////////////////////////////////////
//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#include "arduino.h"

int  GetCommandIndex(String &s);
bool  blAttentionWordFound(String &s);
int  GetCommandValue(String& s);
void processVoiceCommand(String &s);
//
//
CmdMessenger cmd = CmdMessenger(ROBOT_SERIAL);






// ///////////////////////////////////////////////////////////////////////////////
//  PROCESS VOICE COMMANDS      //////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
// Interpreta il comando ricevuto ed invia a Robot il comando corrispondente
#pragma region PROCESS VOICE COMMANDS  


#define ATTENTIONCOMMAND ROBOT
#define COMMANDSCOUNT 7
String Vocabolary[COMMANDSCOUNT] = { "NIL", "ROBOT", "AVANTI", "INDIETRO", "DESTRA","SINISTRA", "STOP" };
enum e_VoiceCommands                { NIL, ROBOT, AVANTI, INDIETRO, DESTRA, SINISTRA, STOP };

//estrae dalla stringa la prima parola e se coincide con un comando ritorna l'indice del comando
// ritorna 0 se non riconosce alcun comando

int  GetCommandIndex(String &s) {
	//estrae dalla stringa la prima parola e se coincide con un comando ritorna l'indice del comando
	// ritorna 0 se non riconosce alcun comando
	// rimuove 
	// copio la stringa in una di servizio
	//String s = *InputString;
	String cmdCandidate = "";
	int cmdId = 0;	// id del comando da restituire
	int i = 1;
	bool found = false;


	// localizzo il primo spazio
	int firstSpace = s.indexOf(' ');
	if (firstSpace == -1) { firstSpace = s.length(); }
	dbg2("firstSpace:", firstSpace)

	cmdCandidate = s.substring(0, firstSpace);
	dbg2("cmdCandidate:",cmdCandidate);

	// confronto con ogni voce del vocabolario
	while (!found && (i < COMMANDSCOUNT))
	{
		dbg2("confronto con Vocabolary[i]",Vocabolary[i]);
		if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
		{
			cmdId = i ;	//trovato >
			dbg2("!!  GetCommandIndex              cmdId=",cmdId);
		}
		i++;
	}

	//rimuovo il comando dalla stringa
	if (cmdId>0)
	{
		s.remove(0, firstSpace);
		dbg2("!!  GetCommandIndex   ritorna   cmdId=",cmdId);
		dbg2("Stringa rimanente",s);

	}
	return cmdId;
}



///cerca nella stringa la parola di attenzione (es "ROBOT") 
///Se presente ritorna true
/// rimuove tutta la parte che precede la parola di attivazione, questa inclusa
bool  blAttentionWordFound(String &s) {
	// copio la stringa in una di servizio
	//String s = *InputString;
	String cmdCandidate = "";
	int cmdId = 0;	// id del comando da restituire
	int i = 0;
	dbg2("AttentionWordFound elabora: ",s)

	// localizzo la parola di attenzione 
	// localizzo il primo spazio
	int firstSpace = s.indexOf(' ');
	dbg2("firstSpace:",firstSpace)
	if (firstSpace == -1) { firstSpace = s.length(); }

	cmdCandidate = s.substring(0, firstSpace);
	dbg2("cmdCandidate:", cmdCandidate)
		bool found = false;
	//while ((cmdId = NIL) && (i < COMMANDSCOUNT))
		while (!found && (i < COMMANDSCOUNT))
		{
		dbg2("confronto con:", Vocabolary[i])
		if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
		{
			cmdId = i ;	//trovato
			dbg("Trovato")
		}
		i++;
	}

	//rimuovo il comando dalla stringa
	if (cmdId>0)
	{
		dbg2("cmdId:",cmdId)
		s.remove(0, firstSpace+1);
		dbg2("Stringa rimanente:",s)

	}
	return cmdId;
}

//estrae dalla stringa un valore numerico
// ritorna -1 se non riconosce alcun comando

#define COMMANDVALUEINVALID -999
int  GetCommandValue(String& s) {
	// copio la stringa in una di servizio
	//String s = *InputString;
	String strCandidateValue = "";
	int CandidateValue = COMMANDVALUEINVALID;	// id del comando da restituire
	int i = 0;
	// travaso str in s

	// localizzo il primo spazio
	int firstSpace = s.indexOf(' '); // -1 se non lo trova
	if (firstSpace == -1) { firstSpace = s.length(); }

	//estraggo dall'inizio fino al primo spazio
	strCandidateValue = s.substring(0, firstSpace);
	dbg2("strCandidateValue", strCandidateValue)
	//provo a convertire
	CandidateValue = strCandidateValue.toInt();


	//rimuovo i caratteri dalla stringa
	if (CandidateValue != -999)
	{
		s.remove(0, firstSpace);
		dbg(s);

	}
	dbg2("GetCommandValue return:", CandidateValue)

	return CandidateValue;
}

#pragma endregion



#pragma region processVoiceCommand
	static int cmdProcessingStatus = 0; //0=idle,1:attn;2:cmd;3;cmdvalue
	static int currentCmdId = -1;
	static e_VoiceCommands currentCmd = NIL; // comand corrente da elaborare
#define processVoiceCommandReset 	cmdProcessingStatus = 0; currentCmdId = -1; currentCmd = NIL;
void processVoiceCommand(String &s) {
	boolean blEnd = false;
											 //String InputString = "";
	int cmdValue = COMMANDVALUEINVALID;
	int cmdId=0;
	while (s.length()>0 && !blEnd)
	{
		dbg2("processVoiceCommand elabora:",s);
		switch (cmdProcessingStatus)
		{
		case 0: // idle
				// si attende la parola di attenzione
				// ritorna 0 se non riconosce alcun comando
			dbg("*       cmdProcessingStatus = 0")
			s.trim();

			if (blAttentionWordFound(s))
			{
				dbg("AttentionWordFound");
				cmdProcessingStatus = 1;
			}
			else
			{
				appendSpeak("OI OI");
				blEnd = true;
				processVoiceCommandReset
			}

			break;
		case 1: // attende il comando
			dbg("*       cmdProcessingStatus = 1")
			s.trim();

			cmdId = GetCommandIndex(s);
			currentCmd = (e_VoiceCommands)cmdId;

			switch (currentCmd)
			{
			case AVANTI:
			case INDIETRO:
			case DESTRA:
			case SINISTRA:
				cmdProcessingStatus = 2; // attende un valore
				dbg("* cmd riconosciuto *");
				 

				break;
			case STOP:
				// wait su semaforo di commandQueue


				//todo: esegue comando di stop
 
				cmd.sendCmdStart(CmdRobotStopMoving);
 
				cmdProcessingStatus = 0; // torna in IDLE
				dbg("\n OK Mi fermo")
				blEnd = true;
				processVoiceCommandReset
				break;

			default:// comando  riconosciuto ma non gestito
				cmdProcessingStatus = 0; // torna in IDLE
				dbg("cmd riconosciuto  ma non gestito");

				blEnd = true;

				break;
			}
			break;

		case 2: //si aspetta un parametro
			dbg("*       cmdProcessingStatus = 2")
			s.trim();
			cmdValue = GetCommandValue(s);
			dbg2("cmdValue:", cmdValue)
			if (cmdValue == COMMANDVALUEINVALID) {
				cmdProcessingStatus = 0; // torna in IDLE
				dbg("  Valore non valido" )

			}
			else
			{
				dbg2("Valore comando:",cmdValue)
				// esegue il comando
				dbg("*****   invia il comando ******")
				switch (currentCmd)
				{
				case AVANTI:
					cmd.sendCmdStart(CmdRobotMoveCm);
					cmd.sendCmdArg(cmdValue);
					cmd.sendCmdEnd();
					cmdProcessingStatus = 0; // running command
					break;

				case INDIETRO:
					cmd.sendCmdStart(CmdRobotMoveCm);
					cmdValue *= -1;
					cmd.sendCmdArg(cmdValue);
					cmd.sendCmdEnd();
					cmdProcessingStatus = 0; // running command
					break;

				case DESTRA:
					cmd.sendCmdStart(CmdRobotRotateRadiants);
					cmdValue *= -1;
					cmd.sendCmdArg(cmdValue);
					cmd.sendCmdEnd();
					cmdProcessingStatus = 0; // running command

					break;
				case SINISTRA:
					break;
					cmd.sendCmdStart(CmdRobotRotateRadiants);
					cmdValue *= -1;
					cmd.sendCmdArg(cmdValue);
					cmd.sendCmdEnd();
					cmdProcessingStatus = 0; // running command


				default:
					appendSpeak("OI OI");
					cmdProcessingStatus = 0; // torna in IDLE
					break;
				} //end switch
			}
			blEnd = true;
			processVoiceCommandReset

			break;
		default: // non dovrebbe mai arrivare qui
			cmdProcessingStatus = 0; // torna in IDLE
			blEnd = true;

			break;

		}// end switch cmdProcessingStatus

	}//end while

//	blEnd = false;//ripristina lo stato iniziale

}

#pragma endregion


void setup() {
	SPEECH_SERIAL.begin(SPEECH_SERIAL_BAUD_RATE);
	SPEECH_SERIAL.setTimeout(5000);
	ROBOT_SERIAL.println("TestVoiceCommands");


}

void loop(){
	// attendo il testo sulla seriale
	while (!ROBOT_SERIAL.available()) { delay(5); }	// dbg(".") chThdYield();delay(50);dbg('.')

	String s = ROBOT_SERIAL.readString();

	dbg2("Ricevuto: ",s);
	processVoiceCommand(s);
}

