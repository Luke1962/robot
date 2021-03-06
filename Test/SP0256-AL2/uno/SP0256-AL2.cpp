/*
 * interfacing to an old 1980s speech chip
 *
 * General Instruments SPO-256-AL2 pinouts
 *   1   V_SS (aka ground)
 *   2   ~RESET
 *   3   ROM_DISABLE (disables external ROM)
 *   4   C1 (external ROM control lines)
 *   5   C2
 *   6   C3
 *   7   V_DD (power for non-CPU bits)
 *   8   SBY (standby; output)
 *   9   ~LRQ (load request; output)
 *  10   A8
 *  11   A7
 *  12   SER_OUT (I/O to external ROM)
 *  13   A6
 *  14   A5
 *  15   A4
 *  16   A3
 *  17   A2
 *  18   A1
 *  19   SE (ALD strobe [mode] enable)
 *  20   ~ALD (address load)
 *  21   SER_IN (I/O from external ROM)
 *  22   TEST
 *  23   V_D1 (power for CPU)
 *  24   DIGITAL_OUT (10kHz PWM sound)
 *  25   ~SBY_RESET
 *  26   ROM_CLOCK (clock for external ROM; output)
 *  27   OSC1 (datasheet requests a 3.12MHz crystal, but that is very hard to find,
 *  28   OSC2  and many people report using a common 3.59 crystal without any side effects)
 *
 * Support parts (all optional, it turns out)
 *  3.12 MHz crystal (though anything up to and including 3.59 MHz apparently works fine)
 *  2x22 pF caps for the crystal
 *  100 K resistor to pull up pins 2 and 25
 *  0.1 uF cap to slow the pull up on pins 2 and 25 (works as a reset circuit)
 *  33 K resistor for low-pass filter
 *  0.022 uF cap for low-pass filter
 *  1 uF cap for output decoupling
 *  one diagram has a diode also helping pull up pins 2 and 25, which discharges the capacitor if power blips on and off faster than the resistor will,
 *  but that is unnecessary for non-production use.
 *  another diagram shows two output filters in series, using 2x (33K R + 0.022 uF C)
 *
 * and to amplify the output to drive a speaker any basic opamp will do. One example has:
 *  386 opamp
 *  3x 0.1 uF caps for power debounce
 *  100 uF cap for output decoupling (another circuit shows a 220 uF here; it doesn't matter much)
 *  10 uf cap for ?
 *
 *
 * References:
 *    http://www.atarimagazines.com/v5n9/TalkingTypewriter.html
 *    http://www.cpcwiki.eu/imgs/e/eb/SV_SPO256_001.jpg
 *    http://analog.katorlegaz.com/analog_1985-04_120dpi_jpeg_cropped/analog_1985-04_065.html
 *
 *
 * In truth I found that the speakString chip was so unpicky about the clock input that I could
 * drive its clock using a timer on the arduino running at 16MHz/6 = 2.66 MHz, outputting on
 * a pin on the atmel, and sending that into the OSC1 pin of the SPO.
 * #define DRIVE_CLOCK_USING_ATMEL to enable this and avoid needing a crystal and supporting caps.
 *
 * Secondly the reset circuits are done using an RC circuit (the 100k resistor and the 0.1 uF capacitor
 * which pull pins 2 and 25 slowly up), can be driven from a arduino pin.
 * #define DRIVE_RESET_USING_ATMEL to enable this and avoid needing the R/C circuit. (Also this lets
 * you reset the SPO whenver the arduino resets. otherwise only power-on resets the SPO)
 *
 * Thirdly the 33K/0.22uF low-pass filtering of the output of pin 24, and subsequent amplification of
 * the result using an opamp, seems pointless. A cheap moving-coil speaker (I used an old over-the-ear earphone)
 * has enough inductance to act as a low-pass filter on its own, when driven with the 5V square wave coming
 * from pin 24.
 * If I used a piezo speaker then the output was buzzy. However the RC filtering didn't reduce the buzz much
 * at all. Neither did two stages of the same filtering. And the view on the oscilloscope showed the signal
 * was still spikey and ugly. Plus the opamp part was delicate to keep from clipping. Anyway the opamp isn't
 * going to make a louder signal than 5V peak-to-peak unless it has its own power supply.
 *
 * So in the end the entire circuit is SPO driving a speaker directly, and some wires connecting the SPO to
 * power, ground, and 10 arduino pins as defined below.
 *
 */
 
// Un/comment as needed/desired.
//#define DRIVE_CLOCK_USING_ATMEL
#define DRIVE_RESET_USING_ATMEL
 
// SPO pin assignments on arduino
#define SPO_A1    8
#define SPO_A2    9
#define SPO_A3    10
#define SPO_A4    11
#define SPO_A5    12
#define SPO_A6    13
#define SPO_ALD   6			// cavetto blu =Wr
#define SPO_LRQ   7 //
#define SPO_RESET 5 // needed if we drive reset from the arduino; not otherwise
#define SPO_OSC1  4 // needed if we drive clocks from arduino; not otherwise. if used this must be pin 11 because we use timer 2 OC2A to create a clock, and that only runs on [board] pin 11
 
// SPO-256-AL2 phonemes
#include "arduino.h"

void readSerialString( char *strArray );
static void sayPhoneme(uint8_t phoneme);
uint8_t CharToPhoneme( char c ,char c1, int &skipNext);
void speakArrayOfChar( char *strArray );
//
//
void speakString( String s );
void speakCodedSentence( char c );
void test();
enum {
  // Silence
  pPA1 = 0x00, // before BB,DD,GG,JH (10 msec)
  pPA2 = 0x01, // before BB,DD,GG,JH (30 msec)
  pPA3 = 0x02, // before PP,TT,KK,CH and between words (50 msec)
  pPA4 = 0x03, // between clauses and sentences (100 msec)
  pPA5 = 0x04, // between clauses and sentences (200 msec)
   
  // Short Vowels
  pIH  = 0x0c, // sIt
  pEH  = 0x07, // End, gEntlemen, Extend
  pAE  = 0x1a, // hAt, extrAct, Act
  pUH  = 0x1e, // bOOk, cOOkie, fUll
  pAO  = 0x17, // tALk, sOng, AUght
  pAX  = 0x0f, // sUCCeed, lApel, intrUCt
  pAA  = 0x18, // hOT, pOTtery, cOTen
 
  // Long Vowels
  pIY  = 0x13, // sEE, trEAt, pEOple, pEnny
  pEY  = 0x14, // bEIge, grEAt, stAte, trAY
  pAY  = 0x06, // skY, kIte, mIGHty
  pOY  = 0x05, // bOY, nOIse, vOIce
  pUW1 = 0x16, // compUter (after clusters with YY)
  pUW2 = 0x1f, // fOOd, tWO (monosyllabic words)
  pOW  = 0x35, // zOne, clOse, snOW, bEAU
  pAW  = 0x20, // OUt, sOUnd, mOUse, dOWn
  pEL  = 0x3e, // saddLE, littLE, angLE, gentLEmen
 
  // R-Colored Vowels
  pER1 = 0x33, // lettER, furnitURE, intERRupt
  pER2 = 0x34, // bIRd, fERn, bURn (monosyllabic words)
  pOR  = 0x3a, // stORe, adORn, fORtune
  pAR  = 0x3b, // alARm, fARm, gARment
  pYR  = 0x3c, // clEAR, EARring, IResponsible
  pXR  = 0x2f, // repaiR, haiR, declaRE, staRE
 
  // Resonants
  pWW  = 0x2e, // Wool, We, lingUIst
  pRR1 = 0x0e, // Rural, WRite, x-Ray (initial position)
  pRR2 = 0x27, // bRain, bRown, gRease (initial clusters)
  pLL  = 0x2d, // Lake, Like, heLLo, steeL
  pYY1 = 0x31, // cUte, bEAUty, compUter (clusters)
  pYY2 = 0x19, // Yes, Yarn, Yo-Yo (initial position)
 
  // Voiced Fricatives
  pVV  = 0x23, // Vest, proVE, eVen
  pDH1 = 0x12, // THis, THen, THey (initial position)
  pDH2 = 0x36, // baTHe, baTHing (word-final and between vowels)
  pZZ  = 0x2b, // Zoo, phaSE
  pZH  = 0x26, // aZure, beiGE, pleaSUre
 
  // Voiceless Fricatives
  pFF  = 0x28, // Food (can be doubled for initial position)
  pTH  = 0x1d, // THin (can be doubled for initial position)
  pSS  = 0x37, // Snake, veSt (can be doubled for initial position)
  pSH  = 0x25, // SHip, SHirt, leaSH, naTion
  pHH1 = 0x1b, // He (before front vowels YR,IY,IH,EY,EH,XR,AE)
  pHH2 = 0x39, // Hoe (before back vowels UW,UH,OW,OY,AO,OR,AR)
  pWH  = 0x30, // WHig, WHite, tWenty
   
  // Voiced Stops
  pBB1 = 0x1c, // riB, fiBBer, (in clusters) BLeed, BRown
  pBB2 = 0x3f, // Business, Beast (initial position before vowel)
  pDD1 = 0x15, // coulD, playeD, enD (final position)
  pDD2 = 0x21, // Do, Down, Drain (initial position and clusters)
  pGG1 = 0x24, // Got (before high front vowels YR,IY,IH,EY,EH,XR)
  pGG2 = 0x3d, // GUest, Glue, Green (before high back vowels UW,UH,OW,OY,AX)
  pGG3 = 0x22, // wiG, anGer (before low vowels AE,AW,AY,AR,AA,AO,OR,ER)
   
  // Voiceless Stops
  pPP  = 0x09, // Pow, triP, amPle, Pleasure
  pTT1 = 0x11, // parTs, tesTs, iTs (final cluster before SS)
  pTT2 = 0x0d, // To, TesT, sTreeT (all positions except before final SS)
  pKK1 = 0x2a, // Can't, Cute, Clown, sCream (before front vowels YR,IY,IH,EY,EH,XR,AY,AE,ER,AX and initial clusters)
  pKK2 = 0x29, // speaK, tasK (final position or final cluster)
  pKK3 = 0x08, // Comb, QUick, Crane, sCream (before back vowels UW,UH,OW,OY,OR,AR,AO and initial clusters)
   
  // Affricates
  pCH  = 0x32, // CHurCH, feaTure
  pJH  = 0x0a, // JudGE, inJUre, dodGE
   
  // Nasal
  pMM  = 0x10, // Milk, alarM, aMple
  pNN1 = 0x0b, // thiN, earN (before front and central vowels YR,IY,IH,EY,EH,XR,AE,ER,AX,AW,AY, and final clusters)
  pNN2 = 0x38, // No (before back vowels UH,OW,OY,OR,AR,AA)
  pNG  = 0x2c, // striNG, aNGer, aNchor
};
 

#define ARRAYSIZE 64
const char* phonemes[ARRAYSIZE] = {
	"pPA1 ",
	"pPA2 ",
	"pPA3 ",
	"pPA4 ",
	"pPA5 ",
	"pIH ",
	"pEH ",
	"pAE ",
	"pUH ",
	"pAO ",
	"pAX ",
	"pAA ",
	"pIY ",
	"pEY ",
	"pAY ",
	"pOY ",
	"pUW1 ",
	"pUW2 ",
	"pOW ",
	"pAW ",
	"pEL ",
	"pER1 ",
	"pER2 ",
	"pOR ",
	"pAR ",
	"pYR ",
	"pXR ",
	"pWW ",
	"pRR1 ",
	"pRR2 ",
	"pLL ",
	"pYY1 ",
	"pYY2 ",
	"pVV ",
	"pDH1 ",
	"pDH2 ",
	"pZZ ",
	"pZH ",
	"pFF ",
	"pTH ",
	"pSS ",
	"pSH ",
	"pHH1 ",
	"pHH2 ",
	"pWH ",
	"pBB1 ",
	"pBB2 ",
	"pDD1 ",
	"pDD2 ",
	"pGG1 ",
	"pGG2 ",
	"pGG3 ",
	"pPP ",
	"pTT1 ",
	"pTT2 ",
	"pKK1 ",
	"pKK2 ",
	"pKK3 ",
	"pCH ",
	"pJH ",
	"pMM ",
	"pNN1 ",
	"pNN2 ",
	"pNG "
};
 

String s;
uint8_t p; //phoneme
#define INPUTCHARARRAYSIZE 255
char InputCharArray[INPUTCHARARRAYSIZE]; // string (char array) that holds bytes of incoming string

// read a string from the serial and store it in an array
// this bit of code adapted from WilsonSerialIO.pde 
// http://userwww.sfsu.edu/~swilson/
void readSerialString( char *strArray ) {
	int i = 0;
	if (Serial.available()) {
		//Serial.print("reading Serial String: "); //optional: for confirmation
		while (Serial.available() > 0){
			strArray[i] = Serial.read();
			i++;
			Serial.print( strArray[(i - 1)] ); // for confirmation
		}
		strArray[i] = '\0'; // append a proper null to end of string
		Serial.println(); // for confirmation
	}
}


// write a phoneme to the SPO chip
static void sayPhoneme(uint8_t phoneme) {
  // first wait for LRQ to assert low, so there is room in the SPO's FIFO
  while (digitalRead(SPO_LRQ));
  // write the phoneme to A1-A6
  digitalWrite(SPO_A1,(phoneme>>0)&1 ? HIGH : LOW);
  digitalWrite(SPO_A2,(phoneme>>1)&1 ? HIGH : LOW);
  digitalWrite(SPO_A3,(phoneme>>2)&1 ? HIGH : LOW);
  digitalWrite(SPO_A4,(phoneme>>3)&1 ? HIGH : LOW);
  digitalWrite(SPO_A5,(phoneme>>4)&1 ? HIGH : LOW);
  digitalWrite(SPO_A6,(phoneme>>5)&1 ? HIGH : LOW);
  // pulse ALD low for between 200ns and 1100 ns
  
  digitalWrite( SPO_ALD, LOW );//  digitalWrite( SPO_ALD, HIGH );

  delayMicroseconds(5); // when checked on a scope we are producing pulses around 4.8usec wide just from the overhead of digitalWrite(), so no explict additional delay is needed
  digitalWrite(SPO_ALD,HIGH); // digitalWrite( SPO_ALD, LOW );
  // make sure ALD stays high at least 1.1 usec before it pulses low again. we do this crudely
  delayMicroseconds(2); // like above, we don't actually need to enforce this that strongly because digitalWrite() is so slow
}

// Converte un carattere in un fonema basandosi anche sul carattere successivo c1
// se c1 � muto (es 'h') skipNext � messo a 1 altrimenti vale 0
uint8_t CharToPhoneme( char c ,char c1, int &skipNext)
{
	// c= carattere da pronunciare
	// c1 carattere successivo 
	skipNext =0; //inizializzo
	if(c1=='\0')
	{
		c1 = ' ';
	}
	uint8_t p;
	Serial.print( c );
	switch (c)
	{
		case ' ':   return pPA3;

		case 'a': case 'A': p = pAX; break;
		case 'b': case 'B': p = pBB2; break;
		case 'c': case 'C':
			
			switch (c1)
			{
			case 'i':case 'I' :case 'e': case 'E' :// C dolce
				p = pCH; break;
			case 'o':case 'O':		// uso la k che suona meglio
				p = pKK3; break;
			case 'h':case 'H':		// uso la k che suona meglio
				p = pKK1; 
				skipNext = 1;
				break;
			default:	//c dura
				p = pKK2; break;
			}			
			
			p = pCH; break;
		case 'd': case 'D': p = pDD2; break;
		case 'e': case 'E': p = pEH; break;
		case 'f': case 'F': p = pFF; break;
		case 'g': case 'G': 
			switch (c1){
				case 'i':case 'I' :case 'e': case 'E' :// g dolce
					p = pJH; break;
				case 'h':case 'H' :// g dura
					p = pGG1;
					
					break;
				default:
					p = pGG1; break;				
			}
		
			

		case 'i': case 'I': p = pIY; break;
		case 'j': case 'J': p = pJH; break;
		case 'k': case 'K': p = pKK1; break;
		case 'l': case 'L': p = pLL; break;
		case 'm': case 'M': p = pMM; break;
		case 'n': case 'N': 
			p = pNN1; break;
		case 'o': case 'O':
			//switch (c1){
			//case 'i':case 'I': 
			//	p = pOY; break;
			//default:
			//	
			//}
			p = pOW; break; 
		case 'p': case 'P': p = pPP; break;
		case 'q': case 'Q': p = pKK3; break;
		case 'r': case 'R': p = pRR1; break;		//anche RR2
		case 's': case 'S': 
			switch (c1){
			case 'c':case 'C': 
				p = pSH; 
				skipNext = 1;
				break;
			default:
				p = pSS; break;
			}

			//p = pSS; break;
		case 't': case 'T':
			//switch (c1){
			//case 't':case 'T':case 's':case 'S':
			//	p = pTT1; break;
			//case 'h': case 'H':
			//	p = pTH; break;
			//default:
				p = pTT2; break;
			//}

			//p = pTH; break;
		case 'u': case 'U': p = pUW2; break;
		case 'v': case 'V': p = pVV; break;
		case 'z': case 'Z': p = pZZ; break;

	default:
		p = pPA3;//pausa
		break;

	}
	return p;
};

/// Pronuncia una frase contenuta in InputCharArray[]
void speakArrayOfChar( char *strArray ){
	int i = 0;	//indice array
	int skipNext = 0; //indica quanti caratteri successivi saltare es. nel caso della sequenza '..ch..' ritorna 1 perch� la h deve salterla 
	uint8_t p; //phoneme
	char c; char c1;
	while (strArray[i] != '\0'){
		c = strArray[i];
		c1 = strArray[i + 1];

		p = CharToPhoneme( c, c1, skipNext );
		i += skipNext;
		skipNext = 0;

		sayPhoneme(p  );
		//Serial.println( phonemes[p] );
		i++;
	}
	sayPhoneme( pPA5 );

}


void setup() {
	Serial.begin(9600);
	Serial.println( "SP0256-AL2 Terminal" );
 
	// configure LRQ from the SPO as an input
 	
	pinMode(SPO_LRQ,INPUT_PULLUP); //pinMode(SPO_LRQ,INPUT);
	// write A1-A6 low and configure them as output
	pinMode(SPO_A1,OUTPUT);
	pinMode(SPO_A2,OUTPUT);
	pinMode(SPO_A3,OUTPUT);
	pinMode(SPO_A4,OUTPUT);
	pinMode(SPO_A5,OUTPUT);
	pinMode(SPO_A6,OUTPUT);
	digitalWrite(SPO_A1,LOW);
	digitalWrite(SPO_A2,LOW);
	digitalWrite(SPO_A3,LOW);
	digitalWrite(SPO_A4,LOW);
	digitalWrite(SPO_A5,LOW);
	digitalWrite(SPO_A6,LOW);
	// write ALD high (idle)
	pinMode(SPO_ALD,OUTPUT);
	digitalWrite(SPO_ALD,HIGH);


	// note that the above operation may have caused a low->high transition, but since the SPO latches on a high->low transition this edge should be ignored by the SPO
 
#ifdef DRIVE_CLOCK_USING_ATMEL // set to 1 to generate the clock signal using a timer on the arduino.
  // I need a approx 3.14 MHz signal to generate a clock for the SPO
  // so I use timer 2 to generate as high a freq signal as I can and feed that into the OSC1 input of the SPO.
  // running timer 2 in CTC mode with OCR2A = 0 means it flips as fast as possible
  // makes the OC2A output toggle at f_IO/(2*N*(1+OCR2A), where N=prescalar
  // by setting N(prescalar)=1 and OCR2A=2 I'll get a 16MHz/6 = 2.66MHz signal, which I hope is enough
  // (the other possibility is 4MHz)
  // first setup so OC2A output is sent to the OC2A pin (pin 17 on the 28-pin DIP package)
  // pin 17 is MOSI/OC2A/PCINT3, or port B bit 3
  pinMode(SPO_OSC1,OUTPUT); // equivalent to DDRB |= (1<<3);
  // make sure timer2 is powered (it should be, but just in case)
  PRR &= ~(/*PRTIM2*/1<<6);
  // NOTE: Port B bit 3 on an arduino uno maps to the board pin 11 (digital)
  // then configure timer 2 to togging OC2A at as close to the desired frequency as we can get
  TCCR2B = (/*CS2=stopped*/0<<0); // keep the clock stopped as we configure the timer
  TIMSK2 = 0; // no interrupts
  TCNT2 = 0; // reset count to 0
  ASSR = 0; // use internal CLK_IO clock source
  OCR2A = 2; // 1 -> 4MHz, 2 -> 2.66MHz on pin OC2A
  OCR2B = 0xff; // to keep it out of the way
  // NOTE WELL: observation of the OC2A pin with an oscilloscope show that I am not reading the manual right, or the manual isn't right
  // If I set WGM2=2 (CTC mode) then OC2A toggles at 8MHz, no matter the value of OCR2A.
  // And if I set WGM2=7 (Fast PWM mode) then OC2A toggles at 16MHz/(2*(1+OCR2A) as would be expected in CTC mode
  // Since the latter mode turns out to generate the signal I want (2.66MHz) I use it, even though I don't fully understand why it works
  TCCR2A = (/*COM2A=toggle OC2A on match*/1<<6) + (/*WGM2=CTC*/3<<0);
  TCCR2B = (/*WGM2*/1<<3) + (/*CS2=CLK_IO (prescalar=1)*/1<<0); // and upper bit of WGM2 is 0; note that the timer starts running once prescalar is configured, so we do this last
  TIFR2 = TIFR2; // toss away any accidental pending interrupts (not that it matters, just being neat & tidy)
#endif
 
#ifdef DRIVE_RESET_USING_ATMEL // set to 1 to drive the resets from the arduino
  // hold SPO_RESET low for 100 msec (way more than would be needed), then pull it high
  pinMode(SPO_RESET,OUTPUT);
  digitalWrite(SPO_RESET,LOW);
  delay(100);
  digitalWrite(SPO_RESET,HIGH);
#endif
 
	// and give the SPO time to get initialized
	delay(1000);
	// sayPhoneme(pHH1);sayPhoneme(pAE);sayPhoneme( pLL);sayPhoneme(pLL );sayPhoneme(pAW );
	s =  " Hello   io  sono  arduino";
	
	//s.toCharArray( InputCharArray, 256 );
	//speakArrayOfChar( InputCharArray );
//	speakString( s );

	speakString("k");
	Serial.setTimeout( 1000 );

}

void loop() {
	while(!Serial.available()>0){}
	s = "";
	s = Serial.readString();
	if (s!="")
	{
		s.toCharArray( InputCharArray, INPUTCHARARRAYSIZE );
		if (strlen( InputCharArray ) == 1){
			speakCodedSentence( InputCharArray[0] );
		}
		else //stringa da pronunciare
		{
			speakString( s );
			//Serial.println( s );
		}		
	}

}
void speakString( String s ){
	// 1 Converto la stringa in ingresso in un array di caratteri
		//2 pronuncio la sequenza
	s.toCharArray( InputCharArray, INPUTCHARARRAYSIZE );
		speakArrayOfChar( InputCharArray );



}
void speakCodedSentence( char c ){
	// pronuncia una frase codificata da un singolo carattere
	switch (c)
	{
		case 't': test();					break;
		case 'k': speakString("OKEI"); 		break;
		case 'h': speakString("hello"); 		break;
		case 'f': speakString("avanti"); 		break;
		case 'b': speakString("indietro"); 		break;
		case 'l': speakString("sinistra"); 		break;
		case 'r': speakString("destra"); 		break;
		case 'o': speakString("oi oi"); 		break;

	default:
		break;
	}
 


}
/// pronuncia il vocabolario 
void test(){
	speakString( F("OKEI ") );
	speakString( F("no ") );
	speakString( F("no okei") );
	speakString( F("oi oi") );
	speakString( F("chi sei ") );
	speakString( F("ki sei ") );
};



