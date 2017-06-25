/********************
Arduino generic menu system
UTFT and UTouch example for arduino due
http://www.r-site.net/?at=//op%5B%40id=%273090%27%5D

Dec.2014 Rui Azevedo - ruihfazevedo(@rrob@)gmail.com
creative commons license 3.0: Attribution-ShareAlike CC BY-SA
This software is furnished "as is", without technical support, and with no
warranty, express or implied, as to its usefulness for any purpose.

Thread Safe: No
Extensible: Yes

UTFT library from:
  http://henningkarlsen.com/electronics/library.php?id=50
UTouch library from:
  http://henningkarlsen.com/electronics/library.php?id=56
*/
//#define DEBUG

// display TFT
#define Pin_TFT_RS 36// 40
#define Pin_TFT_WR 37// 41
#define Pin_TFT_CS 38// 42
#define Pin_TFT_RST 39// 43

// Touch screen
#define Pin_TS_TIRQ  2
#define Pin_TS_CS  3	///was 9
#define Pin_TS_BUSY 4

//----------------------------------------------
// pin da configurare in INPUT -----------------
// N.B. Arduino (Atmega) pins default to inputs, n a high-impedance state. !!!
//----------------------------------------------

#define Pin_ROT_ENCODER_A 2
#define Pin_ROT_ENCODER_B 3
#define Pin_ROT_ENCODER_SWITCH 7



#include <UTFT.h>
//#include <UTouch.h>
#ifdef DEBUG
#include <streamFlow.h>
#endif
//#include <Rotary/Rotary.h>	//https://github.com/brianlow/Rotary

#include <Arduino.h>

#include <ClickEncoder/ClickEncoder.h>
#include <ClickEncoderStream.h> // Quad encoder
#include <TimerOne.h>     // ISR on ClickEncoder

#include <menu.h>
#include <macros.h>
#include <menuUTFT.h>
#include <chainStream.h>// concatenate multiple input streams (this allows adding a button to the encoder)
//#include <menuUTouch.h>
#include <menuFields.h>
 //UTFT myGLCD(CTE28,25,26,27,28);
UTFT     myGLCD(ILI9327_8, Pin_TFT_RS, Pin_TFT_WR, Pin_TFT_CS, Pin_TFT_RST);		//was 38, 39, 40, 41
ClickEncoder qEnc(Pin_ROT_ENCODER_B, Pin_ROT_ENCODER_A, Pin_ROT_ENCODER_SWITCH, 2, LOW);

//extern uint8_t SmallFont[];
extern uint8_t SmallFont[];
//extern uint8_t SevenSegNumFont[];

menuUTFT gfx(myGLCD);

//UTouch  myTouch( 6, 5, 4, 3, 2);
//menuUTouch menuTouch(myTouch,gfx);
/* Quad encoder */
ClickEncoderStream enc(qEnc, 1);// simple quad encoder fake Stream

								//alternative to previous but now we can input from Serial too...
Stream* menuInputs[] = { &enc,&Serial };

//Stream* in2[]={&menuTouch,&Serial};
//Stream* in2[]={&menuInputs,&Serial};
chainStream<2> in(menuInputs);

/////////////////////////////////////////////////////////////////////////
// MENU FUNCTION
// this functions will be wired to menu options
// meaning they will be called on option click/select
// or on field value change/update
bool sayIt(prompt& p,menuOut& o,Stream &c) {
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.clrScr();
  myGLCD.setColor(0, 255, 0);
  myGLCD.print("Activated option:",0,0);
  myGLCD.print(p.text,0,16);
  o.drawn=0;
  delay(1000);
  myGLCD.clrScr();
  return true;
}

int aValue=50;
float fValue=101.1;
/////////////////////////////////////////////////////////////////////////
// MENU DEFINITION
// here we define the menu structure and wire actions functions to it
MENU(subMenu,"Sub-Menu",
  OP("Op1",menu::nothing),
  OP("Op2",menu::nothing),
  OP("Op3",menu::nothing)
);

/*MENU(menuSetup,"Menu config",
  FIELD(myGLCD.*/

MENU(mainMenu,"Sistema",
  OP("Option A",sayIt),
  OP("Option B",sayIt),
  OP("Option C",sayIt),
  OP("Option D",sayIt),
  OP("Option E",sayIt),
  OP("Option F",sayIt),
  FIELD(aValue,"Value","%",0,100,1,0),
  FIELD(fValue,"Value"," Hz",1,100000,100,0.5),
  SUBMENU(subMenu)
);
void timerIsr() {
	qEnc.service();
}

void setup() {
	Serial.begin(115200);
	myGLCD.InitLCD(PORTRAIT);
	myGLCD.setBrightness(4);
	myGLCD.clrScr();
	Serial.println(F("Controlla il menu con + e - dalla seriale"));
 

	//myTouch.InitTouch();
	//myTouch.setPrecision(PREC_MEDIUM);
	// Encoder init
	qEnc.setAccelerationEnabled(false);
	qEnc.setDoubleClickEnabled(true); // must be on otherwise the menu library Hang
									//displayAccelerationStatus();

									// ISR init
	Timer1.initialize(5000); // every 0.05 seconds
	Timer1.attachInterrupt(timerIsr);

	pinMode(13, OUTPUT);

	myGLCD.setFont(SmallFont);
	myGLCD.setColor(0, 255, 0);
	myGLCD.setBackColor(0, 0, 0);

	gfx.init();//setup geometry after myGLCD initialized
	//restrict menu area, for scroll and boundary tests
	gfx.maxX=48; //larghezza max menu
	gfx.maxY=8;  //numero di voci del menu 
	mainMenu.setPosition(1,380);
	mainMenu.data[1]->enabled=false;
	gfx.bgColor=VGA_GRAY;
}

void loop() {
  mainMenu.poll(gfx,in);
  //mainMenu.poll(gfx,enc);
}
