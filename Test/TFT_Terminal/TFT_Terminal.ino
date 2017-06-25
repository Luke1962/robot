
/*************************************************************
  This sketch implements a simple serial receive terminal
  program for monitoring serial debug messages from another
  board.
  
  Connect GND to target board GND
  Connect RX line to TX line of target board
  Make sure the target and terminal have the same baud rate
  and serial stettings!

  The sketch works with the ILI9341 TFT 240x320 display and
  the called up libraries.
  
  The sketch uses the hardware scrolling feature of the
  display. Modification of this sketch may lead to problems
  unless the ILI9341 data sheet has been understood!

  Written by Alan Senior 15th February 2015
  
  BSD license applies, all text above must be included in any
  redistribution
 *************************************************************/

// In most cases characters don't get lost at 9600 baud but
// it is a good idea to increase the serial Rx buffer from 64
// to 512 or 1024 bytes especially if higher baud rates are
// used (this sketch does not need much RAM).
// The method described here works well:
// http://www.hobbytronics.co.uk/arduino-serial-buffer-size
//

#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
//#include <SPFD5408_TouchScreen.h>     // Touch library

//#include <HC05\HC05.h>
//#define BT_ENABLEPIN 42
//#define BT_STATEPIN 40
//#define LED_CMDBT 13
//#define BT_BAUD  9600
//// default Passkey: “1234”
//
//HC05 btserial = HC05(BT_ENABLEPIN, BT_STATEPIN);  // cmd, state

//#define SERIAL_IN btserial
#define SERIAL_IN Serial
#define SERIAL_BAUD_RATE 115200		//115200


 // For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).
//   D0 connects to digital pin 22
//   D1 connects to digital pin 23
//   D2 connects to digital pin 24
//   D3 connects to digital pin 25
//   D4 connects to digital pin 26
//   D5 connects to digital pin 27
//   D6 connects to digital pin 28
//   D7 connects to digital pin 29
 
 
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define LCD_X_WIDTH 240
#define LCD_Y_HEIGHT 320
#define LCD_TEXT_HEIGHT 16 // Height of text to be printed and scrolled
#define LCD_TEXT_WIDTH 10

#define LCD_TEXTCOLOR MAGENTA
#define LCD_BACKGROUND BLACK
#define LCD_TEXTSIZE 2		//20 righe di 24 caratteri (19righe utili se uso la prima come stato con size =1)
#define LCD_CHAR_COLUMNS 24
// The scrolling area must be a integral multiple of LCD_TEXT_HEIGHT
#define LCD_BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
#define LCD_TOP_FIXED_AREA 24 // 16Number of lines in top fixed area (lines counted from top of screen)
# define LCD_SCROLL_ROWS 5

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void tftTextXY(String txt, int16_t x = 0, int16_t y = 1, uint8_t size = 1, int color = BLACK) {
 
	//calcola quante linee occupa il testo
	int Lines = 1 + (int)txt.length() / LCD_CHAR_COLUMNS;
	//pulisce la riga
	tft.fillRect(LCD_TEXTSIZE * LCD_TEXT_WIDTH * x, LCD_TEXT_HEIGHT*y + LCD_TOP_FIXED_AREA, LCD_X_WIDTH, Lines*LCD_TEXT_HEIGHT, LCD_BACKGROUND);

	tft.setCursor(x, y);
	tft.setTextSize(size);
	tft.setTextColor(color);
	tft.println(txt);
}


void setup() {
	// Setup baud rate and draw top banner
	Serial.begin(SERIAL_BAUD_RATE);

	// Setup the TFT display
	tft.reset();
	tft.begin(0x9341); // SDFP5408
	//tft.init();
	tft.setRotation(0);
	tft.fillScreen(BLACK);

	// intestazione --------------------
	tft.fillRect(0,0,240,16, BLUE);
	tft.setTextColor(WHITE, BLUE);
	tftTextXY("Terminal", 1, 0,1,WHITE);


	// Change colour for scrolling zone
	tft.setTextColor(LCD_TEXTCOLOR, LCD_BACKGROUND);
	tft.setTextSize(LCD_TEXTSIZE);


	//Serial.print("Waiting BT ..."); 
	//while (!btserial.connected());
	//Serial.println("..OK Connected");
	tft.drawPixel(1,1, BLUE);
}

 

// ##############################################################################################
// Visualizza la stringa txt alla riga r e colonna c col colore 'color'
// ritorna la riga utile successiva in funzione della lunghezza del testo
// ##############################################################################################
int tftTextRowCol(String txt, int16_t r = 1, int16_t c = 1,  int color = BLACK) {

	//calcola quante linee occupa il testo
	int Lines = 1 + (int)txt.length() / LCD_CHAR_COLUMNS;

	// se stampandolo eccede il numero di righe disponibili, riparte dalla riga 1
	if ( (r + Lines) > (LCD_SCROLL_ROWS+1)){		r= 1;	}

	//pulisce la riga
	tft.fillRect( LCD_TEXTSIZE * LCD_TEXT_WIDTH * (c - 1), LCD_TEXT_HEIGHT*(r - 1) + LCD_TOP_FIXED_AREA, LCD_X_WIDTH, Lines*LCD_TEXT_HEIGHT, LCD_BACKGROUND);

	//posiziona il cursore
	tft.setCursor(LCD_TEXTSIZE * LCD_TEXT_WIDTH * (c - 1), LCD_TEXT_HEIGHT*(r - 1) + LCD_TOP_FIXED_AREA);

	//stampa il testo
	tft.setTextSize(LCD_TEXTSIZE);
	tft.setTextColor(color);	
	tft.println(txt);

	//aggiorno la riga
	return r + Lines;
}

 int rowcnt = 1;// riga corrente

void loop(void) {

	while (!SERIAL_IN.available()) { delay(10); }	//wait for serial data
													
	String strIn = SERIAL_IN.readString();

	rowcnt = tftTextRowCol(strIn,rowcnt, 2,LCD_TEXTCOLOR);
 }





