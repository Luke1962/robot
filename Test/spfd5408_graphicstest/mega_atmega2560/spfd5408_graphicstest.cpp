// IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.

// Modified for SPFD5408 Library by Joao Lopes
// Version 0.9.2 - Rotation for Mega and screen initial

// *** SPFD5408 change -- Begin
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>
// *** SPFD5408 change -- End

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



#include "arduino.h"

void setup(void);
void loop(void);
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
void progmemPrint(const char *str);
void progmemPrintln(const char *str);
void drawhomeicon();
void drawbatt();
void yled( int xled );
void redraw();
void clearcenter();
void clearsettings();
void homescr();
void menu1();
void menu2();
void menu3();
void menu4();
void menu5();
void settingsscr();
void sleepinc();
void sleepdec();
void showsleep();
void option3down();
void option3up();
void m1b1action();
void m1b2action();
void m1b3action();
void m1b4action();
void m1b5action();
void m1b6action();
void m2b1action();
void m2b2action();
void m2b3action();
void m2b4action();
void m2b5action();
void m2b6action();
void m3b1action();
void m3b2action();
void m3b3action();
void m3b4action();
void m3b5action();
void m3b6action();
void m4b1action();
void m4b2action();
void m4b3action();
void m4b4action();
void m4b5action();
void m4b6action();
void m5b1action();
void m5b2action();
void m5b3action();
void m5b4action();
void m5b5action();
void m5b6action();
void blightup();
void blightdown();
void blbar();
void ant();
void boxes();
void signal();
void signalact();
void drawhomeiconred();
void clearmessage();
long readVcc();
int i = 0;
int page = 0;
int blv;
int sleep = 0;
int pulsev = 0;
int redflag = 0;
int greenflag = 0;
int redled = 2;
int greenled = A4;
int backlight = 3;
int battfill;
unsigned long sleeptime;
unsigned long battcheck = 10000; // the amount of time between voltage check and battery icon refresh
unsigned long prevbatt;
int battv;
int battold;
int battpercent;
int barv;
int prevpage;
int sleepnever;
int esleep;
int backlightbox;
int antpos = 278;
unsigned long awakeend;
unsigned long currenttime;
unsigned long ssitime;
char voltage[10];
char battpercenttxt[10];



Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;

// -- Setup

void setup(void) {
  
  Serial.begin(9600);
  
  progmemPrintln(PSTR("TFT LCD test"));

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  progmemPrintln(PSTR("Using Adafruit 2.8\" TFT Arduino Shield Pinout"));
#else
  progmemPrintln(PSTR("Using Adafruit 2.8\" TFT Breakout Board Pinout"));
#endif

  tft.reset();
 
  // *** SPFD5408 change -- Begin

// Original code commented

//  uint16_t identifier = tft.readID();
//
//  if(identifier == 0x9325) {
//    Serial.println(F("Found ILI9325 LCD driver"));
//  } else if(identifier == 0x9328) {
//    Serial.println(F("Found ILI9328 LCD driver"));
//  } else if(identifier == 0x7575) {
//    Serial.println(F("Found HX8347G LCD driver"));
//  } else if(identifier == 0x9341) {
//    Serial.println(F("Found ILI9341 LCD driver"));
//  } else if(identifier == 0x8357) {
//    Serial.println(F("Found HX8357D LCD driver"));
//  } else {
//    Serial.print(F("Unknown LCD driver chip: "));
//    Serial.println(identifier, HEX);
//    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
//    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
//    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
//    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
//    Serial.println(F("Also if using the breakout, double-check that all wiring"));
//    Serial.println(F("matches the tutorial."));
//    return;
//  }
//
//  tft.begin(identifier);

  // Code changed to works 
  
  tft.begin(0x9341); // SDFP5408

  tft.setRotation(0); // Need for the Mega, please changed for your choice or rotation initial

  // *** SPFD5408 change -- End

  progmemPrintln(PSTR("Benchmark                Time (microseconds)"));

  progmemPrint(PSTR("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  progmemPrint(PSTR("Text                     "));
  Serial.println(testText());
  delay(3000);

  progmemPrint(PSTR("Lines                    "));
  Serial.println(testLines(CYAN));
  delay(500);

  progmemPrint(PSTR("Horiz/Vert Lines         "));
  Serial.println(testFastLines(RED, BLUE));
  delay(500);

  progmemPrint(PSTR("Rectangles (outline)     "));
  Serial.println(testRects(GREEN));
  delay(500);

  progmemPrint(PSTR("Rectangles (filled)      "));
  Serial.println(testFilledRects(YELLOW, MAGENTA));
  delay(500);

  progmemPrint(PSTR("Circles (filled)         "));
  Serial.println(testFilledCircles(10, MAGENTA));

  progmemPrint(PSTR("Circles (outline)        "));
  Serial.println(testCircles(10, WHITE));
  delay(500);

  progmemPrint(PSTR("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  progmemPrint(PSTR("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  progmemPrint(PSTR("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  progmemPrint(PSTR("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  progmemPrintln(PSTR("Done!"));
}
void loop(void) {

	tft.fillScreen( BLACK );
	tft.setRotation( 1 );
	tft.fillRect( 71, 70, 50, 100, JJCOLOR );
	tft.fillRect( 134, 70, 50, 100, JJCOLOR );
	tft.fillRect( 197, 70, 50, 100, JJCOLOR );
	tft.drawRect( 46, 45, 228, 150, WHITE );
	for (i = 0; i <= blv; i += 1) {
		analogWrite( backlight, i );
		delay( 2 );
	}
	delay( 250 );
	tft.setCursor( 85, 100 );
	tft.setTextSize( 5 );
	tft.setTextColor( WHITE );
	tft.print( "J" );
	delay( 250 );
	tft.setCursor( 147, 100 );
	tft.print( "O" );
	delay( 250 );
	tft.setCursor( 210, 100 );
	tft.print( "S" );
	delay( 500 );
	tft.setCursor( 84, 210 );
	tft.setTextSize( 1 );
	tft.print( "Jeremy Saglimbeni  -  2012" );
	tft.setCursor( 108, 230 );
	tft.print( "thecustomgeek.com" );
	delay( 500 );
	tft.fillScreen( BLACK );
	tft.fillRect( 0, 0, 320, 10, JJCOLOR ); // status bar
	drawhomeicon(); // draw the home icon
	tft.setCursor( 1, 1 );
	tft.print( "Your status bar message here.    JOS 1.5 Beta" );
	tft.drawRect( 297, 1, 20, 8, WHITE ); //battery body
	tft.fillRect( 317, 3, 2, 4, WHITE ); // battery tip
	tft.fillRect( 298, 2, 18, 6, BLACK ); // clear the center of the battery
	drawbatt();
	ant(); // draw the bas "antenna" line without the "signal waves"
	signal(); // draw the "signal waves" around the "antenna"
	homescr(); // draw the homescreen
	tft.drawRect( 0, 200, 245, 40, WHITE ); // message box
	pinMode( 13, OUTPUT );


  for(uint8_t rotation=0; rotation<4; rotation++) {
    tft.setRotation(rotation);
    testText();
    delay(2000);
  }
}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(BLACK);
  tft.fillScreen(RED);
  tft.fillScreen(GREEN);
  tft.fillScreen(BLUE);
  tft.fillScreen(BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(BLACK);
  unsigned long start = micros();
  tft.setTextColor(WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  tft.fillScreen(BLACK);

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  tft.fillScreen(BLACK);

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  tft.fillScreen(BLACK);

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(0, 0, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
  }

  return micros() - start;
}

// Copy string from flash to serial port
// Source string MUST be inside a PSTR() declaration!
void progmemPrint(const char *str) {
  char c;
  while(c = pgm_read_byte(str++)) Serial.print(c);
}

// Same as above, with trailing newline
void progmemPrintln(const char *str) {
  progmemPrint(str);
  Serial.println();
}

void drawhomeicon() { // draws a white home icon
	tft.drawLine( 280, 219, 299, 200, WHITE );
	tft.drawLine( 300, 200, 304, 204, WHITE );
	tft.drawLine( 304, 203, 304, 200, WHITE );
	tft.drawLine( 305, 200, 307, 200, WHITE );
	tft.drawLine( 308, 200, 308, 208, WHITE );
	tft.drawLine( 309, 209, 319, 219, WHITE );
	tft.drawLine( 281, 219, 283, 219, WHITE );
	tft.drawLine( 316, 219, 318, 219, WHITE );
	tft.drawRect( 284, 219, 32, 21, WHITE );
	tft.drawRect( 295, 225, 10, 15, WHITE );
}

void drawbatt() {
	battv = readVcc(); // read the voltage
	if (battv < battold) { // if the voltage goes down, erase the inside of the battery
		tft.fillRect( 298, 2, 18, 6, BLACK );
	}
	battfill = map( battv, 3000, 4850, 2, 18 ); // map the battery voltage 3000 nis the low, 4150 is the high
	if (battfill > 7) { // if the battfill value is between 8 and 18, fill with green
		tft.fillRect( 298, 2, battfill, 6, GREEN );
	}
	else { // if the battfill value is below 8, fill with red
		tft.fillRect( 298, 2, battfill, 6, RED );
	}
	battold = battv; // this helps determine if redrawing the battfill area is necessary
}

void yled( int xled ) { // "flashes" the "yellow" LED
	for (i = xled; i >= 0; i -= 1) {
		digitalWrite( greenled, LOW );
		digitalWrite( redled, HIGH );
		delay( 1 );
		digitalWrite( greenled, HIGH );
		digitalWrite( redled, LOW );
		delay( 1 );
	}
	digitalWrite( greenled, LOW );
	if (greenflag == 1) {
		digitalWrite( redled, LOW );
		digitalWrite( greenled, HIGH );
	}
	if (redflag == 1) {
		digitalWrite( greenled, LOW );
		digitalWrite( redled, HIGH );
	}
}
void redraw() { // redraw the page
	if ((prevpage != 6) || (page != 7)) {
		clearcenter();
	}
	if (page == 0) {
		homescr();
	}
	if (page == 1) {
		menu1();
	}
	if (page == 2) {
		menu2();
	}
	if (page == 3) {
		menu3();
	}
	if (page == 4) {
		menu4();
	}
	if (page == 5) {
		menu5();
	}
	if (page == 6) {
		settingsscr();
	}
}
void clearcenter() { // the reason for so many small "boxes" is that it's faster than filling the whole thing
	tft.drawRect( 0, 20, 150, 50, BLACK );
	tft.drawRect( 170, 20, 150, 50, BLACK );
	tft.drawRect( 0, 80, 150, 50, BLACK );
	tft.drawRect( 170, 80, 150, 50, BLACK );
	tft.drawRect( 0, 140, 150, 50, BLACK );
	tft.drawRect( 170, 140, 150, 50, BLACK );
	tft.fillRect( 22, 37, 106, 16, BLACK );
	tft.fillRect( 192, 37, 106, 16, BLACK );
	tft.fillRect( 22, 97, 106, 16, BLACK );
	tft.fillRect( 192, 97, 106, 16, BLACK );
	tft.fillRect( 22, 157, 106, 16, BLACK );
	tft.fillRect( 192, 157, 106, 16, BLACK );
}
void clearsettings() { // this is used to erase the extra drawings when exiting the settings page
	tft.fillRect( 0, 20, 320, 110, BLACK );
	delay( 500 );
	clearmessage();
}
void homescr() {
	tft.setTextColor( WHITE );
	tft.setTextSize( 2 );
	boxes();
	tft.setCursor( 41, 37 );
	tft.print( "Menu 1" );
	tft.setCursor( 210, 37 );
	tft.print( "Menu 2" );
	tft.setCursor( 41, 97 );
	tft.print( "Menu 3" );
	tft.setCursor( 210, 97 );
	tft.print( "Menu 4" );
	tft.setCursor( 41, 157 );
	tft.print( "Menu 5" );
	tft.setCursor( 200, 157 );
	tft.print( "Settings" );
}
void menu1() {
	tft.setTextColor( WHITE );
	tft.setTextSize( 2 );
	boxes();
	tft.setCursor( 22, 37 );
	tft.print( "Menu 1 B1" );
	tft.setCursor( 192, 37 );
	tft.print( "Menu 1 B2" );
	tft.setCursor( 22, 97 );
	tft.print( "Menu 1 B3" );
	tft.setCursor( 192, 97 );
	tft.print( "Menu 1 B4" );
	tft.setCursor( 22, 157 );
	tft.print( "Menu 1 B5" );
	tft.setCursor( 192, 157 );
	tft.print( "Menu 1 B6" );
}
void menu2() {
	tft.setTextColor( WHITE );
	tft.setTextSize( 2 );
	boxes();
	tft.setCursor( 22, 37 );
	tft.print( "Menu 2 B1" );
	tft.setCursor( 192, 37 );
	tft.print( "Menu 2 B2" );
	tft.setCursor( 22, 97 );
	tft.print( "Menu 2 B3" );
	tft.setCursor( 192, 97 );
	tft.print( "Menu 2 B4" );
	tft.setCursor( 22, 157 );
	tft.print( "Menu 2 B5" );
	tft.setCursor( 192, 157 );
	tft.print( "Menu 2 B6" );
}
void menu3() {
	tft.setTextColor( WHITE );
	tft.setTextSize( 2 );
	boxes();
	tft.setCursor( 22, 37 );
	tft.print( "Menu 3 B1" );
	tft.setCursor( 192, 37 );
	tft.print( "Menu 3 B2" );
	tft.setCursor( 22, 97 );
	tft.print( "Menu 3 B3" );
	tft.setCursor( 192, 97 );
	tft.print( "Menu 3 B4" );
	tft.setCursor( 22, 157 );
	tft.print( "Menu 3 B5" );
	tft.setCursor( 192, 157 );
	tft.print( "Menu 3 B6" );
}
void menu4() {
	tft.setTextColor( WHITE );
	tft.setTextSize( 2 );
	boxes();
	tft.setCursor( 22, 37 );
	tft.print( "Menu 4 B1" );
	tft.setCursor( 192, 37 );
	tft.print( "Menu 4 B2" );
	tft.setCursor( 22, 97 );
	tft.print( "Menu 4 B3" );
	tft.setCursor( 192, 97 );
	tft.print( "Menu 4 B4" );
	tft.setCursor( 22, 157 );
	tft.print( "Menu 4 B5" );
	tft.setCursor( 192, 157 );
	tft.print( "Menu 4 B6" );
}
void menu5() {
	tft.setTextColor( WHITE );
	tft.setTextSize( 2 );
	boxes();
	tft.setCursor( 22, 37 );
	tft.print( "Menu 5 B1" );
	tft.setCursor( 192, 37 );
	tft.print( "Menu 5 B2" );
	tft.setCursor( 22, 97 );
	tft.print( "Menu 5 B3" );
	tft.setCursor( 192, 97 );
	tft.print( "Menu 5 B4" );
	tft.setCursor( 22, 157 );
	tft.print( "Menu 5 B5" );
	tft.setCursor( 192, 157 );
	tft.print( "Menu 5 B6" );
}
void settingsscr() {
	// backlight level
	tft.setTextColor( WHITE );
	tft.setTextSize( 3 );
	tft.fillRect( 0, 20, 60, 50, RED );
	tft.drawRect( 0, 20, 60, 50, WHITE );
	tft.drawRect( 80, 20, 160, 50, JJCOLOR );
	tft.fillRect( 260, 20, 60, 50, GREEN );
	tft.drawRect( 260, 20, 60, 50, WHITE );
	tft.setCursor( 22, 33 );
	tft.print( "-" );
	tft.setCursor( 282, 33 );
	tft.print( "+" );
	tft.setTextSize( 1 );
	tft.setCursor( 120, 31 );
	tft.print( "Backlight Level" );
	tft.drawRect( 110, 48, 100, 10, WHITE );
	blbar();
	// sleep time
	tft.setTextSize( 3 );
	tft.fillRect( 0, 80, 60, 50, RED );
	tft.drawRect( 0, 80, 60, 50, WHITE );
	tft.drawRect( 80, 80, 160, 50, JJCOLOR );
	tft.fillRect( 260, 80, 60, 50, GREEN );
	tft.drawRect( 260, 80, 60, 50, WHITE );
	tft.setCursor( 22, 93 );
	tft.print( "-" );
	tft.setCursor( 282, 93 );
	tft.print( "+" );
	tft.setTextSize( 1 );
	tft.setCursor( 130, 91 );
	tft.print( "Sleep Time" );
	showsleep();
	//?? uncomment this if you want a third adjustable option
	/*
	tft.fillRect(0, 140, 60, 50, RED);
	tft.drawRect(0, 140, 60, 50, WHITE);
	tft.drawRect(80, 140, 160, 50, JJCOLOR);
	tft.fillRect(260, 140, 60, 50, GREEN);
	tft.drawRect(260, 140, 60, 50, WHITE);
	tft.print(22, 153, "-", WHITE, 3);
	tft.print(130, 151, "Thing #3", WHITE);
	tft.print(282, 153, "+", WHITE, 3);
	tft.drawRect(110, 168, 100, 10, WHITE);
	*/
	battv = readVcc(); // read the voltage
	itoa( battv, voltage, 10 );
	tft.setTextColor( YELLOW );
	tft.setTextSize( 2 );
	tft.setCursor( 12, 213 );
	tft.print( voltage );
	tft.setCursor( 60, 213 );
	tft.print( "mV" );
	/*
	battpercent = (battv / 5000) * 100, 2;
	itoa (battpercent, battpercenttxt, 10);
	tft.print(102, 213, battpercenttxt, YELLOW, 2);
	*/
}
void sleepinc() { // sleep increese adjustment
	if (sleeptime == 14400000) {
		sleepnever = 1;
		esleep = 12;
		sleeptime = 11111111;
		showsleep();
	}
	if (sleeptime == 3600000) {
		sleeptime = 14400000;
		esleep = 11;
		showsleep();
	}
	if (sleeptime == 1800000) {
		sleeptime = 3600000;
		esleep = 10;
		showsleep();
	}
	if (sleeptime == 1200000) {
		sleeptime = 1800000;
		esleep = 9;
		showsleep();
	}
	if (sleeptime == 600000) {
		sleeptime = 1200000;
		esleep = 8;
		showsleep();
	}
	if (sleeptime == 300000) {
		sleeptime = 600000;
		esleep = 7;
		showsleep();
	}
	if (sleeptime == 120000) {
		sleeptime = 300000;
		esleep = 6;
		showsleep();
	}
	if (sleeptime == 60000) {
		sleeptime = 120000;
		esleep = 5;
		showsleep();
	}
	if (sleeptime == 30000) {
		sleeptime = 60000;
		esleep = 4;
		showsleep();
	}
	if (sleeptime == 20000) {
		sleeptime = 30000;
		esleep = 3;
		showsleep();
	}
	if (sleeptime == 10000) {
		sleeptime = 20000;
		esleep = 2;
		showsleep();
	}
	delay( 350 );
}
void sleepdec() { // sleep decreese adjustment
	if (sleeptime == 20000) {
		sleeptime = 10000;
		esleep = 1;
		showsleep();
	}
	if (sleeptime == 30000) {
		sleeptime = 20000;
		esleep = 2;
		showsleep();
	}
	if (sleeptime == 60000) {
		sleeptime = 30000;
		esleep = 3;
		showsleep();
	}
	if (sleeptime == 120000) {
		sleeptime = 60000;
		esleep = 4;
		showsleep();
	}
	if (sleeptime == 300000) {
		sleeptime = 120000;
		esleep = 5;
		showsleep();
	}
	if (sleeptime == 600000) {
		sleeptime = 300000;
		esleep = 6;
		showsleep();
	}
	if (sleeptime == 1200000) {
		sleeptime = 600000;
		esleep = 7;
		showsleep();
	}
	if (sleeptime == 1800000) {
		sleeptime = 1200000;
		esleep = 8;
		showsleep();
	}
	if (sleeptime == 3600000) {
		sleeptime = 1800000;
		esleep = 9;
		showsleep();
	}
	if (sleeptime == 14400000) {
		sleeptime = 3600000;
		esleep = 10;
		showsleep();
	}
	if (sleepnever == 1) {
		sleeptime = 14400000;
		sleepnever = 0;
		esleep = 11;
		showsleep();
	}
	delay( 350 );
}
void showsleep() { // shows the sleep time on the settings page
	tft.fillRect( 110, 108, 80, 10, BLACK );
	tft.setTextSize( 1 );
	tft.setTextColor( WHITE );
	if (sleeptime == 10000) {
		tft.setCursor( 130, 108 );
		tft.print( "10 Seconds" );
	}
	if (sleeptime == 20000) {
		tft.setCursor( 130, 108 );
		tft.print( "20 Seconds" );
	}
	if (sleeptime == 30000) {
		tft.setCursor( 130, 108 );
		tft.print( "30 Seconds" );
	}
	if (sleeptime == 60000) {
		tft.setCursor( 136, 108 );
		tft.print( "1 Minute" );
	}
	if (sleeptime == 120000) {
		tft.setCursor( 133, 108 );
		tft.print( "2 Minutes" );
	}
	if (sleeptime == 300000) {
		tft.setCursor( 133, 108 );
		tft.print( "5 Minutes" );
	}
	if (sleeptime == 600000) {
		tft.setCursor( 130, 108 );
		tft.print( "10 Minutes" );
	}
	if (sleeptime == 1200000) {
		tft.setCursor( 130, 108 );
		tft.print( "20 Minutes" );
	}
	if (sleeptime == 1800000) {
		tft.setCursor( 130, 108 );
		tft.print( "30 Minutes" );
	}
	if (sleeptime == 3600000) {
		tft.setCursor( 142, 108 );
		tft.print( "1 Hour" );
	}
	if (sleeptime == 14400000) {
		tft.setCursor( 139, 108 );
		tft.print( "4 Hours" );
	}
	if (sleepnever == 1) {
		tft.setCursor( 133, 108 );
		tft.print( "Always On" );
	}
}
void option3down() { // adjust option 3 down in the settings screen
}
void option3up() { // adjust option 3 up in the settings screen
}
//custom defined actions - this is where you put your button functions
void m1b1action() {
	signal();
}
void m1b2action() {
	signalact();
}
void m1b3action() {
}
void m1b4action() {
}
void m1b5action() {
}
void m1b6action() {
}
void m2b1action() {
}
void m2b2action() {
}
void m2b3action() {
}
void m2b4action() {
}
void m2b5action() {
}
void m2b6action() {
}
void m3b1action() {
}
void m3b2action() {
}
void m3b3action() {
}
void m3b4action() {
}
void m3b5action() {
}
void m3b6action() {
}
void m4b1action() {
}
void m4b2action() {
}
void m4b3action() {
}
void m4b4action() {
}
void m4b5action() {
}
void m4b6action() {
}
void m5b1action() {
}
void m5b2action() {
}
void m5b3action() {
}
void m5b4action() {
}
void m5b5action() {
}
void m5b6action() {
}
void blightup() { // increase the backlight brightness
	blv = blv + 5;
	if (blv >= 255) {
		blv = 255;
	}
	analogWrite( backlight, blv );
	blbar();
}
void blightdown() { // decrease the backlight brightness
	blv = blv - 5;
	if (blv <= 5) {
		blv = 5;
	}
	analogWrite( backlight, blv );
	blbar();
}
void blbar() { // this function fills the yellow bar in the backlight brightness adjustment
	if (blv < barv) {
		tft.fillRect( 111, 49, 98, 8, BLACK );
	}
	backlightbox = map( blv, 1, 255, 0, 98 );
	tft.fillRect( 111, 49, backlightbox, 8, YELLOW );
	barv = blv;
	delay( 25 );
}
void ant() {
	tft.fillRect( (antpos + 5), 4, 1, 6, WHITE ); // draws the "antenna" for the signal indicator
}
void boxes() { // redraw the button outline boxes
	tft.drawRect( 0, 20, 150, 50, JJCOLOR );
	tft.drawRect( 170, 20, 150, 50, JJCOLOR );
	tft.drawRect( 0, 80, 150, 50, JJCOLOR );
	tft.drawRect( 170, 80, 150, 50, JJCOLOR );
	tft.drawRect( 0, 140, 150, 50, JJCOLOR );
	tft.drawRect( 170, 140, 150, 50, JJCOLOR );
}
void signal() { // draws a whit 'signal indicator'
	tft.drawLine( (antpos + 4), 4, (antpos + 4), 5, WHITE );
	tft.drawPixel( (antpos + 3), 2, WHITE );
	tft.drawPixel( (antpos + 3), 7, WHITE );
	tft.drawPixel( (antpos + 2), 0, WHITE );
	tft.drawLine( (antpos + 2), 3, (antpos + 2), 6, WHITE );
	tft.drawPixel( (antpos + 2), 9, WHITE );
	tft.drawPixel( (antpos + 1), 1, WHITE );
	tft.drawPixel( (antpos + 1), 8, WHITE );
	tft.drawLine( antpos, 2, antpos, 7, WHITE );
	tft.drawLine( (antpos + 6), 4, (antpos + 6), 5, WHITE );
	tft.drawPixel( (antpos + 7), 2, WHITE );
	tft.drawPixel( (antpos + 7), 7, WHITE );
	tft.drawPixel( (antpos + 8), 0, WHITE );
	tft.drawLine( (antpos + 8), 3, (antpos + 8), 6, WHITE );
	tft.drawPixel( (antpos + 8), 9, WHITE );
	tft.drawPixel( (antpos + 9), 1, WHITE );
	tft.drawPixel( (antpos + 9), 8, WHITE );
	tft.drawLine( (antpos + 10), 2, (antpos + 10), 7, WHITE );
}
void signalact() { // draws a red'signal indicator'
	tft.drawLine( (antpos + 4), 4, (antpos + 4), 5, RED );
	tft.drawPixel( (antpos + 3), 2, RED );
	tft.drawPixel( (antpos + 3), 7, RED );
	tft.drawPixel( (antpos + 2), 0, RED );
	tft.drawLine( (antpos + 2), 3, (antpos + 2), 6, RED );
	tft.drawPixel( (antpos + 2), 9, RED );
	tft.drawPixel( (antpos + 1), 1, RED );
	tft.drawPixel( (antpos + 1), 8, RED );
	tft.drawLine( antpos, 2, antpos, 7, RED );
	tft.drawLine( (antpos + 6), 4, (antpos + 6), 5, RED );
	tft.drawPixel( (antpos + 7), 2, RED );
	tft.drawPixel( (antpos + 7), 7, RED );
	tft.drawPixel( (antpos + 8), 0, RED );
	tft.drawLine( (antpos + 8), 3, (antpos + 8), 6, RED );
	tft.drawPixel( (antpos + 8), 9, RED );
	tft.drawPixel( (antpos + 9), 1, RED );
	tft.drawPixel( (antpos + 9), 8, RED );
	tft.drawLine( (antpos + 10), 2, (antpos + 10), 7, RED );
}
void drawhomeiconred() { // draws a red home icon
	tft.drawLine( 280, 219, 299, 200, RED );
	tft.drawLine( 300, 200, 304, 204, RED );
	tft.drawLine( 304, 203, 304, 200, RED );
	tft.drawLine( 305, 200, 307, 200, RED );
	tft.drawLine( 308, 200, 308, 208, RED );
	tft.drawLine( 309, 209, 319, 219, RED );
	tft.drawLine( 281, 219, 283, 219, RED );
	tft.drawLine( 316, 219, 318, 219, RED );
	tft.drawRect( 284, 219, 32, 21, RED );
	tft.drawRect( 295, 225, 10, 15, RED );
}
void clearmessage() {
	tft.fillRect( 12, 213, 226, 16, BLACK ); // black out the inside of the message box
}
long readVcc() {
	long result;
	// Read 1.1V reference against AVcc
	ADMUX = _BV( REFS0 ) | _BV( MUX3 ) | _BV( MUX2 ) | _BV( MUX1 );
	delay( 2 ); // Wait for Vref to settle
	ADCSRA |= _BV( ADSC ); // Convert
	while (bit_is_set( ADCSRA, ADSC ));
	result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result; // Back-calculate AVcc in mV
	return result;
}


