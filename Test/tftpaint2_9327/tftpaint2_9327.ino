// This sketch has been Refurbished by BUHOSOFT
// Original code provided by Smoke And Wires
// http://www.smokeandwires.co.nz
// This code has been taken from the Adafruit TFT Library and modified
//  by us for use with our TFT Shields / Modules
// For original code / licensing please refer to
// https://github.com/adafruit/TFTLCD-Library

// adapted sketch by niq_ro from http://arduinotehniq.blogspot.com/
// ver. 1m5 - 13.11.2014, Craiova - Romania



// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
// #define LCD_CS A3 // Chip Select goes to Analog 3
// #define LCD_CD A2 // Command/Data goes to Analog 2
// #define LCD_WR A1 // LCD Write goes to Analog 1
// #define LCD_RD A0 // LCD Read goes to Analog 0

// #define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

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

//#define DEBUG
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4


#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin
//ili9327
//#define XM A1
//#define YP A2
//#define XP 7
//#define YM 6

#define TS_MINX 180 //150,150
#define TS_MINY 180 //75, 120
#define TS_MAXX 920 //920
#define TS_MAXY 950 //950

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
//#define ROZ     0xFD20
#define ROZ     0xFBE0
#define GRI     0xBDF7
// http://stackoverflow.com/questions/13720937/c-defined-16bit-high-color
// http://wiibrew.org/wiki/U16_colors

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);;

//9327 400x240
//#define BOXSIZE 50
//9488
#define BOXSIZE 60
#define PENRADIUS 2
int oldcolor, currentcolor;
int ics;
uint16_t identifier;

void setup(void) {
//#ifdef DEBUG
  Serial.begin(9600);
  Serial.println(F("Paint!"));
//#endif // DEBUG
  tft.reset();

   identifier = tft.readID();

if(identifier == 0x9325) {
//#ifdef DEBUG
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {

    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {

    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {

    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {

    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier == 0x9327) {

    Serial.println(F("Found ILI9327 LCD driver"));

  } else if(identifier == 0x9488) {
    Serial.println(F("Found ILI9488 LCD driver"));
//#endif // DEBUG
    } else {
    #ifdef DEBUG
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.print(F("I try use ILI9341 LCD driver "));
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    #endif // DEBUG
    identifier = 0x9341;
  }


  tft.begin(identifier);

  tft.fillScreen(BLACK);
  if(identifier == 0x9327) {
    tft.fillRect(0, 0, 400, 240, BLACK);
  }
  if(identifier == 0x9488) {
    tft.fillRect(0, 0, 480, 320, BLACK);
  }  else {
    tft.fillRect(0, 0, 320, 240, BLACK); 
  }
  tft.setRotation(3);
  tft.setCursor(62, 100);
  tft.setTextColor(RED);  tft.setTextSize(3);
  tft.println("LCD driver chip: ");
  tft.setCursor(132, 150);
  tft.setTextColor(BLUE);
  tft.println(identifier, HEX);

delay(2000);
tft.fillRect(0, 0, 400, 240, BLACK);

tft.setRotation(0);
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(0, BOXSIZE, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(0, BOXSIZE*2, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(0, BOXSIZE*3, BOXSIZE, BOXSIZE, CYAN);
  tft.fillRect(0, BOXSIZE*4, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(0, BOXSIZE*5, BOXSIZE, BOXSIZE, MAGENTA);
  tft.fillRect(0, BOXSIZE*6, BOXSIZE, BOXSIZE, GRI);
  tft.fillRect(0, BOXSIZE*7, BOXSIZE, BOXSIZE,  ROZ);
//tft.fillRect(BOXSIZE, BOXSIZE, BOXSIZE, BOXSIZE, WHITE);

  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
  currentcolor = RED;


 cifre (WHITE);
  pinMode(13, OUTPUT);
}

#define MINPRESSURE 10
#define MAXPRESSURE 1000

void loop()
{
  digitalWrite(13, HIGH);
  // Recently Point was renamed TSPoint in the TouchScreen library
  // If you are using an older version of the library, use the
  // commented definition instead.
  // Point p = ts.getPoint();
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  //pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  //pinMode(YM, OUTPUT);

  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
#ifdef DEBUG
    Serial.print("X = "); Serial.print(p.x);
    Serial.print("\tY = "); Serial.print(p.y);
    Serial.print("\tPressure = "); Serial.println(p.z);
#endif // DEBUG
if (p.y < (TS_MINY-5)) stergere();
    // scale from 0->1023 to tft.width
    p.x = tft.width()-(map(p.x, TS_MINX, TS_MAXX, tft.width(), 0));
    p.y = tft.height()-(map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
#ifdef DEBUG
    Serial.print("("); Serial.print(p.x);
    Serial.print(", "); Serial.print(p.y);
    Serial.println(")");
#endif // DEBUG
    if (p.x < BOXSIZE) {
       oldcolor = currentcolor;

       if (p.y < BOXSIZE) {
         currentcolor = RED;
         tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*2) {
         currentcolor = YELLOW;
         tft.drawRect(0, BOXSIZE, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*3) {
         currentcolor = GREEN;
         tft.drawRect(0, BOXSIZE*2, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*4) {
         currentcolor = CYAN;
         tft.drawRect(0, BOXSIZE*3, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*5) {
         currentcolor = BLUE;
         tft.drawRect(0, BOXSIZE*4, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*6) {
         currentcolor = MAGENTA;
         tft.drawRect(0,BOXSIZE*5, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*7) {
         currentcolor = GRI;
         tft.drawRect(0,BOXSIZE*6, BOXSIZE, BOXSIZE, WHITE);
//         text (currentcolor);
       } else if (p.y < BOXSIZE*8) {
         currentcolor = ROZ;
         tft.drawRect(0,BOXSIZE*7, BOXSIZE, BOXSIZE, WHITE);
         stergere();
       }

//text (currentcolor);
//cifre (currentcolor);
       if (oldcolor != currentcolor) {
          if (oldcolor == RED) tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
          if (oldcolor == YELLOW) tft.fillRect(0, BOXSIZE, BOXSIZE, BOXSIZE, YELLOW);
          if (oldcolor == GREEN) tft.fillRect(0, BOXSIZE*2, BOXSIZE, BOXSIZE, GREEN);
          if (oldcolor == CYAN) tft.fillRect(0, BOXSIZE*3, BOXSIZE, BOXSIZE, CYAN);
          if (oldcolor == BLUE) tft.fillRect(0, BOXSIZE*4, BOXSIZE, BOXSIZE, BLUE);
          if (oldcolor == MAGENTA) tft.fillRect(0, BOXSIZE*5, BOXSIZE, BOXSIZE, MAGENTA);
          if (oldcolor == GRI) tft.fillRect(0, BOXSIZE*6, BOXSIZE, BOXSIZE, GRI);
          if (oldcolor == ROZ) tft.fillRect(0, BOXSIZE*7, BOXSIZE, BOXSIZE, ROZ);
     cifre (currentcolor);
     }

    }
   // if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
/*
if (((p.y-PENRADIUS) > 5) && ((p.y+PENRADIUS) < tft.height())) {
   tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
    }
*/
if (((p.x-PENRADIUS) > BOXSIZE) && ((p.x+PENRADIUS) < tft.width())) {
   tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
    }

  }
}

void text (int culoare)
{
tft.setRotation(3);

          tft.setCursor(5, 220);
          tft.setTextSize(2);
          tft.println("BUHOSOFT");
tft.setRotation(0);
}

void cifre (int culoare)
{
// number for "buttons"
 tft.setRotation(3);
 tft.setTextColor(culoare);
 tft.setTextSize(3);
 if (identifier == 0x9327) {
  tft.setTextSize(3);
  tft.drawLine(32, 0, 82, 50, WHITE);
  tft.drawLine(32, 50, 82, 0, WHITE);
  tft.setCursor(100, 12); //55, 10//100, 12
  tft.println("1");
  tft.setCursor(150, 12); //95, 10//150, 12
  tft.println("2");
  tft.setCursor(200, 12);  //135, 10//200, 12
  tft.println("3");
  tft.setCursor(250, 12);  //175, 10//250, 12
  tft.println("4");
  tft.setCursor(300, 12);  //215, 10//300, 12
  tft.println("5");
  tft.setCursor(350, 12);  //255, 10//350, 12
  tft.println("6");
  tft.setCursor(400, 12);  //295, 10//400, 12
  tft.println("7");
 }
 if (identifier == 0x9488) {
  tft.setTextSize(4);
  tft.drawLine(0, 0, 60, 60, WHITE);
  tft.drawLine(0, 60, 60, 0, WHITE);
  tft.setCursor(80, 16); //55, 10//100, 12
  tft.println("1");
  tft.setCursor(140, 16); //95, 10//150, 12
  tft.println("2");
  tft.setCursor(200, 16);  //135, 10//200, 12
  tft.println("3");
  tft.setCursor(260, 16);  //175
  tft.println("4");
  tft.setCursor(320, 16);  //215
  tft.println("5");
  tft.setCursor(380, 16);  //255
  tft.println("6");
  tft.setCursor(440, 16);  //295
  tft.println("7");
 }
 else
 {
  tft.setTextSize(3);
  tft.drawLine(0, 0, 40, 40, WHITE);
  tft.drawLine(0, 40, 40, 0, WHITE);
  tft.setCursor(55, 10); //55, 10//100, 12
  tft.println("1");
  tft.setCursor(95, 10); //95, 10//150, 12
  tft.println("2");
  tft.setCursor(135, 10);  //135, 10//200, 12
  tft.println("3");
  tft.setCursor(175, 10);  //175
  tft.println("4");
  tft.setCursor(215, 10);  //215
  tft.println("5");
  tft.setCursor(255, 10);  //255
  tft.println("6");
  tft.setCursor(295, 10);  //295
  tft.println("7");
 }
//tft.setTextColor(culoare);
// tft.setCursor(15, 10);
// tft.println("1");
 

 tft.setRotation(0);
}

void stergere ()
{
    #ifdef DEBUG
      Serial.println("erase");
    #endif // DEBUG
      // press the bottom of the screen to erase
 //     tft.fillRect(0, BOXSIZE, tft.width(), tft.height()-BOXSIZE, BLACK);
 if (identifier == 0x9327) {
   tft.fillRect(0, 0, 240, 400, BLACK);
 }
 else if (identifier == 0x9488) {
   tft.fillRect(0, 0, 320, 480, BLACK);
 }
 else {
   tft.fillRect(0, 0, 240, 320, BLACK);
 }
   tft.setRotation(0);
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(0, BOXSIZE, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(0, BOXSIZE*2, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(0, BOXSIZE*3, BOXSIZE, BOXSIZE, CYAN);
  tft.fillRect(0, BOXSIZE*4, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(0, BOXSIZE*5, BOXSIZE, BOXSIZE, MAGENTA);
  tft.fillRect(0, BOXSIZE*6, BOXSIZE, BOXSIZE, GRI);
  tft.fillRect(0, BOXSIZE*7, BOXSIZE, BOXSIZE,  ROZ);
//tft.fillRect(BOXSIZE, BOXSIZE, BOXSIZE, BOXSIZE, WHITE);

  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
  currentcolor = RED;
     cifre (WHITE);
    }

