///*
// An example showing 'ring' analogue meters on a 2.2" ILI9341 TFT 240x320
// colour screen
// 
// The meter graphic function works best with slowly changing values without
// large rapid changes such as environmental parameters (temperature etc)
//
// This example uses the hardware SPI and a board based on the ATmega328
// such as an UNO.  If using a Mega or outher micocontroller then comment out
// the following line:
// #define F_AS_T
// in  the "Adafruit_ILI9341_FAST.h" found within the Adafruit_ILI9341_AS
// library folder. Change pins for other Arduinos to the hardware SPI pins.
//
// Needs Fonts 2, and 4 (also Font 6 if using a large size meter)
//
// Alan Senior 18/3/2015
// */

// These are the connections from the UNO to the display
//#define sclk 13  // Don't change
//#define mosi 11  // Don't change
//#define cs   10  // If cs and dc pin allocations are changed then 
//#define dc   9   // comment out #define F_AS_T line in "Adafruit_ILI9341_FAST.h"
// (which is inside "Adafriit_ILI9341_AS" library)

//#define rst  7  // Can alternatively connect this to the Arduino reset

// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define GREY 0x2104 // Dark grey 16 bit colour

//#include <./Adafruit_GFX_AS/Adafruit_GFX_AS.h>     // Core graphics library
//#include <Adafruit_ILI9341_AS.h> // Fast hardware-specific library
//#include <SPI.h>


//Adafruit_ILI9341_AS tft = Adafruit_ILI9341_AS(cs, dc, rst); // Invoke custom library

uint32_t runTime = -99999;       // time for next update

int reading = 0; // Value to be displayed
int d = 0; // Variable used for the sinewave test waveform


		   // *** SPFD5408 change -- Begin



#pragma region LCD SPFD5408
		   ///codice testato con Arduino UNO 
		   /// non funziona con arduino due

		   // IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
		   // CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
		   // SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.

		   // Modified for SPFD5408 Library by Joao Lopes
		   // Version 0.9.2 - Rotation for Mega and screen initial

		   // *** SPFD5408 change -- Begin
#pragma region LCD SPFD5408

#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
		   //#include <SPFD5408_TouchScreen.h>
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



//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
Adafruit_TFTLCD tft;

// -- Setup


#pragma endregion


void setup(void) {
	Serial.begin(9600);

//	progmemPrintln(PSTR("TFT LCD test"));

  tft.begin();

  tft.setRotation(3);

  tft.fillScreen(BLACK);
}


void loop() {
  if (millis() - runTime >= 2000L) { // Execute every 2s
    runTime = millis();

    // Test with a slowly changing value from a Sine function
    d += 5; if (d >= 360) d = 0;

    // Set the the position, gap between meters, and inner radius of the meters
    int xpos = 0, ypos = 5, gap = 4, radius = 52;

    // Draw meter and get back x position of next meter

    // Test with Sine wave function, normally reading will be from a sensor
    reading = 250 + 250 * sineWave(d+0);
    xpos = gap + ringMeter(reading, 0, 500, xpos, ypos, radius, "mA", GREEN2RED); // Draw analogue meter

    reading = 20 + 30 * sineWave(d+60);
    xpos = gap + ringMeter(reading, -10, 50, xpos, ypos, radius, "degC", BLUE2RED); // Draw analogue meter

    reading = 50 + 50 * sineWave(d + 120);
    ringMeter(reading, 0, 100, xpos, ypos, radius, "%RH", BLUE2BLUE); // Draw analogue meter


    // Draw two more larger meters
    xpos = 20, ypos = 115, gap = 24, radius = 64;

    reading = 1000 + 150 * sineWave(d + 90);
    xpos = gap + ringMeter(reading, 850, 1150, xpos, ypos, radius, "mb", BLUE2RED); // Draw analogue meter

    reading = 15 + 15 * sineWave(d + 150);
    xpos = gap + ringMeter(reading, 0, 30, xpos, ypos, radius, "Volts", GREEN2GREEN); // Draw analogue meter

    // Draw a large meter
    xpos = 40, ypos = 5, gap = 15, radius = 120;
    reading = 175;
    // Comment out above meters, then uncomment the next line to show large meter
    //ringMeter(reading,0,200, xpos,ypos,radius," Watts",GREEN2RED); // Draw analogue meter

  }
}


// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  
  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 4;    // Width of outer ring is 1/4 of radius
  
  int angle = 150;  // Half the sweep angle of meter (300 degrees)

  int text_colour = 0; // To hold the text colour

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

  byte seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
  byte inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring

  // Draw colour blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) {

    // Choose colour from scheme
    int colour = 0;
    switch (scheme) {
      case 0: colour = RED; break; // Fixed colour
      case 1: colour = GREEN; break; // Fixed colour
      case 2: colour = BLUE; break; // Fixed colour
      case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
      case 4: colour = rainbow(map(i, -angle, angle, 63, 127)); break; // Green to red (high temperature etc)
      case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
      default: colour = BLUE; break; // Fixed colour
    }

    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, GREY);
    }
  }

  // Convert value to a string
  char buf[10];
  byte len = 4; if (value > 999) len = 5;
  dtostrf(value, len, 0, buf);

  // Set the text colour to default
  tft.setTextColor(WHITE, BLACK);
  // Uncomment next line to set the text colour to the last segment value!
  // tft.setTextColor(text_colour, ILI9341_BLACK);
  
  // Print value, if the meter is large then use big font 6, othewise use 4
  if (r > 84) tft.drawCentreString(buf, x - 5, y - 20, 6); // Value in middle
  else tft.drawCentreString(buf, x - 5, y - 20, 4); // Value in middle

  // Print units, if the meter is large then use big font 4, othewise use 2
  tft.setTextColor(WHITE, BLACK);
  if (r > 84) tft.drawCentreString(units, x, y + 30, 4); // Units display
  else tft.drawCentreString(units, x, y + 5, 2); // Units display

  // Calculate and return right hand side x coordinate
  return x + r;
}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}

// #########################################################################
// Return a value in range -1 to +1 for a given phase angle in degrees
// #########################################################################
float sineWave(int phase) {
  return sin(phase * 0.0174532925);
}
