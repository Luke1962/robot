// Code provided by Smoke And Wires
// http://www.smokeandwires.co.nz
// This code has been taken from the Adafruit TFT Library and modified
//  by us for use with our TFT Shields / Modules
// For original code / licensing please refer to
// https://github.com/adafruit/TFTLCD-Library
#include "Arduino.h"

// *** SPFD5408 change -- Begin
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

#include "arduino.h"

void setup(void);
void loop(void);
unsigned long testFillScreen();
unsigned long testText();
Adafruit_TFTLCD tft;
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// SWTFT tft;

void setup(void) {

	tft.reset();

	uint16_t identifier = tft.readID();
	tft.begin(0x9341);
}

void loop(void) {
	for (uint8_t rotation = 0; rotation = 1; rotation++) {
		tft.setRotation(rotation);
		testText();
		delay(20000);
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
	tft.setCursor(0, 0);
	tft.setTextColor(WHITE);  tft.setTextSize(1);
	tft.println("Bobs Robotics!");

	tft.setTextColor(GREEN);
	tft.setTextSize(2);
	tft.println("Motor temp ~56 degrees C");
	tft.setTextColor(WHITE);

	tft.println("ESC Temp ~50 degrees C");
	tft.setTextColor(MAGENTA);

	tft.println("Methane Sensor ~20ppm");
	tft.setTextColor(CYAN);

	tft.println("Hydrogen sensor~0.2ppm");
	tft.setTextColor(RED);

	tft.println("Co2 Sensor ~70ppm");
	tft.setTextColor(MAGENTA);

	tft.println("Air pressure ~100011Pa");
	tft.setTextColor(GREEN);

	tft.println("Current temp ~30 degrees C");
	tft.setTextColor(YELLOW);

	tft.println("Current Humidity is 50%");
	tft.setTextColor(CYAN);

	tft.println("Altutude ~45m");
	tft.setTextColor(WHITE);

	tft.println("Voltage remaining ~ 7.6V");
	tft.setTextColor(BLUE);

	tft.println("Current draw ~12A");

	tft.setTextColor(GREEN);
	tft.println("UV Index is 12mW/cm^2");

	tft.setTextColor(RED);
	tft.println("light intensity is 930Lux");

	return micros() - start;
}


