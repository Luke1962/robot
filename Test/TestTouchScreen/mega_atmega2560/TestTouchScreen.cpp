// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

#include <stdint.h>
//#include "TouchScreen.h"


#include <SPFD5408_TouchScreen/SPFD5408_TouchScreen.h> 



// Init LCD----------------------------------------------------------
//#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
//#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// Dimensions
#include "arduino.h"

void setup(void);
void loop(void);
uint16_t width = 0;
uint16_t height = 0;
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4 // Optional : otherwise connect to Arduino's reset pin
#define LCD_ROTATION 0

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

//----------------------------------------------------------------




//   T O U C H S C R E E N   ----------------------------------------------------------------
// These are the pins for the shield!
#define YP A1  // A1 must be an analog pin, use "An" notation!
#define XM A2  // A2 must be an analog pin, use "An" notation!
#define YM 7   // 7 can be a digital pin
#define XP 6   // 6 can be a digital pin

#define MINPRESSURE 10
#define MAXPRESSURE 240

short TS_MINX=726;
short TS_MAXX=947;
short TS_MINY=420;
short TS_MAXY=435;

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

void setup(void) {
  Serial.begin(9600);
  Serial.println("\n Test of TouchScreen  ");
  TSPoint p;


  //Serial.println("\n letture prima dell'uso del TFT  ");
  //for (int i = 0; i < 50; i++)
  //{
	 // p = getPressPosition();
	 // Serial.print("x "); Serial.print(p.x);
	 // Serial.print("\ty "); Serial.print(p.y);
	 // Serial.print("\tPressure = "); Serial.println(p.z);
	 // delay(1000);
  //}




  /*
  tft.reset();
  uint16_t identifier = 0x9341;		// tft.readID();
  tft.begin(identifier);
  tft.setRotation(LCD_ROTATION); // Need for the Mega, please changed for your choice or rotation initial
  drawBorder();	// Border

  tft.setCursor (55, 50);
  tft.setTextSize (3);
  tft.setTextColor(RED);
  tft.println("SPFD5408");
  tft.setCursor (65, 85);
  tft.println("Library");
  tft.setCursor (25, 150);
  tft.setTextSize (2);
  tft.setTextColor(BLACK);
  tft.println("Test of BITMAP");
  delay(1000);
*/
}

void loop(void) {
  // a point object holds x y and z coordinates
  TSPoint p= ts.getPoint();
  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!

     Serial.print("\nx "); Serial.print(p.x);
     Serial.print("\ty "); Serial.print(p.y);
     Serial.print("\tPressure = "); Serial.print(p.z);
	 if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
		Serial.print(" * ");

		// Show touch screen point (TSPOINT)  
		//showTouched(p);

	  }  

  delay(500);
}

/*
// Show the coordinates     
void showTouched(TSPoint p) {

  uint8_t w = 40; // Width
  uint8_t h = 10; // Heigth
  uint8_t x = (width - (w*2)); // X
  uint8_t y = 11; // Y
  
  tft.fillRect(x, y, w*2, h, WHITE); // For cleanup

  tft.drawRect(x, y, w, h, RED); // For X
  tft.drawRect(x+w+2, y, w*2, h, RED); // For Y

  tft.setTextColor(BLACK);
  tft.setCursor(x+2, y + 1);
  tft.print("X: ");
  showValue(p.x);
  
  tft.setCursor(x+2+w+2, y + 1);
  tft.print("Y: ");
  showValue(p.y);

}

// Show a value of TSPoint
void showValue (uint16_t value) {

  if (value < 10)
    tft.print("00");
  if (value < 100)
    tft.print("0");
    
  tft.print(value);
  
}
void drawBorder() {

	// Draw a border

	uint16_t width = tft.width() - 1;
	uint16_t height = tft.height() - 1;
	uint8_t border = 5;

	tft.fillScreen(BLUE);
	tft.fillRect(border, border, (width - border * 2), (height - border * 2), WHITE);

}
TSPoint getPressPosition()
{

	TSPoint p, q;
	int upCount = 0;

	// Wait for screen press
	do
	{
		q = ts.getPoint();
		Serial.print(q.x); Serial.print(","); Serial.print(q.y); Serial.print(",");Serial.println(q.z);
		delay(10);
	} while (q.z < MINPRESSURE || q.z > MAXPRESSURE);

	// Save initial touch point
	p.x = q.x; p.y = q.y; p.z = q.z;

	// Wait for finger to come up
	do
	{
		q = ts.getPoint();
		if (q.z < MINPRESSURE || q.z > MAXPRESSURE)
		{
			upCount++;
		}
		else
		{
			upCount = 0;
			p.x = q.x; p.y = q.y; p.z = q.z;
		}

		delay(10);             // Try varying this delay

	} while (upCount < 10);  // and this count for different results.

	p.x = map(p.x, TS_MINX, TS_MAXX, 0, 239);
	p.y = map(p.y, TS_MINY, TS_MAXY, 0, 319);

	// I've been focused on rotation 3, with the USB connector on the right-hand side
	// so, you'll have to add your own code here.
	switch (LCD_ROTATION)
	{
	case 3:
		swap(p.x, p.y);
		p.x = (320 - p.x);
		break;
	}

	// Clean up pin modes for LCD
	pinMode(XM, OUTPUT);
	digitalWrite(XM, LOW);
	pinMode(YP, OUTPUT);
	digitalWrite(YP, HIGH);
	pinMode(YM, OUTPUT);
	digitalWrite(YM, LOW);
	pinMode(XP, OUTPUT);
	digitalWrite(XP, HIGH);

	return p;
}


*/

