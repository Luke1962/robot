
// UTFT_Demo

// This demo was made for modules with a screen resolution 
// of 400x270 pixels.
//
// This program requires the attached UTFT library.
//


#include <Arduino.h>
#include <Streaming.h>   // Serial.print alternative

#include <UTFT\UTFT.h>

// Declare which fonts we will be using
extern uint8_t SmallFont[];
// extern uint8_t LargeFont[];
// extern uint8_t hallfetica_normal[];

// Set the pins to the correct ones for your development shield
// ------------------------------------------------------------

// Arduino Mega:
// -------------------
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Mega      : <display model>,38,39,40,41
//
// Remember to change the model parameter to suit your display module!
UTFT myGLCD(ILI9327_8, 38, 39, 40, 41); //funziona ,anche se non al meglio
//UTFT myGLCD(ILI9488_8, 38, 39, 40, 41); //

//TENTATIVI :	
///UTFT myGLCD(R61581, 38, 39, 40, 41);
///UTFT myGLCD(CTE32HR,38,39,40,41);
///UTFT myGLCD(ILI9481,38,39,40,41);
///UTFT myGLCD(R61581, 38, 39, 40, 41);

// Remember to change the model parameter to suit your display module!
//UTFT myGLCD(ILI9481,38,39,40,41);
//UTFT myGLCD(CTE32HR,38,39,40,41);
//UTFT myGLCD(ILI9486, 38, 39, 40, 41);
//UTFT myGLCD(R61581,38,39,40,41);
//UTFT myGLCD(ILI9327,38,39,40,41);
//UTFT myGLCD(ILI9486, 38, 39, 40, 41);  

//Don't need these
///#define DISABLE_ILI9327_8
//#define DISABLE_ILI9488_8


//#define DISABLE_R61581
#define DISABLE_HX8347A
#define DISABLE_HX8352A
#define DISABLE_ILI9327
#define DISABLE_SSD1289
#define DISABLE_ILI9325C
#define DISABLE_ILI9325D
#define DISABLE_ILI9325D_ALT
#define DISABLE_HX8340B_8
#define DISABLE_HX8340B_S
#define DISABLE_ST7735
#define DISABLE_ST7735_ALT
#define DISABLE_S1D19122
#define DISABLE_PCF8833
#define DISABLE_SSD1963_480
#define DISABLE_SSD1963_800
#define DISABLE_SSD1963_800_ALT
#define DISABLE_S6D1121
#define DISABLE_ILI9481
#define DISABLE_S6D0164
#define DISABLE_ST7735S
#define DISABLE_ILI9341_S4P
#define DISABLE_ILI9341_S5P
#define DISABLE_ILI9486
#define DISABLE_CPLD
#define DISABLE_HX8353C

void setup()
{
	randomSeed(analogRead(0));

	Serial.begin(115200);

	// Setup the LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);
	myGLCD.lcdOn();
	delay(1000);

	//Serial <<  R61581_ALT << endl;
	//Serial <<  myGLCD.getDisplayXSize() << endl;
	//Serial <<  myGLCD.getDisplayYSize() << endl;
	// Serial <<  myGLCD.getxFerMode() << endl;

	// Clear the screen and draw the frame

//	myGLCD.show_color_bar();
//	delay(1000);
//for (size_t i = 0; i < 255; i++) {myGLCD.setBrightness(i);delay(800);}	
	 // 
		//
		//myGLCD.setBackColor(VGA_RED);
		//myGLCD.setColor(VGA_WHITE);
		//myGLCD.print("BRIGHTNESS", LEFT, i);
		////char strVal[7];         //the ASCII of the integer will be stored in this char array
		////itoa(i, strVal, 10); //(integer, yourBuffer, base)

		//// 
		////myGLCD.print(strVal, 100, 224);
		//
	 // 
/*
	myGLCD.setColor(0, 0,255 );
	myGLCD.fillRect(0, 0, 399, 300);


	int buf[478];
	int x, x2;
	int y, y2;
	int r;

	delay(1000);
	// Clear the screen and draw the frame
	myGLCD.clrScr();

	myGLCD.setColor(255, 0, 0);
	myGLCD.fillRect(0, 0, 479, 13);
	myGLCD.setColor(64, 64, 64);
	myGLCD.fillRect(0, 258, 479, 271);
	myGLCD.setColor(255, 255, 255);
	myGLCD.setBackColor(255, 0, 0);
	myGLCD.print("* Universal Color TFT Display Library *", CENTER, 1);
	myGLCD.setBackColor(64, 64, 64);
	myGLCD.setColor(255, 255, 0);
	myGLCD.print("<http://www.RinkyDinkElectronics.com/>", CENTER, 259);

	myGLCD.setColor(0, 0, 255);
	myGLCD.drawRect(0, 14, 479, 257);

	// Draw crosshairs
	myGLCD.setColor(0, 0, 255);
	myGLCD.setBackColor(0, 0, 0);
	myGLCD.drawLine(239, 15, 239, 256);
	myGLCD.drawLine(1, 135, 478, 135);
	for (int i = 9; i<470; i += 10)
		myGLCD.drawLine(i, 133, i, 138);
	for (int i = 15; i<256; i += 10)
		myGLCD.drawLine(237, i, 241, i);

	// Draw sin-, cos- and tan-lines  
	myGLCD.setColor(0, 255, 255);
	myGLCD.print("Sin", 5, 15);
	for (int i = 1; i<478; i++)
	{
		myGLCD.drawPixel(i, 135 + (sin(((i*1.13)*3.14) / 180) * 95));
	}

	myGLCD.setColor(255, 0, 0);
	myGLCD.print("Cos", 5, 27);
	for (int i = 1; i<478; i++)
	{
		myGLCD.drawPixel(i, 135 + (cos(((i*1.13)*3.14) / 180) * 95));
	}

	myGLCD.setColor(255, 255, 0);
	myGLCD.print("Tan", 5, 39);
	for (int i = 1; i<478; i++)
	{
		myGLCD.drawPixel(i, 135 + (tan(((i*1.13)*3.14) / 180)));
	}

	delay(2000);
	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);
	myGLCD.setColor(0, 0, 255);
	myGLCD.setBackColor(0, 0, 0);
	myGLCD.drawLine(239, 15, 239, 256);
	myGLCD.drawLine(1, 135, 478, 135);

	// Draw a moving sinewave
	x = 1;
	for (int i = 1; i<(478 * 20); i++)
	{
		x++;
		if (x == 479)
			x = 1;
		if (i>479)
		{
			if ((x == 239) || (buf[x - 1] == 135))
				myGLCD.setColor(0, 0, 255);
			else
				myGLCD.setColor(0, 0, 0);
			myGLCD.drawPixel(x, buf[x - 1]);
		}
		myGLCD.setColor(0, 255, 255);
		y = 135 + (sin(((i*1.65)*3.14) / 180)*(90 - (i / 100)));
		myGLCD.drawPixel(x, y);
		buf[x - 1] = y;
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some filled rectangles
	for (int i = 1; i<6; i++)
	{
		switch (i)
		{
		case 1:
			myGLCD.setColor(255, 0, 255);
			break;
		case 2:
			myGLCD.setColor(255, 0, 0);
			break;
		case 3:
			myGLCD.setColor(0, 255, 0);
			break;
		case 4:
			myGLCD.setColor(0, 0, 255);
			break;
		case 5:
			myGLCD.setColor(255, 255, 0);
			break;
		}
		myGLCD.fillRect(150 + (i * 20), 46 + (i * 20), 210 + (i * 20), 106 + (i * 20));
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some filled, rounded rectangles
	for (int i = 1; i<6; i++)
	{
		switch (i)
		{
		case 1:
			myGLCD.setColor(255, 0, 255);
			break;
		case 2:
			myGLCD.setColor(255, 0, 0);
			break;
		case 3:
			myGLCD.setColor(0, 255, 0);
			break;
		case 4:
			myGLCD.setColor(0, 0, 255);
			break;
		case 5:
			myGLCD.setColor(255, 255, 0);
			break;
		}
		myGLCD.fillRoundRect(330 - (i * 20), 46 + (i * 20), 270 - (i * 20), 106 + (i * 20));
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some filled circles
	for (int i = 1; i<6; i++)
	{
		switch (i)
		{
		case 1:
			myGLCD.setColor(255, 0, 255);
			break;
		case 2:
			myGLCD.setColor(255, 0, 0);
			break;
		case 3:
			myGLCD.setColor(0, 255, 0);
			break;
		case 4:
			myGLCD.setColor(0, 0, 255);
			break;
		case 5:
			myGLCD.setColor(255, 255, 0);
			break;
		}
		myGLCD.fillCircle(180 + (i * 20), 75 + (i * 20), 30);
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some lines in a pattern
	myGLCD.setColor(255, 0, 0);
	for (int i = 15; i<256; i += 5)
	{
		myGLCD.drawLine(1, i, (i*1.88) - 10, 256);
	}
	myGLCD.setColor(255, 0, 0);
	for (int i = 256; i>15; i -= 5)
	{
		myGLCD.drawLine(478, i, (i*1.88) - 11, 15);
	}
	myGLCD.setColor(0, 255, 255);
	for (int i = 256; i>15; i -= 5)
	{
		myGLCD.drawLine(1, i, 491 - (i*1.88), 15);
	}
	myGLCD.setColor(0, 255, 255);
	for (int i = 15; i<256; i += 5)
	{
		myGLCD.drawLine(478, i, 490 - (i*1.88), 256);
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some random circles
	for (int i = 0; i<150; i++)
	{
		myGLCD.setColor(random(255), random(255), random(255));
		x = 32 + random(416);
		y = 45 + random(178);
		r = random(30);
		myGLCD.drawCircle(x, y, r);
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some random rectangles
	for (int i = 0; i<150; i++)
	{
		myGLCD.setColor(random(255), random(255), random(255));
		x = 2 + random(476);
		y = 16 + random(239);
		x2 = 2 + random(476);
		y2 = 16 + random(239);
		myGLCD.drawRect(x, y, x2, y2);
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	// Draw some random rounded rectangles
	for (int i = 0; i<150; i++)
	{
		myGLCD.setColor(random(255), random(255), random(255));
		x = 2 + random(476);
		y = 16 + random(239);
		x2 = 2 + random(476);
		y2 = 16 + random(239);
		myGLCD.drawRoundRect(x, y, x2, y2);
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	for (int i = 0; i<150; i++)
	{
		myGLCD.setColor(random(255), random(255), random(255));
		x = 2 + random(476);
		y = 16 + random(239);
		x2 = 2 + random(476);
		y2 = 16 + random(239);
		myGLCD.drawLine(x, y, x2, y2);
	}

	delay(2000);

	myGLCD.setColor(0, 0, 0);
	myGLCD.fillRect(1, 15, 478, 256);

	for (int i = 0; i<10000; i++)
	{
		myGLCD.setColor(random(255), random(255), random(255));
		myGLCD.drawPixel(2 + random(476), 16 + random(239));
	}

	delay(2000);

	myGLCD.fillScr(0, 0, 255);
	myGLCD.setColor(255, 0, 0);
	myGLCD.fillRoundRect(160, 70, 319, 169);

	myGLCD.setColor(255, 255, 255);
	myGLCD.setBackColor(255, 0, 0);
	myGLCD.print("That's it!", CENTER, 93);
	myGLCD.print("Restarting in a", CENTER, 119);
	myGLCD.print("few seconds...", CENTER, 132);

	myGLCD.setColor(0, 255, 0);
	myGLCD.setBackColor(0, 0, 255);
	myGLCD.print("Runtime: (msecs)", CENTER, 243);
	myGLCD.printNumI(millis(), CENTER, 258);

	delay(10000);
	*/
}

void loop()
{
	myGLCD.fillCircle(0, 0, 10, VGA_RED);delay(2000);
	myGLCD.fillCircle(0, 320, 10, VGA_GREEN);delay(2000);
	myGLCD.fillCircle(470, 320, 10, VGA_BLUE);delay(2000);
	myGLCD.fillCircle(470,0, 10, VGA_LIME);delay(2000);

	delay(2000);
/*
	myGLCD.show_color_bar();
	delay(1000);
	myGLCD.setColor( 0, 0,250);
	myGLCD.print("* Universal Color TFT Display Library *", CENTER, 1);
	delay(1000);

	for (size_t i = 0; i < 250; i+=50) {
		myGLCD.setBrightness(i);delay(100);
		myGLCD.printNumI(i,100,100,3);

	}
	myGLCD.setColor(1,1,1);
	myGLCD.fillRect(0, 0, 480,320);
	delay(1000);
	myGLCD.setColor( 140, 255,0);
	myGLCD.fillRect(0, 0, 298, 400);
*/
	myGLCD.clrScr();
	delay(1000);

	// Draw some filled, rounded rectangles
	for (int i = 1; i<6; i++)
	{
		switch (i)
		{
		case 1:
			myGLCD.setColor(255, 0, 255);
			break;
		case 2:
			myGLCD.setColor(255, 0, 0);
			break;
		case 3:
			myGLCD.setColor(0, 255, 0);
			break;
		case 4:
			myGLCD.setColor(0, 0, 255);
			break;
		case 5:
			myGLCD.setColor(255, 255, 0);
			break;
		}
		myGLCD.fillRoundRect(330 - (i * 20), 46 + (i * 20), 270 - (i * 20), 106 + (i * 20));
	}
	delay(1000);



	//myGLCD.lcdOff();
	//delay(1000);
	//myGLCD.lcdOn();
	//delay(1000);
 //

	//myGLCD.clrScr();
	//delay(1000);

}
