// UTouch_ButtonTest 
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program is a quick demo of how create and use buttons.
//
// This program requires the UTFT library.
//
// It is assumed that the display module is connected to an
// appropriate shield or that you know how to change the pin 
// numbers in the setup.
//


#pragma region Mio TFT 320x480

#include <UTFT.h>
UTFT     tft(ILI9327_8, 38, 39, 40, 41);
// Declare which fonts we will be using
//extern uint8_t BigFont[];
extern uint8_t SmallFont[];
#pragma endregion


// Initialize touchscreen
#pragma region touch screen
#include <SPI.h>
// richiede SPI.h
#define CS_PIN  53	///was 9
#define TIRQ_PIN  2

//#define  TS1	//Touch screen  driver XPT2046_Touchscreen
#define  TS2	//Touch screen  driver XPT2046-2

#ifdef  TS1
#include <XPT2046_Touchscreen\XPT2046_Touchscreen.h>
#define isTouching() touched()
#else
#include <XPT2046-2/XPT2046-2.h>

#endif // TS1



//class TS_Point {
//public:
//	TS_Point(void) : x(0), y(0), z(0) {}
//	TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
//	bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
//	bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
//	uint16_t x, y, z;
//};
//#include <XPT2046\XPT2046.h>


//XPT2046 ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
#ifdef TS1
XPT2046_Touchscreen ts((uint8_t) CS_PIN, (uint8_t) TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
#else
															   
XPT2046 ts((uint8_t) CS_PIN, (uint8_t) TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

#endif // TS1


#pragma endregion

int x, y;
uint16_t xRaw;
uint16_t yRaw;
// Buffer input from TFT keyboard
char stCurrent[20]="";
int stCurrentLen=0;
char stLast[20]="";
#define dispy 480
#define dispx 320
#define BUTT_W 60
#define BUTT_H 26
#define BUTT2_W 120
#define BUTT2_H 28
// R1,2,3 posizione y della mezzeria dei Buttons
#define R1 380		///WAS 10
#define R2 420	/// (R1 + BUTT_H +2)
#define R3 460		/// (R1 + 2*BUTT_H +2)
#define C1 32
#define C2 dispx/2
#define C3 dispx-11
#define C_B1 80		//clear
#define C_B2 250	//enter

#ifdef TS1
#else

static void calibratePoint(uint16_t x, uint16_t y, uint16_t &vi, uint16_t &vj) {
	// Draw cross
	tft.setColor(0xff, 0xff, 0xff);
	tft.drawHLine(x - 8, y, 16);
	tft.drawVLine(x, y - 8, 16);
	while (!ts.isTouching()) {
		delay(10);
	}
	ts.getRaw(vi, vj);
	// Erase by overwriting with black
	tft.setColor(0, 0, 0);
	tft.drawHLine(x - 8, y, 16);
	tft.drawVLine(x, y - 8, 16);
}

void calibrate() {
	uint16_t x1, y1, x2, y2;
	uint16_t vi1, vj1, vi2, vj2;
	ts.getCalibrationPoints(x1, y1, x2, y2);
	calibratePoint(x1, y1, vi1, vj1);
	delay(1000);
	calibratePoint(x2, y2, vi2, vj2);
	ts.setCalibration(vi1, vj1, vi2, vj2);

	char buf[80];
	snprintf(buf, sizeof(buf), "%d,%d,%d,%d", (int)vi1, (int)vj1, (int)vi2, (int)vj2);

	tft.setFont(SmallFont);
	tft.setBackColor(0, 0, 255);
	tft.setColor(0xff, 0xff, 0xff);

	tft.print("setCalibration params:",0,25);

	tft.print(buf,0,50);
}


#endif // TS1

/*************************
**   Custom functions   **
*************************/
void drawButtons()
{
// Draw the upper row of buttons
  for (x=0; x<5; x++)
  {
    tft.setColor(0, 0, 255);
	tft.fillRoundRect(C1 + (x *( BUTT_W)) - BUTT_W/2, R1-BUTT_H/2, C1 + (x * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
	tft.setColor(255, 255, 255);
	tft.drawRoundRect(C1 + (x * (BUTT_W )) - BUTT_W/2, R1-BUTT_H/2, C1 + (x * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
	tft.printNumI(x+1,10+ C1 + (x * (BUTT_W )) - BUTT_W / 2,  R1-5);
  }
// Draw the center row of buttons
  for (x=0; x<5; x++)
  {
	  tft.setColor(0, 0, 255);
	  tft.fillRoundRect(C1 + (x *(BUTT_W )) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (x * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
	  tft.setColor(255, 255, 255);
	  tft.drawRoundRect(C1 + (x * (BUTT_W )) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (x * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
	  tft.printNumI(x + 6, 10 + C1 + (x * (BUTT_W )) - BUTT_W / 2, R2 - 5);
	  
	  //if (x<4)
   //   tft.printNumI(x+6, 27+(x*60), 87);
  }
  //tft.print("0", 267, 87);


// Draw the lower row of buttons CLEAR & ENTER
  tft.setColor(0, 0, 255);
  tft.fillRoundRect(C_B1 - BUTT2_W/2, R3 - BUTT2_H/2 , C_B1 + BUTT2_W/2, R3 + BUTT2_H/2 );
  tft.setColor(255, 255, 255);
  tft.drawRoundRect(C_B1 - BUTT2_W/2, R3 - BUTT2_H/2 , C_B1 + BUTT2_W/2, R3 + BUTT2_H/2 );
  tft.print("Clear", C_B1 - BUTT2_W/2 +4, R3-5);

  tft.setColor(0, 0, 255);
  tft.fillRoundRect(C_B2 - BUTT2_W/2, R3 - BUTT2_H/2 , C_B2 + BUTT2_W/2, R3 + BUTT2_H/2 );
  tft.setColor(255, 255, 255);
  tft.drawRoundRect(C_B2 - BUTT2_W/2, R3 - BUTT2_H/2 , C_B2 + BUTT2_W/2, R3 + BUTT2_H/2);
  tft.print("Enter", C_B2 - BUTT2_W/2 +4, R3-5);
  tft.setBackColor (0, 0, 0);
}
void updateStr(int val)
{
  if (stCurrentLen<20)
  {
    stCurrent[stCurrentLen]=val;
    stCurrent[stCurrentLen+1]='\0';
    stCurrentLen++;
    tft.setColor(0, 255, 0);
    tft.print(stCurrent, LEFT, 224);
  }
  else
  {
    tft.setColor(255, 0, 0);
    tft.print("BUFFER FULL!", CENTER, 192);
    delay(500);
    tft.print("            ", CENTER, 192);
    delay(500);
    tft.print("BUFFER FULL!", CENTER, 192);
    delay(500);
    tft.print("            ", CENTER, 192);
    tft.setColor(0, 255, 0);
  }
}

// Draw a red frame while a button is touched
void waitForIt(int x1, int y1, int x2, int y2)
{
  tft.setColor(255, 0, 0);
  tft.drawRoundRect (x1, y1, x2, y2);
  ///while (ts.dataAvailable())

#ifdef TS1
  while (ts.touched()) ;//attende il rilascio del pulsante

#else
  while (ts.isTouching()) ;//attende il rilascio del pulsante

#endif // TS1



	//	  ts.read();
  tft.setColor(255, 255, 255);
  tft.drawRoundRect (x1, y1, x2, y2);
}

/*************************
**  Required functions  **
*************************/

void setup()
{
// Initial setup
	//Serial.begin(38400);
	SPISettings(500000, MSBFIRST, SPI_MODE0);  // original 4000000, ok 2000000

	// TFT setup-----------------------------
	tft.InitLCD(PORTRAIT);
	tft.clrScr();
	///  ts.setPrecision(PREC_HI);

	tft.setFont(SmallFont);
	tft.setBackColor(10, 10, 10);
	//---------------------------------------


	drawButtons();  

	tft.print("x:", 10, 20);
	tft.print("y:", 10, 40);

	tft.print("raw X:", 10, 70);
	tft.print("raw Y:", 10, 90);
	tft.print("zraw:", 10, 110);

	// Touch Screen initialization---------------------------
	//ts.InitTouch(PORTRAIT);  // include già ts.begin();
	//stampo la calibrazione corrente
	int32_t cx, cy, dx , dy;
	
#ifdef TS1
	ts.begin();
#else
	ts.begin(320, 480);
	ts.setRotation(XPT2046::ROT0);
	ts.setCalibration(209, 1759, 1775, 273);
	ts.getCalibration(cx, cy, dx, dy);

#endif // TS1

//	ts.setCalibration(1830, 2000, 180, 1170);


	tft.setColor(VGA_RED);
	tft.print("cal cx:", 10, 130);
	tft.print("cal cy:", 10, 150);
	tft.print("cal dx:", 10, 170);
	tft.print("cal dy:", 10, 190);

	tft.printNumI(cx, 160, 130, 6);
	tft.printNumI(cy, 160, 150, 6);
	tft.printNumI(dx, 160, 170, 6);
	tft.printNumI(dy, 160, 190, 6);

}

/*
int xpt2046GetCoordinates(int * pX, int * pY)
{
	int i;
	int allX[7], allY[7];
	_xpt2046_get_reading(0xd1);
	_xpt2046_get_reading(0x91);
	for (i = 0; i < 7; i++) {
		allX[i] = _xpt2046_get_reading(0xd1);
		allY[i] = _xpt2046_get_reading(0x91);
	}

	int j;
	for (i = 0; i < 4; i++) {
		for (j = i; j < 7; j++) {
			int temp = allX[i];
			if (temp > allX[j]) {
				allX[i] = allX[j];
				allX[j] = temp;
			}
			temp = allY[i];
			if (temp > allY[j]) {
				allY[i] = allY[j];
				allY[j] = temp;
			}
		}
	}
	_xpt2046_get_reading(0x90);

	if (palReadPad(XPT2046_IRQ_PORT, XPT2046_IRQ_PAD)) {
		return 0;
	}

	*pX = allX[3];
	*pY = allY[3];

	return 1;
}

int xpt2046GetAverageCoordinates(int * pX, int * pY, int nSamples)
{
	int nRead = 0;
	int xAcc = 0, yAcc = 0;
	int x, y;

	while (nRead < nSamples) {
		if (!xpt2046GetCoordinates(&x, &y)) {
			break;
		}
		xAcc += x;
		yAcc += y;
		nRead++;
	}

	if (nRead == 0) {
		return 0;
	}
	*pX = xAcc / nRead;
	*pY = yAcc / nRead;
	return 1;
}
*/

void loop()
{

#ifdef TS1
	
    if (ts.touched())
    {
		TS_Point p = ts.getPoint();
		//ts.readData( &x,  &y,  &z);
#else
    if (ts.isTouching())
    {
		TS_Point p ;

		//ts.getRaw(xRaw, yRaw, XPT2046::MODE_SER, 30);
		//ts.getPosition(p.x, p.y, XPT2046::MODE_SER, 30);
		ts.getRaw(xRaw, yRaw, XPT2046::MODE_DFR, 5);
		ts.getPosition(p.x, p.y, XPT2046::MODE_DFR, 5);
		x = p.x;
		y = p.y;

#endif // TS1



		// visualizzo il punto----
		tft.setColor(VGA_LIME);
		tft.drawPixel(x,y);
		//------------------------------


		// stampo le coordinate x,y --------
		tft.setColor(VGA_LIME);
		tft.printNumI(x, 160,20,6);
		tft.printNumI(y, 160, 40,6);


		// stampo i dati grezzi ----------
		tft.setColor(VGA_BLUE);
		tft.printNumI(xRaw, 160, 70,6);
		tft.printNumI(yRaw, 160, 90,6);
		//tft.printNumI(ts.getZ(), 160, 110,6);


#if 1

#pragma region individuazione del pulsante premuto
		int n = 0;
		if ((y >= R1 - BUTT_H / 2) && (y <= R1 + BUTT_H / 2))  // Upper row
		{
			n = 0;
			if ((x >= C1 - BUTT_W / 2) && (x <= C1 + BUTT_W / 2)) // Button: 1
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				updateStr('1');
			}
			n = 1;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 2
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				updateStr('2');
			}


			n = 2;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 3
			{
				waitForIt(C1 + (n *BUTT_W) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				updateStr('3');
			}


			n = 3;
			if ((x > (C1 + (n *BUTT_W) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 4
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				updateStr('4');
			}


			n = 4;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 5
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				updateStr('5');
			}
		}
		/// PULSANTI DA 6 A 0
		if ((y >= R2 - BUTT_H / 2) && (y <= R2 + BUTT_H / 2))  // Center row
		{
			n = 0;
			if ((x >= C1 - BUTT_W / 2) && (x <= C1 + BUTT_W / 2)) // Button: 6
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				updateStr('6');
			}
			n = 1;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 7
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				updateStr('7');
			}


			n = 2;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 8
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				updateStr('8');
			}


			n = 3;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button:9
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				updateStr('9');
			}


			n = 4;
			if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 0
			{
				waitForIt(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				updateStr('0');
			}
		}
		/// PULSANTI CLEAR E ENTER -----------------------------------------
		if ((y >= R3 - BUTT2_H / 2) && (y <= R3 + BUTT2_H / 2))  // Upper row
		{
			if ((x >= C_B1 - BUTT2_W / 2) && (x <= C_B1 + BUTT2_W / 2))  // Button: Clear
			{
				// waitForIt(10, 130, 150, 180);
				waitForIt(C_B1 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B1 + BUTT2_W / 2, R3 - BUTT2_H / 2);
				stCurrent[0] = '\0';
				stCurrentLen = 0;

				// esegue un clear dell'area del display
				tft.setColor(0, 0, 0);
				tft.fillRect(0, 224, 320, R1 - BUTT_H);
			}
			if ((x >= C_B2 - BUTT2_W / 2) && (x <= C_B2 + BUTT2_W / 2))  // Button: Enter
			{
				//waitForIt(160, 130, 300, 180);
				waitForIt(C_B2 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B2 + BUTT2_W / 2, R3 - BUTT2_H / 2);
				if (stCurrentLen > 0)
				{
					for (x = 0; x < stCurrentLen + 1; x++)
					{
						stLast[x] = stCurrent[x];
					}
					stCurrent[0] = '\0';
					stCurrentLen = 0;
					tft.setColor(0, 0, 0);
					tft.fillRect(0, 208, 319, 239);
					tft.setColor(0, 255, 0);
					tft.print(stLast, LEFT, 208);
				}
				else
				{
					tft.setColor(255, 0, 0);
					tft.print("BUFFER EMPTY", CENTER, 192);
					delay(500);
					tft.print("            ", CENTER, 192);
					delay(500);
					tft.print("BUFFER EMPTY", CENTER, 192);
					delay(500);
					tft.print("            ", CENTER, 192);
					tft.setColor(0, 255, 0);
				}
			}
		}

#pragma endregion


#endif // 0

	}
 
}

