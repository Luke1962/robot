// UTouch_Calibration 
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program can be used to calibrate the touchscreen
// of the display modules.
// This program requires the UTFT library and a touch
// screen module that is compatible with UTFT.
//
// It is assumed that the display module is connected to an
// appropriate shield or that you know how to change the pin 
// numbers in the setup.
//
// Instructions will be given on the display.
//

#include <UTFT.h>
#include <UTouch.h>


// Initialize display
// ------------------
// Set the pins to the correct ones for your development board
// -----------------------------------------------------------
// Standard Arduino Uno/2009 Shield            : <display model>,19,18,17,16
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Due       : <display model>,25,26,27,28
// Teensy 3.x TFT Test Board                   : <display model>,23,22, 3, 4
// ElecHouse TFT LCD/SD Shield for Arduino Due : <display model>,22,23,31,33
//
// Remember to change the model parameter to suit your display module!
UTFT     tft(ILI9327_8, 38, 39, 40, 41);

// Initialize touchscreen
// ----------------------
// Set the pins to the correct ones for your development board
// -----------------------------------------------------------
// Standard Arduino Uno/2009 Shield            : 15,10,14, 9, 8
// Standard Arduino Mega/Due shield            :  6, 5, 4, 3, 2
// CTE TFT LCD/SD Shield for Arduino Due       :  6, 5, 4, 3, 2
// Teensy 3.x TFT Test Board                   : 26,31,27,28,29
// ElecHouse TFT LCD/SD Shield for Arduino Due : 25,26,27,29,30
//
// configurazione shield
//UTouch  ts( 6, 5, 4, 3, 2); ///OK  6, 5, 4, 3, 2
// Configurazione SPI 
UTouch  ts( 52, 53, 51, 50, 2); ///OK  6, 5, 4, 3, 2

									 //UTouch(byte tclk, byte tcs, byte tdin, byte dout, byte irq);

// DEFINISCO I PUNTI DI CALIBRAZIONE
// Define the orientation of the touch screen. Further 
// information can be found in the instructions.
#define TOUCH_ORIENTATION PORTRAIT		///  LANDSCAPE	///
#define TFT_PIXEL_X 320
#define TFT_PIXEL_y 480
#define OFFSET 10
#define R1 420		///WAS 10
#define R2 (R1+(TFT_PIXEL_y-R1)/2)
#define R3 TFT_PIXEL_y-OFFSET
#define C1 OFFSET
#define C2 TFT_PIXEL_X/2
#define C3 (TFT_PIXEL_y-OFFSET-1)




// Declare which fonts we will be using
extern uint8_t SmallFont[];

uint32_t cx, cy;
uint32_t rx[8], ry[8];
uint32_t clx, crx, cty, cby;
float px, py;
int dispx, dispy, text_y_center;
uint32_t calx, caly, cals;
char buf[13];

void setup()
{

  tft.InitLCD(TOUCH_ORIENTATION);
  
  tft.clrScr();
  tft.setFont(SmallFont);

  ts.InitTouch(TOUCH_ORIENTATION);
  ts.setPrecision(PREC_MEDIUM);///Mia aggiunta
  //WAS
  //dispx=tft.getDisplayXSize();
  //dispy=tft.getDisplayYSize();

  dispx=tft.getDisplayXSize();
  dispy = R3 - R1;

  text_y_center=(dispy/2)-6;
  tft.print("getDisplayXSize=", 1, 200);
  tft.setColor(VGA_FUCHSIA);

  tft.print("dispx=", 1, 250);
  tft.print("dispY=", 1, 270);
  tft.printNumI(dispx,200,250);
  tft.printNumI(dispy,200,270);

}

void drawCrossHair(int x, int y)
{
  tft.drawRect(x-10, y-10, x+10, y+10);
  tft.drawLine(x-5, y, x+5, y);
  tft.drawLine(x, y-5, x, y+5);
}

void readCoordinates()
{
  int iter = 5000;
  int failcount = 0;
  int cnt = 0;
  uint32_t tx=0;
  uint32_t ty=0;
  boolean OK = false;
  
  while (OK == false)
  {
    tft.setColor(255, 255, 255);
    tft.print("*  PRESS  *", CENTER, text_y_center);
    while (ts.dataAvailable() == false) {}
    tft.print("*  HOLD!  *", CENTER, text_y_center);
    while ((ts.dataAvailable() == true) && (cnt<iter) && (failcount<10000))
    {
      ts.calibrateRead();
      if (!((ts.TP_X==65535) || (ts.TP_Y==65535)))
      {
        tx += ts.TP_X;
        ty += ts.TP_Y;
        cnt++;
      }
      else
        failcount++;
    }
    if (cnt>=iter)
    {
      OK = true;
    }
    else
    {
      tx = 0;
      ty = 0;
      cnt = 0;
    }
    if (failcount>=10000)
      fail();
  }

  cx = tx / iter;
  cy = ty / iter;
}

void calibrate(int x, int y, int i)
{
  tft.setColor(255, 255, 255);
  drawCrossHair(x,y);
  tft.setBackColor(255, 0, 0);
  readCoordinates();
  tft.setColor(255, 255, 255);
  tft.print("* RELEASE *", CENTER, text_y_center);
  tft.setColor(80, 80, 80);
  drawCrossHair(x,y);
  rx[i]=cx;
  ry[i]=cy;
  while (ts.dataAvailable() == true) {}
}

void waitForTouch()
{
  while (ts.dataAvailable() == true) {}
  while (ts.dataAvailable() == false) {}
  while (ts.dataAvailable() == true) {}
}

void toHex(uint32_t num)
{
  buf[0] = '0';
  buf[1] = 'x';
  buf[10] = 'U';
  buf[11] = 'L';
  buf[12] = 0;
  for (int zz=9; zz>1; zz--)
  {
    if ((num & 0xF) > 9)
      buf[zz] = (num & 0xF) + 55;
    else
      buf[zz] = (num & 0xF) + 48;
    num=num>>4;
  }
}

void startup()
{
  tft.setColor(255, 0, 0);
  tft.fillRect(0, 0, dispx-1, 13);
  tft.setColor(255, 255, 255);
  tft.setBackColor(255, 0, 0);
  tft.drawLine(0, 14, dispx-1, 14);
  tft.print("UTouch Calibration", CENTER, 1);
  tft.setBackColor(0, 0, 0);

  if (dispx==220)
  {  
    tft.print("Use a stylus or something", LEFT, 30);
    tft.print("similar to touch as close", LEFT, 42);
    tft.print("to the center of the", LEFT, 54);
    tft.print("highlighted crosshair as", LEFT, 66);
    tft.print("possible. Keep as still as", LEFT, 78);
    tft.print("possible and keep holding", LEFT, 90);
    tft.print("until the highlight is", LEFT, 102);
    tft.print("removed. Repeat for all", LEFT, 114);
    tft.print("crosshairs in sequence.", LEFT, 126);
    tft.print("Touch screen to continue", CENTER, 162);
  }
  else
  {
    tft.print("INSTRUCTIONS", CENTER, 30);
    tft.print("Use a stylus or something similar to", LEFT, 50);
    tft.print("touch as close to the center of the", LEFT, 62);
    tft.print("highlighted crosshair as possible. Keep", LEFT, 74);
    tft.print("as still as possible and keep holding", LEFT, 86);
    tft.print("until the highlight is removed. Repeat", LEFT, 98);
    tft.print("for all crosshairs in sequence.", LEFT, 110);

    tft.print("Further instructions will be displayed", LEFT, 134);
    tft.print("when the calibration is complete.", LEFT, 146);

    tft.print("Do NOT use your finger as a calibration", LEFT, 170);
    tft.print("stylus or the result WILL BE imprecise.", LEFT, 182);

    tft.print("Touch screen to continue", CENTER, 226);
  }

  waitForTouch();
  tft.clrScr();
}

void done()
{
  tft.clrScr();
  tft.setColor(255, 0, 0);
  tft.fillRect(0, 0, dispx-1, 13);
  tft.setColor(255, 255, 255);
  tft.setBackColor(255, 0, 0);
  tft.drawLine(0, 14, dispx-1, 14);
  tft.print("UTouch Calibration", CENTER, 1);
  tft.setBackColor(0, 0, 0);
  
  if (dispx==220)
  {  
    tft.print("To use the new calibration", LEFT, 30);
    tft.print("settings you must edit the", LEFT, 42);
    tft.setColor(160, 160, 255);
    tft.print("UTouchCD.h", LEFT, 54);
    tft.setColor(255, 255, 255);
    tft.print("file and change", 88, 54);
    tft.print("the following values. The", LEFT, 66);
    tft.print("values are located right", LEFT, 78);
    tft.print("below the opening comment.", LEFT, 90);
    tft.print("CAL_X", LEFT, 110);
    tft.print("CAL_Y", LEFT, 122);
    tft.print("CAL_S", LEFT, 134);
    toHex(calx);
    tft.print(buf, 75, 110);
    toHex(caly);
    tft.print(buf, 75, 122);
    toHex(cals);
    tft.print(buf, 75, 134);
  }
  else
  {  
    tft.print("CALIBRATION COMPLETE", CENTER, 30);
    tft.print("To use the new calibration", LEFT, 50);
    tft.print("settings you must edit the", LEFT, 62);
    tft.setColor(160, 160, 255);
    tft.print("UTouchCD.h", LEFT, 74);
    tft.setColor(255, 255, 255);
    tft.print("file and change", 88, 74);
    tft.print("the following values.", LEFT, 86);
    tft.print("The values are located right", LEFT, 98);
    tft.print("below the opening comment in", LEFT, 110);
    tft.print("the file.", LEFT, 122);
    tft.print("CAL_X", LEFT, 150);
    tft.print("CAL_Y", LEFT, 162);
    tft.print("CAL_S", LEFT, 174);

    toHex(calx);
    tft.print(buf, 75, 150);
    toHex(caly);
    tft.print(buf, 75, 162);
    toHex(cals);
    tft.print(buf, 75, 174);
  }
  
}

void fail()
{
  tft.clrScr();
  tft.setColor(255, 0, 0);
  tft.fillRect(0, 0, dispx-1, 13);
  tft.setColor(255, 255, 255);
  tft.setBackColor(255, 0, 0);
  tft.drawLine(0, 14, dispx-1, 14);
  tft.print("UTouch Calibration FAILED", CENTER, 1);
  tft.setBackColor(0, 0, 0);
  
  tft.print("Unable to read the position", LEFT, 30);
  tft.print("of the press. This is a", LEFT, 42);
  tft.print("hardware issue and can", 88, 54);
  tft.print("not be corrected in", LEFT, 66);
  tft.print("software.", LEFT, 78);
  
  while(true) {};
}

void loop()
{
  startup();
  
  tft.setColor(80, 80, 80);
  // PRIMA  RIGA-------------
  drawCrossHair(dispx-11, R1);
  drawCrossHair(dispx/2, R1);
  drawCrossHair(10, R1);

  // RIGA CENTRALE----------------
  drawCrossHair(dispx-11, R2);
  drawCrossHair(10, R2);

  // RIGA IN BASSO-----------------
  drawCrossHair(dispx-11, R3);
  drawCrossHair(dispx/2, R3);
  drawCrossHair(10, R3);

  tft.setColor(255, 255, 255);
  tft.setBackColor(255, 0, 0);
  tft.print("***********", CENTER, text_y_center-12);
  tft.print("***********", CENTER, text_y_center+12);

  calibrate(C1, R1, 0);
  calibrate(C1, R2, 1);
  calibrate(C1, R3, 2);

  calibrate(C2, R1, 3);
  calibrate(C2, R3, 4);

  calibrate(C3, R1, 5);
  calibrate(C3, R2, 6);
  calibrate(C3, R3, 7);
  
  if (TOUCH_ORIENTATION == LANDSCAPE)
    cals=(long(dispx-1)<<12)+(dispy-1);
  else
    cals=(long(dispy-1)<<12)+(dispx-1);

  if (TOUCH_ORIENTATION == PORTRAIT)
    px = abs(((float(rx[2]+rx[4]+rx[7])/3)-(float(rx[0]+rx[3]+rx[5])/3))/(dispy-20));  // PORTRAIT
  else
	  ///WAS px = abs(((float(rx[5] + rx[6] + rx[7]) / 3) - (float(rx[0] + rx[1] + rx[2]) / 3)) / (dispy - 20));  // LANDSCAPE
	px = abs(((float(rx[5] + rx[6] + rx[7]) / 3) - (float(rx[0] + rx[1] + rx[2]) / 3)) / (C3-C1));  // LANDSCAPE

  if (TOUCH_ORIENTATION == PORTRAIT)
  {
	  //media rx prima riga
    clx = (((rx[0]+rx[3]+rx[5])/3));  // PORTRAIT
	// media rx terza riga
    crx = (((rx[2]+rx[4]+rx[7])/3));  // PORTRAIT
  }
  else
  {
    clx = (((rx[0]+rx[1]+rx[2])/3));  // LANDSCAPE
    crx = (((rx[5]+rx[6]+rx[7])/3));  // LANDSCAPE
  }
  if (clx<crx)
  {
    clx = clx - (px*10);
    crx = crx + (px*10);
  }
  else
  {
    clx = clx + (px*10);
    crx = crx - (px*10);
  }
  
  if (TOUCH_ORIENTATION == PORTRAIT)
	  // py = media ry della prima riga - media py della terza riga; il tutto diviso per i pixel verticali
    py = abs(((float(ry[5]+ry[6]+ry[7])/3)-(float(ry[0]+ry[1]+ry[2])/3))/(R3-R1));  // PORTRAIT
  else
    py = abs(((float(ry[0]+ry[3]+ry[5])/3)-(float(ry[2]+ry[4]+ry[7])/3))/(dispx-20));  // LANDSCAPE

  if (TOUCH_ORIENTATION == PORTRAIT)
  {
    cty = (((ry[5]+ry[6]+ry[7])/3));  // PORTRAIT
    cby = (((ry[0]+ry[1]+ry[2])/3));  // PORTRAIT
  }
  else
  {
    cty = (((ry[0]+ry[3]+ry[5])/3));  // LANDSCAPE
    cby = (((ry[2]+ry[4]+ry[7])/3));  // LANDSCAPE
  }
  if (cty<cby)
  {
    cty = cty - (py*10);
    cby = cby + (py*10);
  }
  else
  {
    cty = cty + (py*10);
    cby = cby - (py*10);
  }
  
  calx = (long(clx)<<14) + long(crx);
  caly = (long(cty)<<14) + long(cby);
  if (TOUCH_ORIENTATION == LANDSCAPE)
    cals = cals + (1L<<31);

  done();
  while(true) {}
}
