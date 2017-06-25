/// adattamento da http://myarduinoproject.weebly.com/past-projects.html
#include <SPI.h>
#include <Camera\Camera.h>


#pragma region LIBRERIE ILI9341_due tft 
#include <Wire.h>

#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion

// If we are using the hardware SPI interface, these are the pins (for future ref)
#define sclk 4
#define mosi 5
//#define cs 6  //Soft SPI
#define cs 10
#define dc 7
#define rst 8  // you can also connect this to the Arduino reset

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF


//OV7670 cam = OV7670(11,2,12,44,45,41,3,13);
OV7670 cam = OV7670(52,42,24,44,45,41,3,13);
// vsync,  hrf,  we,  rt, OE, xclock,   rs,   WRST


// information we extract about the bitmap file
int bmpWidth, bmpHeight;
uint8_t bmpDepth, bmpImageoffset;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup(void) {
  int repeat=2;
  Serial.begin(9600);
  Serial3.begin(9600);
    inputString.reserve(200);
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS,HIGH);


  tft.fillScreen(BLUE);
  tft.setTextColor(WHITE);
  tft.printAt("Camera Test",0,52);
  delay(3000);


  // Init cam
  cam.Init();
  cam.InitQQVGA();
}

int mouse_x=80;
int mouse_y=60;
int mouse_pix=0;

#define STRIDE  32

int read_frame()
{
	int i,j,k;
	uint16_t pix1,pix2,pix,npix;
	uint16_t p[STRIDE];
	int time;
  
	uint8_t rotback = tft.getRotation();
  
	time = millis();
	//Serial.print("Get Rotation ");
	//Serial.println(rotback,HEX);
	rotback = iliRotation0;
	tft.setRotation(iliRotation90);
	tft.setAddrWindow(0, 0, 159, 119);
	cam.ReadStart();
	for(i=0;i<120;i++){
		for(j=0;j<160;j+=STRIDE){
			for(k=0;k<STRIDE;k++){
			pix1=cam.ReadOneByte();
			pix2 = cam.ReadOneByte();
			p[k] = (pix2<<8)|pix1;
			if ((i==mouse_y)&&((j+k)==mouse_x)){
				mouse_pix = p[k];
			}
			}
			//tft.pushColorMulti(p,STRIDE);
			tft.pushColor(p[k]);
			//tft.drawPixel(i, j, npix);
			//Serial.print(cam.ReadOneByte(),HEX);
		}
	}
	cam.ReadStop();
	return (millis()-time);
}

#ifdef SD_CARD
void bmpdraw(File f, int x, int y) {
	uint16_t s[STRIDE];

	uint32_t time = millis();
	uint16_t p;
	uint8_t g, b;
	int i, j, k;
	uint8_t sdbuffer[2 * BUFFPIXEL];  // 3 * pixels to buffer
	uint8_t buffidx = 2 * BUFFPIXEL;

	bmpFile.seek(bmpImageoffset);

	Serial.print("rotation = "); Serial.println(tft.getRotation(), HEX);

	tft.setRotation(iliRotation90);
	//set up the 'display window'
	tft.setAddrWindow(x, y, x + bmpWidth - 1, y + bmpHeight - 1);
	//tft.setRotation(0xC8);
	uint8_t rotback = tft.getRotation();


	for (i = 0; i < bmpHeight; i++) {
		// bitmaps are stored with the BOTTOM line first so we have to move 'up'

		for (j = 0; j < bmpWidth; j += STRIDE) {
			for (k = 0; k < STRIDE; k++) {
				// read more pixels
				//if (buffidx >= 2*BUFFPIXEL) {
				//bmpFile.read(sdbuffer, 3*BUFFPIXEL);
				//buffidx = 0;

				sdbuffer[0] = bmpFile.read();
				sdbuffer[1] = bmpFile.read();
				sdbuffer[2] = bmpFile.read();

				//sdbuffer[0] = 0;
				//sdbuffer[1] = 255;
				//sdbuffer[2] = 0;

				//Serial.print(sdbuffer[0], HEX);
				//Serial.print(sdbuffer[1], HEX);
				//Serial.print(sdbuffer[2], HEX);
				//}
				//p = (sdbuffer[1] << 8) | sdbuffer[0];
				//buffidx++;
				//buffidx++;
				// convert pixel from 565 to 888
				//b = sdbuffer[0] & 0x1f;
				//g = (sdbuffer[0] >> 5) | (sdbuffer[1] << 5);
				//r = sdbuffer[1] >> 3;

				// convert pixel from 888 to 565
				p = sdbuffer[0];     // blue
				g = sdbuffer[1];     // green
				b = sdbuffer[2];     // red

				p >>= 3;
				p <<= 6;

				g >>= 2;
				p |= g;
				p <<= 5;

				b >>= 3;
				p |= b;
				//Serial.print(p, HEX);
				// write out the 16 bits of color
				//tft.drawPixel(bmpWidth-j-1, i, p);
				s[k] = p;
			}
			delayMicroseconds(200);
			//tft.setRotation(0xC8);
			tft.setAddrWindow(j + k - STRIDE, i, j + k - 1, i); // Need to do this to workaround interferncewith SPI read/write from SD
			tft.setAddrWindow(j + k - STRIDE, i, j + k - 1, i);

			// was tft.pushColorMulti(s,STRIDE);
			tft.pushColors(s, 0, STRIDE);
		}
	}
	Serial.print(millis() - time, DEC);
	Serial.println(" ms");
}

#endif // SD_CARD

void loop() {
  int time;
  char s[32];
  int start_time,end_time;
  float frame_rate;
  //Serial.println(cam.vsync_ period());
  start_time=millis();
  cam.capture_frame();
  time = read_frame();
  end_time = millis();
  frame_rate=1000.0/(float)(end_time-start_time);
  tft.fillRect(0,120,159,8,BLACK);
  sprintf(s,"%03dms %3.1ffps (%d,%d,%d)",time,frame_rate,
  mouse_pix>>11&0x1F, mouse_pix>>5&0x3F,mouse_pix&0x1F);
  tft.setTextColor(WHITE);
  tft.printAt(s,0,120);
  
  drawMouse();
  
  #if 0
    if (stringComplete) {
      int reg,val;
    Serial.println(inputString);
    sscanf(&inputString[0],"%x %x",&reg,&val);
    cam.WriteReg(reg,val);
    // clear the string:
    inputString = "";
    stringComplete = false;
    }
  #endif
  
  if (Serial.available()){
    switch(Serial.read()){
      case 'a':
        if (mouse_x>1)
          mouse_x--;
          break;
       case 's':
         if (mouse_x<159)
           mouse_x++;
           break;
       case 'w':
         if (mouse_y<119)
           mouse_y++;
           break;
        case 'z':
          if (mouse_y>1)
            mouse_y--;
            break;







 
        case '#': // This is a servo command
        {
          char sc[80];
          int i=0;
          sc[0]='#';
          i++;
          while (Serial.available()){
            sc[i]=Serial.read();
            if (sc[i]==0x0a){
              sc[i++]=0;
              Serial.print(sc);
              Serial3.println(sc);
              break;
            }
            i++;
          }
        }
        break;
      }  
           
    }
  
}



void drawMouse()
{
    if (mouse_x>6)
      tft.drawFastHLine(mouse_x-6,mouse_y,5,WHITE);
    if (mouse_x<154)
      tft.drawFastHLine(mouse_x+1,mouse_y,5,WHITE);
    if (mouse_y>6)
      tft.drawFastVLine(mouse_x,mouse_y-6,5,WHITE);
    if (mouse_y<114)
      tft.drawFastVLine(mouse_x,mouse_y+1,5,WHITE);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void testfastlines(uint16_t color1, uint16_t color2) {
   tft.fillScreen(BLACK);
   for (uint16_t y=0; y < tft.height(); y+=5) {
     tft.drawFastHLine(0, y, tft.width(), color1);
   }
   for (uint16_t x=0; x < tft.width(); x+=5) {
     tft.drawFastVLine(x, 0, tft.height(), color2);
   }
}

/*********************************************/
// This procedure reads a bitmap and draws it to the screen
// its sped up by reading many pixels worth of data at a time
// instead of just one pixel at a time. increading the buffer takes
// more RAM but makes the drawing a little faster. 20 pixels' worth
// is probably a good place

#define BUFFPIXEL 20



/*********************************************/

// These read data from the SD card file and convert them to big endian 
// (the data is stored in little endian format!)

// LITTLE ENDIAN!
uint16_t read16(File f) {
  uint16_t d;
  uint8_t b;
  b = f.read();
  d = f.read();
  d <<= 8;
  d |= b;
  return d;
}


// LITTLE ENDIAN!
uint32_t read32(File f) {
  uint32_t d;
  uint16_t b;
 
  b = read16(f);
  d = read16(f);
  d <<= 16;
  d |= b;
  return d;
}


