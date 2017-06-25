//#############################################################################�
// Arduino DUE + mia Serial Camera + TFT seriale ILI9341
// porting su arduino due della versione demo che funziona con uno + tft shield
//#############################################################################�

//  File SerialCamera_DemoCode_CJ-OV528.ino
//  8/8/2013 Jack Shao
//  Demo code for using seeeduino or Arduino board to cature jpg format
//  picture from seeed serial camera and save it into sd card. Push the
//  button to take the a picture .
//  For more details about the product please check http://www.seeedstudio.com/depot/

#pragma region debug

#define dbg(cha) 	 Serial.println(F(cha));
#define dbg2(t,cha)	 Serial.print(F(t));Serial.println(cha);  
#define dbgHex(b)	 Serial.print(b,HEX);Serial.print(" ");  
//#include <MemoryFree/MemoryFree.h>

#pragma endregion
#pragma region Parametri dell'applicazione'

#define MAXJPGSIZE 10000
#include "arduino.h"

#include "arduino.h"

//
//
void freeMemoryReport();
void convertToDistance(int tftX, int tftY);
void renderJPEG(int xpos, int ypos);
void subtractJPEG(int xpos, int ypos);
void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos);
void jpegInfo();
void clearRxBuf();
void sendCmd(char cmd[6], int cmd_len);
void sendCmdWaitAck(byte c0, byte c1, byte c2, byte c3, byte c4, byte c5);
void cameraSYNC();
bool waitAck(byte cmdId);
void initialize();
void initial(byte c0, byte c1, byte c2, byte imagFormat, byte rawSetting, byte jpegResolution);
void preCapture_orig();
void setPktSize(unsigned int packageSize);
void snapshot(byte c0, byte c1, byte snapshotType, byte c3, byte c4, byte c5);
void Capture_orig();
void GetPicture_toLCD(byte c0, byte c1, byte c2, byte c3, byte c4, byte c5, unsigned int pictureSize_X, unsigned int pictureSize_Y);
void GetData_SD();
unsigned int GetJpegData_toArray( uint8_t array[]);
//
//
uint8_t jpgImgRef[MAXJPGSIZE];//contenuto immagine senza laser acceso
uint8_t jpgImg[MAXJPGSIZE];
#define PIN_LINELASER 23
#define LASER_ON 	digitalWrite(PIN_LINELASER, 1);
#define LASER_OFF 	digitalWrite(PIN_LINELASER, 0);

const int buttonPin = A5;                 // the number of the pushbutton pin


unsigned int picTotalLen = 0;            // picture length
int picNameNum = 0;



#pragma endregion


#pragma region memory monitor

#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

extern char _end;
extern "C" char *sbrk(int i);
char *ramstart = (char *)0x20070000;
char *ramend = (char *)0x20088000;

void freeMemoryReport() {
	char *heapend = sbrk(0);
	register char * stack_ptr asm("sp");
	struct mallinfo mi = mallinfo();
	dbg2("\nDynamic ram used:  ", mi.uordblks);
	dbg2("Program static ram used ", &_end - ramstart);
	dbg2("Stack ram used ", ramend - stack_ptr);
	dbg2("My guess at free mem:  ", stack_ptr - heapend + mi.fordblks);
}

#pragma endregion

// JPEG decoder library
#include <JPEGDecoder.h>

#pragma region LIBRERIE ILI9341_due tft 


//#include <SPI.h>
#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion

#pragma region costanti SERIALCAMERA
#define BUFFPIXEL 20

#define CMD_PREFIX 0xAA
#define CMD_INITIAL 0x01
#define CMD_GETPICTURE 0x04
#define CMD_SNAPSHOT 0x05
#define CMD_SETPKGSIZE 0x06
#define CMD_RESET 0x08
#define CMD_ACK 0x0E
#define CMD_DATA 0x0A
#define CMD_SYNC 0x0D


#define GETSETTING_SNAPSHOT 0x01
#define GETSETTING_RAWPICT 0x02
#define GETSETTING_JPEGPREVIEW 0x03




#define COLORSETTING_BW2BIT 0x01
#define COLORSETTING_BW4BIT 0x02
#define COLORSETTING_BW8BIT 0x03
#define COLORSETTING_COLOR2BIT 0x05
#define COLORSETTING_COLOR16BIT 0x06
#define COLORSETTING_JPEG 0x07

#define PICT_OFFSET_X 0
#define PICT_OFFSET_Y 0


#define RAW_80X60 0x01
#define RAW_160X120 0x03
#define RAW_128X128 0x09
#define RAW_128X96 0x0B

#define JPEGRESOLUTION_80X64 0x01
#define JPEGRESOLUTION_160X128 0x03
#define JPEGRESOLUTION_320X240 0x05
#define JPEGRESOLUTION_640X480 0x07

#define SNAPSHOTSETTING_COMPRESSED 0x00
#define SNAPSHOTSETTING_UNCOMPRESSED 0x01

#define DATATYPE_SNAPSHOT 0x01
#define DATATYPE_RAW 0x02
#define DATATYPE_JPEGPICT 0x05

#define PIC_PKT_LEN    720      //was 128            //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0

#define PIC_FMT        PIC_FMT_VGA

#define CAM_SERIAL     Serial3


#pragma endregion

// converte tftX in alfa e tftY in distanza (cm) e li mette in un array
void convertToDistance(int tftX, int tftY) {
	const int lensAngle = 56;
	int alfa =  (160-tftX )*lensAngle/320;
	//dbg2("alfa", alfa)
		int dist = tftY;
}


#pragma region funzioni DecodeJpeg
//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void renderJPEG(int xpos, int ypos) {

	//jpegInfo(); // Print information from the JPEG file (could comment this line out)

	uint16_t *pImg;
	uint16_t mcu_w = JpegDec.MCUWidth;    // Width of MCU
	uint16_t mcu_h = JpegDec.MCUHeight;   // Height of MCU
	uint32_t mcu_pixels = mcu_w * mcu_h;  // Total number of pixels in an MCU
	dbg2("MCU_pixels:", mcu_pixels)
										  // Serial.print("comp size = ");Serial.println(comp_size);

	uint32_t drawTime = millis(); // For comparison purpose the draw time is measured

								  // Fetch data from the file, decode and display
	while (JpegDec.read()) {    // While there is more data in the file

		pImg = JpegDec.pImage;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

		int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
		int mcu_y = JpegDec.MCUy * mcu_h + ypos;
		//dbg2("mcu_x", mcu_x)
		//dbg2("mcu_y", mcu_y)

		if ((mcu_x + mcu_w) <= tft.width() && (mcu_y + mcu_h) <= tft.height())
		{
			// Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
			//tft.setWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);
			tft.setAddrWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);

			// Push all MCU pixels to the TFT window
			uint32_t count = mcu_pixels;
			while (count--) {
				// Push each pixel to the TFT MCU area
				tft.pushColor(*pImg++);
			}

			// Push all MCU pixels to the TFT window, ~18% faster to pass an array pointer and length to the library
			// tft.pushColor16(pImg, mcu_pixels); //  To be supported in HX8357 library at a future date

		}
		else if ((mcu_y + mcu_h) >= tft.height()) JpegDec.abort(); // Image has run off bottom of screen so abort decoding
	}

	//	showTime(millis() - drawTime); // These lines are for sketch testing only
	//Serial.print(" Draw count:");
	//Serial.println(icount++);
}
//####################################################################################################
// Subtract a JPEG from the TFT, 
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void subtractJPEG(int xpos, int ypos) {
	dbg("\\nfaccio la differenza tra le immagini")
	//jpegInfo(); // Print information from the JPEG file (could comment this line out)
	int tftX;
	int tftY;
	uint16_t px;
	uint16_t px1;
	uint16_t red;
	uint16_t green;
	uint16_t blu;
	uint16_t red1;
	uint16_t green1;
	uint16_t blu1;
	uint16_t r;
	uint16_t g;
	uint16_t b;

	uint16_t *pImg;
	uint16_t mcu_w = JpegDec.MCUWidth;    // Width of MCU
	uint16_t mcu_h = JpegDec.MCUHeight;   // Height of MCU
	uint32_t mcu_pixels = mcu_w * mcu_h;  // Total number of pixels in an MCU
	//dbg2("MCU_pixels:", mcu_pixels)
		// Serial.print("comp size = ");Serial.println(comp_size);

		uint32_t drawTime = millis(); // For comparison purpose the draw time is measured

									  // Fetch data from the file, decode and display
	while (JpegDec.read()) {    // While there is more data in the file

		pImg = JpegDec.pImage;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

		int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
		int mcu_y = JpegDec.MCUy * mcu_h + ypos;

		//dbg2("mcu_x", mcu_x)
		//dbg2("mcu_y", mcu_y)

		if ((mcu_x + mcu_w) <= tft.width() && (mcu_y + mcu_h) <= tft.height())
		{
			// Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
			//tft.setWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);
			tft.setAddrWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);

				tftX = mcu_x;
				tftY = mcu_y;



			// Push all MCU pixels to the TFT window
			uint32_t count = mcu_pixels;
			while (count--) {

				//mia modifica  �����������������������������������������������
				// per essere allineati al contenuto di *pImg 
				// devo percorrere l'area del TFT con:
				// tftX che deve andare da mcu_x a mcu_x+ mcu_w
				// tftY che deve andare da mcu_y a mcu_y+ mcu_h

				//valore corrente dell'immagine su LCD con laser
				px = tft.readPixel(tftX, tftY);
				//estraggo le componenti r g b:  rrrrgg ggbbbb
				red = px >> 10;
				green = ( px >> 5) & 0x3F;
				blu = px   & 0x3F;
				//if (red>60)
				//{
				//	dbgHex(red)
				//	dbg2("@x",tftX)
				//	dbg2("@y", tftY)
				//}

				//valore immagine senza laser
				px1 = *pImg;
				//estraggo le componenti r g b:  rrrrgg ggbbbb
				red1 = px1 >> 10;
				green1 = ( px1 >> 5) & 0x3F;
				blu1 = px1   & 0x3F;


#if 0
				// faccio la semplice differenza
				r = red - red1;
				g = green - green1;
				b = blu - blu1;
				//sopra soglia esalto il rosso
				if (r>30)
				{
					r = 0xFFFF;

				}
#else
				//differenza con soglia 

				r = max(red - red1, 0);
				g = max(green - green1, 0);
				b = max(blu - blu1, 0);

				if (r>30)		//soglie ok : 20
				{
					dbgHex(r)
					r = 0xFFFF;
					g = 0;
					b = 0;
					convertToDistance(tftX, tftY);
				}
				else {
					r = 0;
					g = 0;
					b = 0;
				}

#endif // 1

				//dbgHex(r)
				//dbgHex(g)
				//dbgHex(b)

				// Push each pixel to the TFT MCU area

				//tft.pushColor( px-px1);
				//*pImg=tft.color565(r, g, b);
				//tft.pushColor(*pImg++);		// original
				tft.drawPixel(tftX, tftY, tft.color565(r, g, b));
				*pImg++;

				tftX++;
				if (tftX>= mcu_x + mcu_w)				{
					tftX = mcu_x;
					tftY++;
				}

				//�����������������������������������������������������������




			}


		}
		else if ((mcu_y + mcu_h) >= tft.height()) JpegDec.abort(); // Image has run off bottom of screen so abort decoding
	}

	//	showTime(millis() - drawTime); // These lines are for sketch testing only
	//Serial.print(" Draw count:");
	//Serial.println(icount++);
}

//####################################################################################################
// Draw a JPEG on the TFT pulled from a program memory array
//####################################################################################################

void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos) {

	int x = xpos;
	int y = ypos;

	JpegDec.decodeArray(arrayname, array_size);
	renderJPEG(x, y);
}

//####################################################################################################
// Print image information to the serial port (optional)
//####################################################################################################
// JpegDec.decodeFile(...) or JpegDec.decodeArray(...) must be called before this info is available!
void jpegInfo() {

	// Print information extracted from the JPEG file
	Serial.println("JPEG image info");
	Serial.println("===============");
	Serial.print("Width      :");
	Serial.println(JpegDec.width);
	Serial.print("Height     :");
	Serial.println(JpegDec.height);
	Serial.print("Components :");
	Serial.println(JpegDec.comps);
	Serial.print("MCU / row  :");
	Serial.println(JpegDec.MCUSPerRow);
	Serial.print("MCU / col  :");
	Serial.println(JpegDec.MCUSPerCol);
	Serial.print("Scan type  :");
	Serial.println(JpegDec.scanType);
	Serial.print("MCU width  :");
	Serial.println(JpegDec.MCUWidth);
	Serial.print("MCU height :");
	Serial.println(JpegDec.MCUHeight);
	Serial.println("===============");
	Serial.println("");
}

//####################################################################################################
// Draw a scaled JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.

// Images are scaled 1:N so if scale = 2 the image is drawn half size, 4 quarter size, 8 eighth size



#pragma endregion

#pragma region funzioni della SerialCamera

/*********************************************************************/
void clearRxBuf()
{
	while (CAM_SERIAL.available())
	{
		CAM_SERIAL.read();
	}
}
/*********************************************************************/
void sendCmd(char cmd[6], int cmd_len)
{
	Serial.print("[");
	for (char i = 0; i < cmd_len; i++) {
		CAM_SERIAL.print(cmd[i]);
		Serial.print(cmd[i], HEX); Serial.print(" ");
	}
	Serial.print("]");

}
/*********************************************************************/
/*********************************************************************/
void sendCmdWaitAck(byte c0, byte c1, byte c2, byte c3, byte c4, byte c5)
{
	char cmd[6];
	cmd[0] = c0;
	cmd[1] = c1;
	cmd[2] = c2;//snapshot type 0=jpeg, 1=raw
	cmd[3] = c3;//skip frame Low
	cmd[4] = c4;//skpframe  high
	cmd[5] = c5;

	unsigned char resp[6];


	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == (0x0e) && resp[2] == cmd[1] && resp[4] == 0 && resp[5] == 0)
		{
			break;
		}
	}

}

void cameraSYNC()
{
	dbg("\r\nCamera SYNC ...");
	delay(800); // attesa necessaria  dall'accensione prima di inviare un sync
	int retries = 0;
	char cmd[] = { CMD_PREFIX,CMD_SYNC  ,0x00,0x00,0x00,0x00 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(500);
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		//while (!CAM_SERIAL.available() == 6) { delay(10); }
		while (CAM_SERIAL.available() <= 6) { delayMicroseconds(10); }
		Serial.print("<");
		for (size_t i = 0; i < 6; i++) {
			resp[i] = CAM_SERIAL.read();
			Serial.print(resp[i], HEX); Serial.print(" ");
		}
		Serial.print(">");
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_SYNC && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
		{
			retries++;
			dbg2("retr:", retries);
			//dbg2("cmd:", cmd);

			//	for (size_t j = 0; j < strlen((char *)resp); j++)	{dbg(resp[j])}

			continue;
		}
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
		{
			if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
			if (resp[0] == CMD_PREFIX && resp[1] == (CMD_SYNC) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
		}
	}
	cmd[1] = CMD_ACK;
	cmd[2] = CMD_SYNC;
	sendCmd(cmd, 6);
	dbg("\r\nCamera SYNC done.");
}
/*********************************************************************/
bool waitAck(byte cmdId) {
	unsigned char resp[6];
	while (1) {
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == cmdId && resp[4] == 0 && resp[5] == 0)
		{
			dbg2("<ACK:", cmdId)
				return true;
		}
		else
		{
			return false;
		}

	}

}
//invia comando SYNC
void initialize()
{
	char cmd[] = { 0xaa,0x0d ,0x00,0x00,0x00,0x00 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(500);
	while (1)
	{
		//clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
		{
			continue;
		}
		if (resp[0] == 0xaa && resp[1] == (0x0e) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
		{
			if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
			if (resp[0] == 0xaa && resp[1] == (0x0d ) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
		}
	}
	cmd[1] = 0x0e ;
	cmd[2] = 0x0d;
	sendCmd(cmd, 6);
	Serial.println("\nCamera initialization done.");
}
/*********************************************************************/
//invia comando  INITIAL
void initial(byte c0, byte c1, byte c2, byte imagFormat, byte rawSetting, byte jpegResolution)
{
	dbg("\r\nInitial...")
		char cmd[6];
	cmd[0] = CMD_PREFIX;
	cmd[1] = CMD_INITIAL;
	cmd[2] = 0x00;
	cmd[3] = imagFormat; //img format
	cmd[4] = rawSetting; //RAW setting
	cmd[5] = jpegResolution; 

	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (waitAck(cmd[1])) break;
	}
}

void preCapture_orig()
{
	//char cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT }; JPEG VGA
	char cmd[] = { 0xaa, CMD_INITIAL  , 0x00, 0x07, 0x00, 0x05 };	//JPEG 320X240
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] ==  0x0e  && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) break;
	}
}

/*********************************************************************/

void sendAck(byte cmdId, unsigned int pktCnt = 0)
{
	//dbg("\r\nInitial...")
	char cmd[6];
	cmd[0] = CMD_PREFIX;
	cmd[1] = CMD_ACK;
	cmd[2] = cmdId;
	cmd[3] = 0;
	cmd[4] = pktCnt & 0xff;;
	cmd[5] = pktCnt >> 8;

	sendCmd(cmd, 6);

}

/*********************************************************************/

// solo per jpeg
void setPktSize(unsigned int packageSize)
{
	dbg("\r\nSetting Package Size...")
		char cmd[] = { 0xaa, 0x06 , 0x08, packageSize & 0xff, (packageSize >> 8) & 0xff ,0 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (waitAck(cmd[1])) { break; }

		//if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		//if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
	}


}


/*********************************************************************/
void snapshot(byte c0, byte c1, byte snapshotType, byte c3, byte c4, byte c5)
{
	char cmd[6];
	cmd[0] = 0xaa;
	cmd[1] = CMD_SNAPSHOT;
	cmd[2] = snapshotType;//snapshot type 0=jpeg, 1=raw
	cmd[3] = c3;//skip frame Low
	cmd[4] = c4;//skpframe  high
	cmd[5] = 0;

	unsigned char resp[6];


	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == (0x0e) && resp[2] == cmd[1] && resp[4] == 0 && resp[5] == 0)
		{
			dbg("<ACK snapshot>")
				break;
		}
	}

}
//void Capture(byte c0, byte c1, byte c2, byte c3, byte c4, byte c5 )
//{
//	#pragma region setPackage size
//	char cmd[] = { 0xaa, 0x06 , 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN >> 8) & 0xff ,0 };
//	unsigned char resp[6];
//
//	CAM_SERIAL.setTimeout(100);
//	while (1)
//	{
//		clearRxBuf();
//		sendCmd(cmd, 6); //set package size 
//		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
//		if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0)
//			break;
//	}
//	#pragma endregion
//
//
//
//#pragma region Snapshot 1=raw/0=jpeg
//	cmd[1] = 0x05 | cameraAddr;
//	cmd[2] = c2;
//	cmd[3] = c3; //skip
//	cmd[4] = c4;
//	cmd[5] = 0;
//	while (1)
//	{
//		clearRxBuf();
//		sendCmd(cmd, 6); // snapshot
//		// wait ack
//		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
//		if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
//	}
//#pragma endregion
//
//
//
//
//
//    cmd[1] = 0x04 | cameraAddr;
//    cmd[2] = 0x1;
//    while (1)
//    {
//        clearRxBuf();
//        sendCmd(cmd, 6);
//        if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
//        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
//        {
//			CAM_SERIAL.setTimeout(1000);
//            if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
//            {
//                continue;
//            }
//            if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
//            {
//                picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
//                Serial.print("\npicTotalLen:");
//                Serial.println(picTotalLen);
//                break;
//            }
//        }
//    }
//
//}
/*********************************************************************/
void Capture_orig() //Versione originale
{
	char cmd[] = { 0xaa, CMD_SETPKGSIZE, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN >> 8) & 0xff ,0 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == CMD_SETPKGSIZE && resp[4] == 0 && resp[5] == 0) break;
	}
	cmd[1] = CMD_SNAPSHOT ;
	cmd[2] = 0;
	cmd[3] = 0;
	cmd[4] = 0;
	cmd[5] = 0;
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] ==0x0e  && resp[2] == CMD_SNAPSHOT && resp[4] == 0 && resp[5] == 0) break;
	}
	cmd[1] = CMD_GETPICTURE ;
	cmd[2] = GETSETTING_SNAPSHOT;

	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0)
		{
			CAM_SERIAL.setTimeout(1000);
			if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
			{
				continue;
			}
			if (resp[0] == 0xaa && resp[1] == (0x0a ) && resp[2] == 0x01)
			{
				picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
				Serial.print("\npicTotalLen:");
				Serial.println(picTotalLen);
				break;
			}
		}
	}

}
/*********************************************************************/
// strada abbandonata, non funziona
void GetPicture_toLCD(byte c0, byte c1, byte c2, byte c3, byte c4, byte c5, unsigned int pictureSize_X, unsigned int pictureSize_Y)
{
	//************************************************************
	// setup per la visualizzazion su LCD
	//************************************************************
	int row = 1;
	int	col = 1;	// riga e colonna corrente del TFT
	uint8_t  r, g, b;

	uint16_t color = 0; //colore
	uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)

	//************************************************************


#pragma region invio comando GetPicture con ricezione dimensione immagine

	char cmd[] = { 0xaa, 0x04  , c2, 0x00, 0x00, 0x00 };
	unsigned char resp[6];

	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);//invia comando GETPICTURE 

		//e attende la dimensione dell'immagine
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
		{
			CAM_SERIAL.setTimeout(1000);
			if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
			{
				continue;
			}
			if (resp[0] == 0xaa && resp[1] == (0x0a ) && resp[2] == 0x01)
			{
				picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
				dbg2("\npicTotalLen:", picTotalLen)

					break;
			}
		}
	}

#pragma endregion

	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;
	dbg2("\nPacket count:", pktCnt);
	int currentPkt = 0;
	sendAck(0, 0);// ok fa partire l'invio dei dati

	// a questo punto dovrebbe inviare i dati di tutto il frame
//	while (CAM_SERIAL.available()<100) { delay(1); }

#pragma region Visualizzazione dei dati su TFT
//************************************************************
// qui devo mandare l'immagine sul display
//************************************************************
	unsigned int bytecount = 0; //conta i byte ricevuti
	byte pixel_R;
	byte pixel_G;
	byte pixel_B;
	for (size_t x = PICT_OFFSET_X; x < PICT_OFFSET_X + pictureSize_X; x++)
	{
		dbg2("\n[x:", x)
			//freeMemoryReport();
			dbg2("\nbytecount: ", bytecount)

			for (size_t y = PICT_OFFSET_Y; y < PICT_OFFSET_Y + pictureSize_Y; y++)
			{


				while (CAM_SERIAL.available() < 3) {
					delayMicroseconds(10);
					if (bytecount > PIC_PKT_LEN - 10)
					{
						bytecount = 0;
						dbg2("\n--End Packet:", currentPkt)
							currentPkt++;
						sendAck(0, currentPkt);// ok fa partire l'invio dei dati del pacchetto successivo

					}
					else
					{
						dbg2("\nbytecount", bytecount)
							delay(20);
					}
				}
				pixel_R = CAM_SERIAL.read();
				pixel_G = CAM_SERIAL.read();
				pixel_B = CAM_SERIAL.read();
				dbgHex(pixel_R)
					dbgHex(pixel_G)
					dbgHex(pixel_B)


					bytecount += 3;

				// riduco di 4 volte il valore per adattarlo ai colori visualizzabili da LCD
				r = pixel_R; //>> 2;
				g = pixel_G; //>> 2;
				b = pixel_G; // >> 2;

				color = tft.color565(r, g, b);

				tft.drawPixel(x, y, color);  //zero based, auto clip

				if (bytecount >= picTotalLen)
				{
					dbg("Fine immagine")
#pragma region send ACK fine frame

						// send ACK fine frame
						cmd[1] = 0xae;
					cmd[2] = 0x0a;
					cmd[3] = 0x00;
					cmd[4] = 0x01;
					cmd[5] = 0x00;
					sendCmd(cmd, 6);

#pragma endregion


				}
			}
	}
	//************************************************************  
#pragma endregion
	tft.printAligned(F("FINE"), gTextAlignMiddleCenter);

}
/*********************************************************************/
void GetData_SD()  //versione che funziona con SD abilitata
{
	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;
	dbg2("\nPacket count:", pktCnt);

	char cmd[] = { 0xaa, 0x0e , 0x00, 0x00, 0x00, 0x00 };
	unsigned char pkt[PIC_PKT_LEN];

#if OPT_SDCARD

	char picName[] = "pic00.jpg";
	picName[3] = picNameNum / 10 + '0';
	picName[4] = picNameNum % 10 + '0';

	if (SD.exists(picName))
	{
		SD.remove(picName);
	}

	myFile = SD.open(picName, FILE_WRITE);
	if (!myFile) {
		Serial.println("myFile open fail...");
}
#else
	if (false) {
	}

#endif // OPT_SDCARD

	else {
		CAM_SERIAL.setTimeout(1000);
		for (unsigned int i = 0; i < pktCnt; i++)
		{
			cmd[4] = i & 0xff;
			cmd[5] = (i >> 8) & 0xff;

			int retry_cnt = 0;
		retry:
			delay(10);
			clearRxBuf();
			sendCmd(cmd, 6);
			uint16_t cnt = CAM_SERIAL.readBytes((char *)pkt, PIC_PKT_LEN);

			unsigned char sum = 0;
			for (int y = 0; y < cnt - 2; y++)
			{
				sum += pkt[y];
				dbgHex(pkt[y])
			}
			if (sum != pkt[cnt - 2])
			{
				if (++retry_cnt < 100) goto retry;
				else break;
			}
#if OPT_SDCARD
			myFile.write((const uint8_t *)&pkt[4], cnt - 6);
			//if (cnt != PIC_PKT_LEN) break;
#endif // OPT_SDCARD
			}
		cmd[4] = 0xf0;
		cmd[5] = 0xf0;
		sendCmd(cmd, 6);
	}
#if OPT_SDCARD
	myFile.close();
#endif // OPT_SDCARD

	picNameNum++;
		}
/*********************************************************************/
unsigned int GetJpegData_toArray( uint8_t array[])  //versione che usa jpegDecoder
{
	int jpgImgEnd = 0; // posizione dove scrivere il prossimo pacchetto nell'array


	// calcolo il numero dei pacchetti
	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;
	dbg2("\nPacket count:", pktCnt);

	char cmd[] = { 0xaa, 0x0e , 0x00, 0x00, 0x00, 0x00 };

	//array con i dati del pacchetto
	uint8_t pkt[PIC_PKT_LEN];

 
	CAM_SERIAL.setTimeout(1000);

	// per ogni pacchetto
	for (unsigned int i = 0; i < pktCnt; i++)
	{
		cmd[4] = i & 0xff;
		cmd[5] = (i >> 8) & 0xff;

		int retry_cnt = 0;
	retry:
		delay(10);
		clearRxBuf();
		// invio l'ACK con il numero del pacchetto da inviare
		sendCmd(cmd, 6);

		//leggo i dati nell'array pkt
		uint16_t cnt = CAM_SERIAL.readBytes((char *)pkt, PIC_PKT_LEN);

		//calcolo checksum e ricopio nell'array jpgImg
		unsigned char sum = 0;
		for (int y = 0; y < cnt - 2; y++)
		{
			sum += pkt[y];

		}
		if (sum != pkt[cnt - 2])
		{
			if (++retry_cnt < 100) goto retry;
			else break;
		}

		// transfer into array
		for (int y = 0; y < cnt - 2; y++)
		{
			array[jpgImgEnd]=  pkt[y];
			jpgImgEnd++;
		}


	}
	cmd[4] = 0xf0;
	cmd[5] = 0xf0;
	sendCmd(cmd, 6);
 

	jpgImgEnd = 0;
	dbg("\n-end img-")
	return picTotalLen;

}
/*********************************************************************/

#pragma endregion

unsigned int jpgSize = 0;
uint16_t px;
byte red = 0;
byte green = 0;
byte blu = 0;
byte soglia = 60;

void setup()
{
	Serial.begin(115200);

#pragma region ILI9341_due tft  setup
	//Serial.begin(9600);

	bool result = tft.begin();

	tft.setRotation(iliRotation90);
	tft.fillScreen(ILI9341_BLUE);

	tft.setFont(Arial_bold_14);
	tft.setTextLetterSpacing(5);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
	tft.printAligned(F("Serial camera JPG display"), gTextAlignMiddleCenter);

	//for (size_t x = PICT_OFFSET_X; x < PICT_OFFSET_X + PICT_SIZE_X; x++)
	//{
	//	for (size_t y = PICT_OFFSET_Y; y < PICT_OFFSET_Y + PICT_SIZE_Y; y++)
	//	{
	//		tft.drawPixel(x, y, ILI9341_RED);

	//	}
	//}


#pragma endregion

	
#pragma region SERIAL_CAMERA SETUP

	CAM_SERIAL.begin(115200);
	CAM_SERIAL.setTimeout(5000);


//	cameraSYNC();
	initialize();
#pragma endregion

	//initial(0xaa, 0x01, 0x00,0x03 , 0x01, 0x00);//raw non ricevo ack
 	//initial(0xaa, 0x01, 0x00, COLORSETTING_JPEG, 0x00, JPEGRESOLUTION_320X240); //cos� mi da l'ack
	//initial(0xaa, 0x01, 0x00, COLORSETTING_COLOR16BIT, 7, 7);//  ACK

	// deve essere coerente con l'impostazione RAW___X__ utilizzata
#define PICT_SIZE_X 320 
#define PICT_SIZE_Y 240 

 	//initial(0xaa, 0x01, 0x00, COLORSETTING_JPEG, 0, JPEGRESOLUTION_320X240); //cos� mi da l'ack
	setPktSize(PIC_PKT_LEN);//240x3
//	snapshot(0xaa, 0x05 , SNAPSHOTSETTING_UNCOMPRESSED, 0 ,0, 0);
 
#define PICTURETYPE_SNAPSHOT 1
#define PICTURETYPE_RAW 2
#define PICTURETYPE_JPEG 5

	//GetPicture_toLCD(0xaa, 0x04, PICTURETYPE_SNAPSHOT, 0x00, 0x01, 0x00, PICT_SIZE_X, PICT_SIZE_Y);



 	initial(0xaa, 0x01, 0x00, COLORSETTING_JPEG, COLORSETTING_JPEG, JPEGRESOLUTION_320X240); //cos� mi da l'ack
	pinMode(PIN_LINELASER, OUTPUT);





}
/*********************************************************************/
void loop()
{
 
 	// catturo l'immagine con il laser acceso
	LASER_ON
	delay(10);
	Capture_orig();
	jpgSize= GetJpegData_toArray(jpgImg);
	JpegDec.decodeArray(jpgImg, jpgSize);
	dbg("\nRendering to LCD..")
	renderJPEG(0, 0);
	jpegInfo();


	LASER_OFF
	delay(10);
	Capture_orig();
	jpgSize= GetJpegData_toArray(jpgImg);
	JpegDec.decodeArray(jpgImg, jpgSize);
	// visualizzo la differenza
	subtractJPEG(0, 0);

	//freeMemoryReport();





}
/*********************************************************************/




