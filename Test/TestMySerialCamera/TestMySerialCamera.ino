//  File SerialCamera.pde for camera.
//  25/7/2011 by Piggy
//  Modify by Deray  08/08/2012 
//  Demo code for using seeeduino or Arduino board to cature jpg format 
//  picture from seeed serial camera and save it into sd card. Push the 
//  button to take the a picture .
 
//  For more details about the product please check http://www.seeedstudio.com/depot/
//#############################################################################à
#define dbg(cha) 	 Serial.println(cha);
#define dbg2(t,cha)	 Serial.print(t);Serial.println(cha);  

#include <SD.h>
#define SD_CS 10     // Set the chip select line to whatever you use (10 doesnt conflict with the library)

File myFile;
#if 0

#pragma region LIBs , Maacro e funzioni TFT
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
//#include <SPFD5408_TouchScreen.h>     // Touch library

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#define SD_CS 10     // Set the chip select line to whatever you use (10 doesnt conflict with the library)

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);



// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF




// This function opens a Windows Bitmap (BMP) file and

#define BUFFPIXEL 20


uint16_t read16(File f) {
	uint16_t result;
	((uint8_t *)&result)[0] = f.read(); // LSB
	((uint8_t *)&result)[1] = f.read(); // MSB
	return result;
}

uint32_t read32(File f) {
	uint32_t result;
	((uint8_t *)&result)[0] = f.read(); // LSB
	((uint8_t *)&result)[1] = f.read();
	((uint8_t *)&result)[2] = f.read();
	((uint8_t *)&result)[3] = f.read(); // MSB
	return result;
}

// Copy string from flash to serial port
// Source string MUST be inside a PSTR() declaration!
void progmemPrint(const char *str) {
	char c;
	while (c = pgm_read_byte(str++)) Serial.print(c);
}

// Same as above, with trailing newline
void progmemPrintln(const char *str) {
	progmemPrint(str);
	Serial.println();
}

void drawBorder() {

	// Draw a border

	uint16_t width = tft.width() - 1;
	uint16_t height = tft.height() - 1;
	uint8_t border = 10;

	tft.fillScreen(RED);
	tft.fillRect(border, border, (width - border * 2), (height - border * 2), WHITE);

}
#pragma endregion


#endif // 0

#pragma region TFT 

#include <SPI.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>

#include "fonts\Arial_bold_14.h"

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);

#pragma endregion


#define outputSerial Serial	// messaggi 

//#############################################################################à
#pragma region SerialCamera
#if 1
#define cameraSerial Serial
#else
	#include <SoftwareSerial/SoftwareSerial.h>

	SoftwareSerial cameraSerial(2, 3); // RX, TX//  //

#endif // 0

//  File SerialCamera_DemoCode_CJ-OV528.ino
//  8/8/2013 Jack Shao
//  Demo code for using seeeduino or Arduino board to cature jpg format
//  picture from seeed serial camera and save it into sd card. Push the
//  button to take the a picture .
//  For more details about the product please check http://www.seeedstudio.com/depot/

#include <SPI.h>
#include <arduino.h>
// per i comandi della Serial Camera vedi 	"CJ - CAM User Manual OV528 Protocol"
#pragma region costanti SERIALCAMERA

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
#define GETSETTING_PREVIEWPICT 0x02
#define GETSETTING_JPEGPREVIEW 0x03




#define COLORSETTING_BW2BIT 0x01
#define COLORSETTING_BW4BIT 0x02
#define COLORSETTING_BW8BIT 0x03
#define COLORSETTING_COLOR2BIT 0x05
#define COLORSETTING_COLOR16BIT 0x06
#define COLORSETTING_JPEG 0x07

#define PREVIEWRESOLUTION_80X60 0x01
#define PREVIEWRESOLUTION_160X120 0x03

#define JPEGRESOLUTION_80X64 0x01
#define JPEGRESOLUTION_160X128 0x03
#define JPEGRESOLUTION_320X240 0x05
#define JPEGRESOLUTION_640X480 0x07

#define SNAPSHOTSETTING_COMPRESSED 0x00
#define SNAPSHOTSETTING_UNCOMPRESSED 0x01

#define DATATYPE_SNAPSHOT 0x01
#define DATATYPE_PREVIEW 0x02
#define DATATYPE_JPEGPICT 0x05

#define PIC_PKT_LEN    240      //was 128            //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0
#define CAM_SERIAL     cameraSerial

#define PIC_FMT        PIC_FMT_VGA

#pragma endregion


const byte cameraAddr = (CAM_ADDR << 5);  // addr
const int buttonPin = A5;                 // the number of the pushbutton pin
unsigned long picTotalLen = 0;            // picture length
int picNameNum = 0;

/*********************************************************************/
void clearRxBuf()
{
	while (cameraSerial.available())
	{
		cameraSerial.read();
	}
}
/*********************************************************************/
void sendCmd(char cmd[], int cmd_len)
{
	for (char i = 0; i < cmd_len; i++) cameraSerial.print(cmd[i]);
}
/*********************************************************************/
void cameraSYNC()
{
	int retries = 0;
	char cmd[] = { CMD_PREFIX,CMD_SYNC  ,0x00,0x00,0x00,0x00 };
	unsigned char resp[6];

	cameraSerial.setTimeout(500);
	while (1)
	{
		//clearRxBuf();
		sendCmd(cmd, 6);
		while (!cameraSerial.available()==6){delay(10);	}
		for (size_t i = 0; i < 6; i++){
			resp[i] = cameraSerial.read(); 
			Serial.print(resp[i], HEX);
		}
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_SYNC && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;

		//if (cameraSerial.readBytes((char *)resp, 6) != 6)
		{
			retries++;
			dbg2("retr:",retries);
			dbg2("cmd:",cmd);

			for (size_t j = 0; j < strlen((char *)resp); j++)
			{
				dbg(resp[j])
			}

			continue;
		}
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK  ) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
		{
			if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
			if (resp[0] == CMD_PREFIX && resp[1] == (CMD_SYNC  ) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
		}
	}
	cmd[1] = CMD_ACK  ;
	cmd[2] = CMD_SYNC;
	sendCmd(cmd, 6);
	dbg(" Camera SYNC done.");
}
/*********************************************************************/
void cameraInitial(uint8_t colorsetting = COLORSETTING_JPEG, uint8_t previewResolution = 0x00,  uint8_t resolution= JPEGRESOLUTION_640X480)
{
	char cmd[] = { CMD_PREFIX, CMD_INITIAL  , 0x00, colorsetting,previewResolution , resolution };
	unsigned char resp[6];

	cameraSerial.setTimeout(100);
	while (1) //ripeti finchè non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK  ) && resp[2] == CMD_INITIAL && resp[4] == 0 && resp[5] == 0) break;
	}
}
void preCaptureQVGAUncompressed()
{
	char cmd[] = { CMD_PREFIX, CMD_INITIAL  , 0x00, COLORSETTING_JPEG, PREVIEWRESOLUTION_160X120, JPEGRESOLUTION_320X240 };
	unsigned char resp[6];

	cameraSerial.setTimeout(100);
	while (1) //ripeti finchè non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK  ) && resp[2] == CMD_INITIAL && resp[4] == 0 && resp[5] == 0) break;
	}
}

void cameraSETPACKAGESIZE(uint16_t pkgSize= PIC_PKT_LEN) {
	char cmd[] = { CMD_PREFIX, CMD_SETPKGSIZE , 0x08, pkgSize & 0xff, (pkgSize >> 8) & 0xff ,0 };
	unsigned char resp[6];
	cameraSerial.setTimeout(100);
	while (1) // invia il comando finchè non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (Serial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK  ) && resp[2] == CMD_SETPKGSIZE && resp[4] == 0 && resp[5] == 0) break;
	}

}

void cameraSNAPSHOT (uint8_t compression = SNAPSHOTSETTING_COMPRESSED, uint16_t skipframe =0)
{
	unsigned char resp[6];

	// cameraSETPACKAGESIZE
	cameraSETPACKAGESIZE(PIC_PKT_LEN);
	char cmd[6]= { CMD_PREFIX, CMD_SNAPSHOT , compression ,skipframe & 0xff, (skipframe >> 8) & 0xff,0 };


	while (1) // invia il comando finchè non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (Serial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK  ) && resp[2] == CMD_SNAPSHOT && resp[4] == 0 && resp[5] == 0) break;
	}

}
/*********************************************************************/

void cameraGETPICTURE(uint8_t getsettings = GETSETTING_SNAPSHOT)
{
	char cmd[] = { CMD_PREFIX, CMD_GETPICTURE ,getsettings,  0x00,  0x00 ,0x00 };
	unsigned char resp[6];

	cameraSerial.setTimeout(100);
	outputSerial.print("CMD_GETPICTURE...:");
	while (1) // invia il comando finchè non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (Serial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == CMD_ACK  && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0) break;
	}
	outputSerial.print("ok");


	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) {
	outputSerial.print((char *)resp);

			continue;
		}
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK  ) && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0)
		{
			cameraSerial.setTimeout(1000);
			if (cameraSerial.readBytes((char *)resp, 6) != 6)
			{
				continue;
			}
			if (resp[0] == CMD_PREFIX && resp[1] == (CMD_DATA  ) && resp[2] == DATATYPE_SNAPSHOT)
			{
				picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
				outputSerial.print("picTotalLen:");
				outputSerial.println(picTotalLen);
				break;
			}
		}
	}

}
/*********************************************************************/


void GetData_toSD() //Salva su SD l'immagine
{

	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

	char cmd[] = { 0xaa, 0x0e  , 0x00, 0x00, 0x00, 0x00 };
	unsigned char pkt[PIC_PKT_LEN];

	char picName[] = "pic00.jpg";
	picName[3] = picNameNum / 10 + '0';
	picName[4] = picNameNum % 10 + '0';

	if (SD.exists(picName))
	{
		SD.remove(picName);
	}

	myFile = SD.open(picName, FILE_WRITE);
	if (!myFile) {
		outputSerial.println("myFile open fail...");
	}
	else {
		cameraSerial.setTimeout(1000);
		for (unsigned int i = 0; i < pktCnt; i++)
		{
			cmd[4] = i & 0xff;
			cmd[5] = (i >> 8) & 0xff;

			int retry_cnt = 0;
		retry:
			delay(10);
			clearRxBuf();
			sendCmd(cmd, 6);
			uint16_t cnt = cameraSerial.readBytes((char *)pkt, PIC_PKT_LEN);

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

			myFile.write((const uint8_t *)&pkt[4], cnt - 6);
			//if (cnt != PIC_PKT_LEN) break;



		}
		cmd[4] = 0xf0;
		cmd[5] = 0xf0;
		sendCmd(cmd, 6);
	}
	myFile.close();
	picNameNum++;
}

/*********************************************************************/
void GetData_toLCD()
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

	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);//numero di pacchetti di dati
	// più uno per il resto
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

	dbg2("PKT Count: ",pktCnt)


	char cmd[] = { CMD_PREFIX, CMD_ACK  , 0x00, 0x00, 0x00, 0x00 };
	unsigned char pkt[PIC_PKT_LEN];


	cameraSerial.setTimeout(1000);

	//per ogni pacchetto dati
	for (unsigned int i = 0; i < pktCnt; i++)
	{
		cmd[4] = i & 0xff;	// Package ID Byte 0
		cmd[5] = (i >> 8) & 0xff; //Package ID Byte 1

		int retry_cnt = 0;
	retry:
		delay(10);	//was 10
		clearRxBuf();
		//--------------------------------------------------------
		sendCmd(cmd, 6);// chiedi l'invio dei dati del pacchetto i
		//----------------------------------------------------------
		uint16_t cnt = cameraSerial.readBytes((char *)pkt, PIC_PKT_LEN);
		//dbg2("cnt: ", cnt)

		unsigned char sum = 0;
		for (int y = 0; y < cnt - 2; y++)
		{
			sum += pkt[y];
		}
		if (sum != pkt[cnt - 2])
		{		
			dbg2("retry_cnt: ", retry_cnt)

			if (++retry_cnt < 100) goto retry;
			else break;
		}

		//Serial.write((const uint8_t *)&pkt[4], cnt - 6);
		//if (cnt != PIC_PKT_LEN) break;

		//Serial.print((const uint8_t *)&pkt[4], cnt - 6);

#pragma region Visualizzazione dei dati su TFT
		//************************************************************
		// qui devo mandare l'immagine sul display
		//************************************************************

		// per ogni elemento del pacchetto
		for (size_t k = 0; k < cnt - 2; k+=3)	// -5 deriva da -2 -3 che sono i 3 valori di colore ?
		{	
			r = pkt[k];
			g = 0; //pkt[k + 5];
			b = 0;	//pkt[k + 6];
			//pkt[4 + i];
			color = tft.color565(r,g,b);
			//tft.drawPixel(row, col, pkt[i]); // col = y lato lungo

			tft.drawPixel(row, col, color); // col = y lato lungo
			row++;
			if (row > 240) { //resetto la colonna e incremento la riga
				row = 1; 
				col++;}// dbg2("row:",row)
			 if(col >= 320) { row = 1; }
		}

		//lcdbuffer[lcdidx++] = tft.color565(r, g, b);
		// Convert pixel from BMP to TFT format
		//b = pkt[4];
		//g = pkt[5];
		//r = pkt[6];

		//************************************************************  
#pragma endregion


	}
	cmd[4] = 0xf0;
	cmd[5] = 0xf0;
	sendCmd(cmd, 6);
	//}
	//myFile.close();
	//picNameNum++;
}

#pragma endregion
/*********************************************************************/
void setup()
{
	outputSerial.begin(115200);


#pragma region TFT setup
	//Serial.begin(9600);
	digitalWrite(35, HIGH);         //I use this on mega for LCD Backlight

	tft.reset();
	// SDFP5408
	uint16_t identifier = 0x9341;		// tft.readID();
	tft.begin(identifier);
	tft.setRotation(0); // Need for the Mega, please changed for your choice or rotation initial

	drawBorder();	// Border

					// cameraInitial screen
	tft.fillScreen(BLACK);
	//tft.setRotation(1);

	tft.setCursor(55, 50);
	tft.setTextSize(3);
	tft.setTextColor(RED);
	tft.println("SPFD5408");
	tft.setCursor(65, 85);
	tft.println("Library");
	tft.setCursor(25, 150);
	tft.setTextSize(2);
	tft.setTextColor(BLACK);
	tft.println("Test of BITMAP");
	delay(1000);
	tft.fillScreen(BLACK);

	//for (size_t x = 0; x < 240; x++) // se SetRotation =0 qs è il lato corto
	//{
	//	for (size_t y = 0; y < 320; y++) // se SetRotation =0 qs è il lato lungo
	//	{
	//		tft.drawPixel(x, y, BLUE);
	//	}
	//}
#pragma endregion

#pragma region SD setup
	//pinMode(10, OUTPUT);          // CS pin of SD Card Shield

	outputSerial.println("Initializing SD card...");
	if (!SD.begin(SD_CS)) {
		outputSerial.println("sd init failed!");
		//return;
	}
	else
	{
	//progmemPrintln(PSTR("sd init done."));
	outputSerial.println("sd init done.");

	}



#pragma endregion
#pragma region CameraSetup
	cameraSerial.begin(115200);
	cameraSYNC();



#pragma endregion

}
/*********************************************************************/
void loop()
{
	int n = 0;
	while (1) {
		delay(2000);
		//if (n == 0) cameraInitial(COLORSETTING_COLOR16BIT,PREVIEWRESOLUTION_160X120, JPEGRESOLUTION_320X240);
		if (n == 0) cameraInitial(COLORSETTING_COLOR16BIT, PREVIEWRESOLUTION_160X120, 0x07);
		//cameraSETPACKAGESIZE();
		cameraGETPICTURE(GETSETTING_PREVIEWPICT);
		//cameraSNAPSHOT(SNAPSHOTSETTING_COMPRESSED);
		GetData_toLCD();		//		GetData_toLCD();

			 
		n++;
		 
	}
}

//#############################################################################
/*


#ifndef _MSC_VER
#ifdef __cplusplus
#define stbi_inline inline
#else
#define stbi_inline
#endif
#else
#define stbi_inline __forceinline
#endif
// should produce compiler error if size is wrong
typedef unsigned char validate_uint32[sizeof(stbi__uint32) == 4 ? 1 : -1];
typedef struct
{
	stbi_uc  fast[1 << FAST_BITS];
	// weirdly, repacking this into AoS is a 10% speed loss, instead of a win
	stbi__uint16 code[256];
	stbi_uc  values[256];
	stbi_uc  size[257];
	unsigned int maxcode[18];
	int    delta[17];   // old 'firstsymbol' - old 'firstcode'
} stbi__huffman;

typedef struct
{
	stbi__context *s;
	stbi__huffman huff_dc[4];
	stbi__huffman huff_ac[4];
	stbi_uc dequant[4][64];
	stbi__int16 fast_ac[4][1 << FAST_BITS];

	// sizes for components, interleaved MCUs
	int img_h_max, img_v_max;
	int img_mcu_x, img_mcu_y;
	int img_mcu_w, img_mcu_h;

	// definition of jpeg image component
	struct
	{
		int id;
		int h, v;
		int tq;
		int hd, ha;
		int dc_pred;

		int x, y, w2, h2;
		stbi_uc *data;
		void *raw_data, *raw_coeff;
		stbi_uc *linebuf;
		short   *coeff;   // progressive only
		int      coeff_w, coeff_h; // number of 8x8 coefficient blocks
	} img_comp[4];

	stbi__uint32   code_buffer; // jpeg entropy-coded buffer
	int            code_bits;   // number of valid bits
	unsigned char  marker;      // marker seen while filling entropy buffer
	int            nomore;      // flag if we saw a marker so must stop

	int            progressive;
	int            spec_start;
	int            spec_end;
	int            succ_high;
	int            succ_low;
	int            eob_run;

	int scan_n, order[4];
	int restart_interval, todo;

	// kernels
	void(*idct_block_kernel)(stbi_uc *out, int out_stride, short data[64]);
	void(*YCbCr_to_RGB_kernel)(stbi_uc *out, const stbi_uc *y, const stbi_uc *pcb, const stbi_uc *pcr, int count, int step);
	stbi_uc *(*resample_row_hv_2_kernel)(stbi_uc *out, stbi_uc *in_near, stbi_uc *in_far, int w, int hs);
} stbi__jpeg;

///////////////////////////////////////////////
//
//  stbi__context struct and start_xxx functions

// stbi__context structure is our basic context used by all images, so it
// contains all the IO context, plus some basic image information
typedef struct
{
	stbi__uint32 img_x, img_y;
	int img_n, img_out_n;

	stbi_io_callbacks io;
	void *io_user_data;

	int read_from_callbacks;
	int buflen;
	stbi_uc buffer_start[128];

	stbi_uc *img_buffer, *img_buffer_end;
	stbi_uc *img_buffer_original, *img_buffer_original_end;
} stbi__context;
typedef unsigned char stbi_uc;
typedef stbi_uc *(*resample_row_func)(stbi_uc *out, stbi_uc *in0, stbi_uc *in1,
	int w, int hs);
typedef struct
{
	resample_row_func resample;
	stbi_uc *line0, *line1;
	int hs, vs;   // expansion factor in each axis
	int w_lores; // horizontal pixels pre-expansion
	int ystep;   // how far through vertical expansion we are
	int ypos;    // which pre-expansion row we're on
} stbi__resample;
static void stbi__refill_buffer(stbi__context *s)
{
	int n = (s->io.read)(s->io_user_data, (char*)s->buffer_start, s->buflen);
	if (n == 0) {
		// at end of file, treat same as if from memory, but need to handle case
		// where s->img_buffer isn't pointing to safe memory, e.g. 0-byte file
		s->read_from_callbacks = 0;
		s->img_buffer = s->buffer_start;
		s->img_buffer_end = s->buffer_start + 1;
		*s->img_buffer = 0;
	}
	else {
		s->img_buffer = s->buffer_start;
		s->img_buffer_end = s->buffer_start + n;
	}
}
stbi_inline static stbi_uc stbi__get8(stbi__context *s)
{
	if (s->img_buffer < s->img_buffer_end)
		return *s->img_buffer++;
	if (s->read_from_callbacks) {
		stbi__refill_buffer(s);
		return *s->img_buffer++;
	}
	return 0;
}
static  stbi_uc *stbi__readval(stbi__context *s, int channel, stbi_uc *dest)
{
	int mask = 0x80, i;

	for (i = 0; i<4; ++i, mask >>= 1) {
		if (channel & mask) {
			if (stbi__at_eof(s)) return stbi__errpuc("bad file", "PIC file too short");
			dest[i] = stbi__get8(s);
		}
	}

	return dest;
}
static stbi_uc *stbi__pic_load_core(stbi__context *s, int width, int height, int *comp, stbi_uc *result)
{
	int act_comp = 0, num_packets = 0, y, chained;
	stbi__pic_packet packets[10];

	// this will (should...) cater for even some bizarre stuff like having data
	// for the same channel in multiple packets.
	do {
		stbi__pic_packet *packet;

		if (num_packets == sizeof(packets) / sizeof(packets[0]))
			return stbi__errpuc("bad format", "too many packets");

		packet = &packets[num_packets++];

		chained = stbi__get8(s);
		packet->size = stbi__get8(s);
		packet->type = stbi__get8(s);
		packet->channel = stbi__get8(s);

		act_comp |= packet->channel;

		if (stbi__at_eof(s))          return stbi__errpuc("bad file", "file too short (reading packets)");
		if (packet->size != 8)  return stbi__errpuc("bad format", "packet isn't 8bpp");
	} while (chained);

	*comp = (act_comp & 0x10 ? 4 : 3); // has alpha channel?

	for (y = 0; y<height; ++y) {
		int packet_idx;

		for (packet_idx = 0; packet_idx < num_packets; ++packet_idx) {
			stbi__pic_packet *packet = &packets[packet_idx];
			stbi_uc *dest = result + y*width * 4;

			switch (packet->type) {
			default:
				return stbi__errpuc("bad format", "packet has bad compression type");

			case 0: {//uncompressed
				int x;

				for (x = 0; x < width; ++x, dest += 4)
					if (!stbi__readval(s, packet->channel, dest))
						return 0;
				break;
			}

			}
			
		}
	}

	return result;
}
static stbi_uc *stbi__pic_load(stbi__context *s, int *px, int *py, int *comp, int req_comp)
{
	stbi_uc *result;
	int i, x, y;

	for (i = 0; i<92; ++i)
		stbi__get8(s);

	x = stbi__get16be(s);
	y = stbi__get16be(s);
	if (stbi__at_eof(s))  return stbi__errpuc("bad file", "file too short (pic header)");
	if ((1 << 28) / x < y) return stbi__errpuc("too large", "Image too large to decode");

	stbi__get32be(s); //skip `ratio'
	stbi__get16be(s); //skip `fields'
	stbi__get16be(s); //skip `pad'

					  // intermediate buffer is RGBA
	result = (stbi_uc *)stbi__malloc(x*y * 4);
	memset(result, 0xff, x*y * 4);

	if (!stbi__pic_load_core(s, x, y, comp, result)) {
		STBI_FREE(result);
		result = 0;
	}
	*px = x;
	*py = y;
	if (req_comp == 0) req_comp = *comp;
	result = stbi__convert_format(result, 4, req_comp, x, y);

	return result;
}
static unsigned char *stbi__load_main(stbi__context *s, int *x, int *y, int *comp, int req_comp)
{
#ifndef STBI_NO_JPEG
	if (stbi__jpeg_test(s)) return stbi__jpeg_load(s, x, y, comp, req_comp);
#endif
#ifndef STBI_NO_PNG
	if (stbi__png_test(s))  return stbi__png_load(s, x, y, comp, req_comp);
#endif
#ifndef STBI_NO_BMP
	if (stbi__bmp_test(s))  return stbi__bmp_load(s, x, y, comp, req_comp);
#endif
#ifndef STBI_NO_GIF
	if (stbi__gif_test(s))  return stbi__gif_load(s, x, y, comp, req_comp);
#endif
#ifndef STBI_NO_PSD
	if (stbi__psd_test(s))  return stbi__psd_load(s, x, y, comp, req_comp);
#endif
#ifndef STBI_NO_PIC
	if (stbi__pic_test(s))  return stbi__pic_load(s, x, y, comp, req_comp);
#endif
#ifndef STBI_NO_PNM
	if (stbi__pnm_test(s))  return stbi__pnm_load(s, x, y, comp, req_comp);
#endif

#ifndef STBI_NO_HDR
	if (stbi__hdr_test(s)) {
		float *hdr = stbi__hdr_load(s, x, y, comp, req_comp);
		return stbi__hdr_to_ldr(hdr, *x, *y, req_comp ? req_comp : *comp);
	}
#endif

#ifndef STBI_NO_TGA
	// test tga last because it's a crappy test!
	if (stbi__tga_test(s))
		return stbi__tga_load(s, x, y, comp, req_comp);
#endif

	return stbi__errpuc("unknown image type", "Image not of any known type, or corrupt");
}
static stbi_uc *load_jpeg_image(stbi__jpeg *z, int *out_x, int *out_y, int *comp, int req_comp)
{
	int n, decode_n;
	z->s->img_n = 0; // make stbi__cleanup_jpeg safe

					 // validate req_comp
	if (req_comp < 0 || req_comp > 4) return stbi__errpuc("bad req_comp", "Internal error");

	// load a jpeg image from whichever source, but leave in YCbCr format
	if (!stbi__decode_jpeg_image(z)) { stbi__cleanup_jpeg(z); return NULL; }

	// determine actual number of components to generate
	n = req_comp ? req_comp : z->s->img_n;

	if (z->s->img_n == 3 && n < 3)
		decode_n = 1;
	else
		decode_n = z->s->img_n;

	// resample and color-convert
	{
		int k;
		unsigned int i, j;
		stbi_uc *output;
		stbi_uc *coutput[4];

		stbi__resample res_comp[4];

		for (k = 0; k < decode_n; ++k) {
			stbi__resample *r = &res_comp[k];

			// allocate line buffer big enough for upsampling off the edges
			// with upsample factor of 4
			z->img_comp[k].linebuf = (stbi_uc *)stbi__malloc(z->s->img_x + 3);
			if (!z->img_comp[k].linebuf) { stbi__cleanup_jpeg(z); return stbi__errpuc("outofmem", "Out of memory"); }

			r->hs = z->img_h_max / z->img_comp[k].h;
			r->vs = z->img_v_max / z->img_comp[k].v;
			r->ystep = r->vs >> 1;
			r->w_lores = (z->s->img_x + r->hs - 1) / r->hs;
			r->ypos = 0;
			r->line0 = r->line1 = z->img_comp[k].data;

			if (r->hs == 1 && r->vs == 1) r->resample = resample_row_1;
			else if (r->hs == 1 && r->vs == 2) r->resample = stbi__resample_row_v_2;
			else if (r->hs == 2 && r->vs == 1) r->resample = stbi__resample_row_h_2;
			else if (r->hs == 2 && r->vs == 2) r->resample = z->resample_row_hv_2_kernel;
			else                               r->resample = stbi__resample_row_generic;
		}

		// can't error after this so, this is safe
		output = (stbi_uc *)stbi__malloc(n * z->s->img_x * z->s->img_y + 1);
		if (!output) { stbi__cleanup_jpeg(z); return stbi__errpuc("outofmem", "Out of memory"); }

		// now go ahead and resample
		for (j = 0; j < z->s->img_y; ++j) {
			stbi_uc *out = output + n * z->s->img_x * j;
			for (k = 0; k < decode_n; ++k) {
				stbi__resample *r = &res_comp[k];
				int y_bot = r->ystep >= (r->vs >> 1);
				coutput[k] = r->resample(z->img_comp[k].linebuf,
					y_bot ? r->line1 : r->line0,
					y_bot ? r->line0 : r->line1,
					r->w_lores, r->hs);
				if (++r->ystep >= r->vs) {
					r->ystep = 0;
					r->line0 = r->line1;
					if (++r->ypos < z->img_comp[k].y)
						r->line1 += z->img_comp[k].w2;
				}
			}
			if (n >= 3) {
				stbi_uc *y = coutput[0];
				if (z->s->img_n == 3) {
					z->YCbCr_to_RGB_kernel(out, y, coutput[1], coutput[2], z->s->img_x, n);
				}
				else
					for (i = 0; i < z->s->img_x; ++i) {
						out[0] = out[1] = out[2] = y[i];
						out[3] = 255; // not used if n==3
						out += n;
					}
			}
			else {
				stbi_uc *y = coutput[0];
				if (n == 1)
					for (i = 0; i < z->s->img_x; ++i) out[i] = y[i];
				else
					for (i = 0; i < z->s->img_x; ++i) *out++ = y[i], *out++ = 255;
			}
		}
		stbi__cleanup_jpeg(z);
		*out_x = z->s->img_x;
		*out_y = z->s->img_y;
		if (comp) *comp = z->s->img_n; // report original components, not output
		return output;
	}
}
static unsigned char *stbi__jpeg_load(stbi__context *s, int *x, int *y, int *comp, int req_comp)
{
	stbi__jpeg j;
	j.s = s;
	stbi__setup_jpeg(&j);
	return load_jpeg_image(&j, x, y, comp, req_comp);
}
*/