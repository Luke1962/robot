// Arduino DUE + Serial Camera + TFT seriale
//#############################################################################�
#define dbg(cha) 	 Serial.println(cha);
#define dbg2(t,cha)	 Serial.print(t);Serial.println(cha);  
#define OutputSerial Serial	// messaggi 
#define dbgHex(b)	 Serial.print(b,HEX);Serial.print(" ");  


#define BUFFPIXEL 20
#pragma region ILI9341_due tft 


#include <spi/SPI.h>
#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

#include "arduino.h"

int wait_for_ack(uint8_t* command);
void sendCmd_SYNC();
bool blACKreceived(byte cmd);
void echoCameraInputForever();
void preCaptureQVGAUncompressed();
void clearRxBuf();
void sendCmd(char cmd[], int cmd_len);
void sendCmd_INITIAL();
void cameraSYNC();
void cameraInitial(uint8_t colSetting, uint8_t size);
void cameraInitial_new();
long getImgSize();
//
//
ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion

//#############################################################################�
#pragma region SerialCamera
#if 1
#define CAM_SERIAL Serial1
#else
#include <SoftwareSerial/SoftwareSerial.h>

SoftwareSerial cameraSerial(2, 3); // RX, TX//  //

#endif // 0


uint8_t   _INITIAL[6] = { 0xAA,0x01,0x00,0x03,0x09,0x00 }; //RAW 8-bit gray scale 128x128
uint8_t   _GET_PICTURE[6] = { 0xAA,0x04,0x01,0x00,0x00,0x00 }; // SnapShot picture mode
uint8_t   _SNAPSHOT[6] = { 0xAA,0x05,0x01,0x00,0x00,0x00 }; // RAW - current frame
uint8_t   _PACK_SIZE[6] = { 0xAA,0x06,0x08,0x00,0x02,0x00 }; // only for JPEG - 512B
uint8_t	  _SET_BAUD_RATE[6] = { 0xAA,0x07,0x1F,0x00,0x00,0x00 }; // 115200 
uint8_t   _SYNC_COMMAND[6] = { 0xAA,0x0D,0x00,0x00,0x00,0x00 };
uint8_t   _ACK_COMMAND[6] = { 0xAA,0x0E,0x0D,0x00,0x00,0x00 };
uint8_t   _ACK_PICTURE_COMMAND[6] = { 0xAA,0x0E,0x0A,0x00,0x01,0x00 };
uint8_t   _PACKET_ACK[6] = { 0xAA,0x0E,0x00,0x00,0x00,0x00 };


char   _ACK_PICTURE_SIZE[6] = { 0xAA,0x0E,0x0A,0x00,0x00,0x00 };
char   _ACK_STARTSENDINGDATA[6] = { 0xAA,0x0E,0x00,0x00,0x00,0x00 };
char   _ACK_PICTURE[6] = { 0xAA,0x0E,0x0A,0x00,0x01,0x00 };


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
#define GETSETTING_RAWPICT 0x02
#define GETSETTING_JPEGPREVIEW 0x03




#define COLORSETTING_BW2BIT 0x01
#define COLORSETTING_BW4BIT 0x02
#define COLORSETTING_BW8BIT 0x03
#define COLORSETTING_COLOR2BIT 0x05
#define COLORSETTING_COLOR16BIT 0x06
#define COLORSETTING_JPEG 0x07

// deve essere coerente con l'imostazione RAW___X__ utilizzata
#define PICT_SIZE_X 80 
#define PICT_SIZE_Y 60 
#define PICT_OFFSET_X 0
#define PICT_OFFSET_Y 0


#define RAW_80X60 0x01
#define RAW_160X120 0x03
#define RAW_128X128 0x09

#define JPEGRESOLUTION_80X64 0x01
#define JPEGRESOLUTION_160X128 0x03
#define JPEGRESOLUTION_320X240 0x05
#define JPEGRESOLUTION_640X480 0x07

#define SNAPSHOTSETTING_COMPRESSED 0x00
#define SNAPSHOTSETTING_UNCOMPRESSED 0x01

#define DATATYPE_SNAPSHOT 0x01
#define DATATYPE_RAW 0x02
#define DATATYPE_JPEGPICT 0x05

#define PIC_PKT_LEN    10      //was 128            //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0
#define CAM_SERIAL     Serial3

#define PIC_FMT        PIC_FMT_VGA

#pragma endregion


const byte cameraAddr = (CAM_ADDR << 5);  // addr
const int buttonPin = A5;                 // the number of the pushbutton pin
unsigned long picTotalLen = 0;            // picture length
int picNameNum = 0;

#pragma region non usate 

int wait_for_ack(uint8_t* command) {
	uint8_t cam_reply;

	int wait_success = 0;
	int last_reply = 0;
	long t0 = millis();

	while (!CAM_SERIAL.available() && millis() - t0 < 3000);

	if (millis() - t0 > 3000)
		return wait_success;

	Serial.println("Wait Ack");

	while (CAM_SERIAL.available()) {

		cam_reply = CAM_SERIAL.read();
		Serial.println(cam_reply, HEX);
		Serial.flush();

		//delay(100);
		if (cam_reply == command[last_reply] && last_reply == 0) {
			last_reply++;
			//Serial.println("matched 0");
		}
		else
			if (cam_reply == command[last_reply] && last_reply == 1) {
				last_reply++;
				//Serial.println("matched 1");
			}
			else
				if (cam_reply == command[last_reply] && last_reply == 2) {
					last_reply++;
					//Serial.println("matched 2");
				}
				else
					if (last_reply == 3) {
						last_reply++;
						//Serial.println("skipped 3");
					}
					else
						if (cam_reply == command[last_reply] && last_reply == 4) {
							last_reply++;
							//Serial.println("matched 4");
						}
						else
							if (cam_reply == command[last_reply] && last_reply == 5) {
								last_reply++;
								//Serial.println("matched 5 - success");
								wait_success = 1;
								break;
							}
	}

	return wait_success;
}

/*********************************************************************/
void sendCmd_SYNC()
{
	for (short i = 0; i < 6; i++) CAM_SERIAL.print(_SYNC_COMMAND[i]);
}

/*********************************************************************/
bool blACKreceived(byte cmd) {
	CAM_SERIAL.setTimeout(100);
	unsigned char resp[6];
	int b = CAM_SERIAL.readBytes((char *)resp, 6);
	// stampo cosa ricevo 		
	for (size_t i = 0; i < 6; i++) {
		OutputSerial.print(resp[i], HEX); OutputSerial.print(" ");
	}
	OutputSerial.println(" ");
	if (resp[0] == CMD_PREFIX
		&& resp[1] == CMD_ACK
		&& resp[2] == cmd
		&& resp[4] == 0
		&& resp[5] == 0) {
		return true;
	}
	else
		return false;

}
void echoCameraInputForever() {
	char c;
	while (true)
	{
		//attendi finch� non ci sono dati
		while (!CAM_SERIAL.available() );

		while (CAM_SERIAL.available())
		{
			c = CAM_SERIAL.read();
			OutputSerial.print(c,HEX);OutputSerial.print(" ");
		}

	}

}
/*********************************************************************/
void preCaptureQVGAUncompressed()
{
	char cmd[] = { CMD_PREFIX, CMD_INITIAL  , 0x00, COLORSETTING_JPEG, RAW_160X120, JPEGRESOLUTION_320X240 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);
	while (1) //ripeti finch� non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_INITIAL && resp[4] == 0 && resp[5] == 0) break;
	}
}
/*********************************************************************/

void cameraSETPACKAGESIZE(uint16_t pkgSize = PIC_PKT_LEN) {
	char cmd[] = { CMD_PREFIX, CMD_SETPKGSIZE , 0x08, pkgSize & 0xff, (pkgSize >> 8) & 0xff ,0 };
	unsigned char resp[6];
	CAM_SERIAL.setTimeout(100);
	while (1) // invia il comando finch� non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (Serial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_SETPKGSIZE && resp[4] == 0 && resp[5] == 0) break;
	}

}
/*********************************************************************/

void cameraSNAPSHOT(uint8_t compression = SNAPSHOTSETTING_COMPRESSED, uint16_t skipframe = 0)
{
	unsigned char resp[6];

	// cameraSETPACKAGESIZE
	cameraSETPACKAGESIZE(PIC_PKT_LEN);
	char cmd[6] = { CMD_PREFIX, CMD_SNAPSHOT , compression ,skipframe & 0xff, (skipframe >> 8) & 0xff,0 };


	while (1) // invia il comando finch� non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (Serial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_SNAPSHOT && resp[4] == 0 && resp[5] == 0) break;
	}

}
/*********************************************************************/


#pragma endregion



/*********************************************************************/
void clearRxBuf()
{
	while (CAM_SERIAL.available())
	{
		CAM_SERIAL.read();
	}
}
/*********************************************************************/
void sendCmd(char cmd[], int cmd_len)
{
	OutputSerial.print("[");
	for (char i = 0; i < cmd_len; i++) {
		CAM_SERIAL.print(cmd[i]);
		OutputSerial.print(cmd[i], HEX); OutputSerial.print(" ");
	}
	OutputSerial.println("]");


}

/*********************************************************************/
void sendCmd_INITIAL()
{
	for (short i = 0; i < 6; i++) CAM_SERIAL.print(_INITIAL[i]);
}
/*********************************************************************/

void cameraSYNC()
{
	int retries = 0;
	char cmd[] = { CMD_PREFIX,CMD_SYNC  ,0x00,0x00,0x00,0x00 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(500);
	while (1)
	{
		//clearRxBuf();
		sendCmd(cmd, 6);
		while (!CAM_SERIAL.available() == 6) { delay(10); }
		for (size_t i = 0; i < 6; i++) {
			resp[i] = CAM_SERIAL.read();
			Serial.print(resp[i], HEX);
		}
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
	dbg(" Camera SYNC done.");
}
/*********************************************************************/

/*********************************************************************/
void cameraInitial(uint8_t colSetting, uint8_t size) {	
	OutputSerial.println("cameraInitial...");

	char cmd[] = { CMD_PREFIX, CMD_INITIAL  , 0x00,colSetting, size, 0x07 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);
	while (1) //ripeti finch� non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_INITIAL && resp[4] == 0 && resp[5] == 0) { 
			OutputSerial.println("OK cameraInitial!");
			break; 
		}
	}
}

void cameraInitial_new() {
	char cmd[] = { CMD_PREFIX, CMD_INITIAL  , 0x00,COLORSETTING_COLOR16BIT, RAW_160X120, 0x07 };
	unsigned char resp[6];

	CAM_SERIAL.setTimeout(100);

	do //ripeti finch� non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);

	} while (!blACKreceived(CMD_INITIAL));
	Serial.println("cameraInitial!");
}
/*********************************************************************/

long getImgSize() {
	OutputSerial.println("Getting ImgSize...");

	char c;
	int i = 0;

	uint8_t data[6];
	// attendo dati
	while (!CAM_SERIAL.available()) ;
	// attendo 0xAA
	while (CAM_SERIAL.available())
	{
		c = CAM_SERIAL.read();
		OutputSerial.print(c, HEX);
		if (c == 0xAA)
		{
			data[i++] = c;
			break;
		}
	}
	// carico i successivi byte
	while (CAM_SERIAL.available() && i<6) {
		c = CAM_SERIAL.read();
		Serial.println(c, HEX);
		data[i++] = c;
	}

	// check the second byte
	if (data[1] != 0x0A)
		return false;

	// check the second byte
	if (data[2] != 0x02)
		return false;


	// ok get size

	picTotalLen = (data[3]) | (data[4] << 8) | (data[5] << 16);

	OutputSerial.print("picTotalLen:"); 	OutputSerial.println(picTotalLen);

	return picTotalLen;

}
/*********************************************************************/

void GetRAWPictureToLCD(uint8_t colorSetting, uint8_t size, uint8_t getsettings = GETSETTING_RAWPICT)
{
#pragma region Comando GetPicture con ACK
	OutputSerial.println("cameraGETPICTURE...");
	char cmd[] = { CMD_PREFIX, CMD_GETPICTURE ,getsettings,  0x00,  0x00 ,0x00 };
	unsigned char resp[6];

	bool blAck = false;
	do
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		//if (!blACKreceived(CMD_GETPICTURE)) { blAck = true; }
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK) && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0) {
			blAck = true;
			Serial.println("ACK GETPICTURE!");
//			break;
			}


	} while (!blAck);
#pragma endregion

#pragma region get Image Size
	// leggo la dimensione dell'immagine
	//picTotalLen = getImgSize();

	OutputSerial.println("waiting img size...");
	while (CAM_SERIAL.available() < 6) { delay(10); };
	CAM_SERIAL.readBytes((char *)resp, 6);
	if (resp[0] == CMD_PREFIX && resp[1] == (CMD_DATA) && resp[2] == DATATYPE_RAW)
	{
		picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
		OutputSerial.print("picTotalLen:");
		OutputSerial.println(picTotalLen);
	}
#pragma endregion


//	OutputSerial.println("Getting RAW Data to LCD...");

	//************************************************************
	// setup per la visualizzazion su LCD
	//************************************************************
	#pragma region  setup per la visualizzazion su LCD

		int16_t lcdX = PICT_OFFSET_X;
		int16_t	lcdY = PICT_OFFSET_Y;	// riga e colonna corrente del TFT
		uint8_t  r, g, b;

		uint16_t color = 0; //colore
		//uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)

	#pragma endregion

/*


	cmd[1] = CMD_ACK;
	cmd[2] = 0x00;

	CAM_SERIAL.setTimeout(1000);
	// se lo tolgo non invia dati
	sendCmd(cmd, 6);// chiedi l'invio dei dati (tutto il frame se in RAW)
*/
	int px;



	for (int lcdX = 0; lcdX < PICT_SIZE_X; lcdX++)
	{
		for (int lcdY = 0; lcdY < PICT_SIZE_Y; lcdY++)
		{
			//attendi finch� non ci sono dati
			while (!CAM_SERIAL.available()) { delayMicroseconds(5); }

			px = CAM_SERIAL.read() >>2;
			color = tft.color565(px, px, px);
			tft.drawPixel(lcdX, lcdY, color); // col = lcdY lato lungo

		}
	}



/*

	for (unsigned long i = 0; i < picTotalLen; i++)
	{
		switch (colorSetting)
		{
		case COLORSETTING_BW8BIT:
		//attendi finch� non ci sono dati
			while (!CAM_SERIAL.available()) { delayMicroseconds(5); }

			px = CAM_SERIAL.read()>>2;
			color = tft.color565(px, px, px);
			break;

		case COLORSETTING_COLOR16BIT:
			while (!CAM_SERIAL.available()) { delayMicroseconds(10); }
			r = CAM_SERIAL.read() >> 2;
			while (!CAM_SERIAL.available()) { delayMicroseconds(10); }
			g = CAM_SERIAL.read() >> 2;
			while (!CAM_SERIAL.available()) { delayMicroseconds(10); }
			b = CAM_SERIAL.read() >> 2;
			while (!CAM_SERIAL.available()) { delayMicroseconds(10); }
			color = tft.color565(r, g, b);
			break;

		default:
			break;
		}

		dbg2("lcdX", lcdX)
		dbg2("lcdY",lcdY)

		OutputSerial.print(px, HEX); OutputSerial.print(" ");



		if (lcdX= PICT_OFFSET_X)		{		OutputSerial.print("\n|");		}


		#pragma region Visualizzazione dei dati su TFT
		//************************************************************
		// qui devo mandare il pixel sul display
		//************************************************************
		tft.drawPixel(lcdX, lcdY, color); // col = lcdY lato lungo

		//tft.drawPixel(row, col, color); // col = lcdY lato lungo
		lcdX++;

		// se ultima colonna resetto la colonna e incremento la riga
		if (lcdX >= PICT_OFFSET_X + PICT_SIZE_X) { 
			lcdY++;
			lcdX = PICT_OFFSET_X;
			dbg2("\ny:",lcdY)
		}// dbg2("row:",row)

		//ultima riga?
		if (lcdY >= PICT_OFFSET_Y + PICT_SIZE_Y) {
			lcdY = PICT_OFFSET_Y; 		
			lcdX = PICT_OFFSET_X;
		}
		#pragma endregion

	}

*/

#pragma region send OK fine frame
	cmd[1] = CMD_ACK;
	cmd[2] = 0x0A;
	cmd[3] = 0xF0;
	cmd[4] = 0xF0;

	sendCmd(cmd, 6);// ACK DEL frame

#pragma endregion

	
//echoCameraInputForever();
  

		//lcdbuffer[lcdidx++] = tft.color565(r, g, b);
		// Convert pixel from BMP to TFT format
		//b = pkt[4];
		//g = pkt[5];
		//r = pkt[6];



  
}
/*********************************************************************/






#pragma endregion
/*********************************************************************/

void setup()
{
	OutputSerial.begin(115200);
#pragma region ILI9341_due tft  setup
	//Serial.begin(9600);

	bool result = tft.begin();


	tft.setRotation(iliRotation270);
	tft.fillScreen(ILI9341_BLUE);

	tft.setFont(Arial_bold_14);
	tft.setTextLetterSpacing(5);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
	tft.printAligned(F("Hello World"), gTextAlignMiddleCenter);

	for (size_t x = PICT_OFFSET_X; x < PICT_OFFSET_X + PICT_SIZE_X; x++)
	{
		for (size_t y = PICT_OFFSET_Y; y < PICT_OFFSET_Y + PICT_SIZE_Y; y++)
		{
			tft.drawPixel(x, y, ILI9341_RED);

		}
	}


#pragma endregion
#pragma region CameraSetup
	CAM_SERIAL.begin(115200);
	CAM_SERIAL.setTimeout(500);
	cameraSYNC();

	cameraInitial(COLORSETTING_BW8BIT, RAW_80X60);

#pragma endregion


}
	int frame=0;

void loop()
{	
	GetRAWPictureToLCD(COLORSETTING_BW8BIT, RAW_80X60,GETSETTING_RAWPICT);
	OutputSerial.print("Frame:"); OutputSerial.println(frame++);
		
	
}


