
//#############################################################################
// TFT shield + Serial camera
// salva su SD immagini jpeg
#pragma region SerialCamera
//#define cameraSerial Serial  //SoftwareSerial cameraSerial(2, 3); // RX, TX
//  File SerialCamera_DemoCode_CJ-OV528.ino
//  8/8/2013 Jack Shao
//  Demo code for using seeeduino or Arduino board to cature jpg format
//  picture from seeed serial camera and save it into sd card. Push the
//  button to take the a picture .
//  For more details about the product please check http://www.seeedstudio.com/depot/

#include <SPI.h>
#include <arduino.h>
#include <sd.h>
#include <SoftwareSerial.h>
SoftwareSerial SwSerial(2, 3); 

#if 0
#define cameraSerial cameraSwSerial //  Serial //Configure pin 4 and 5 as soft serial port
#define outputSerial Serial

#else
#define cameraSerial Serial
#define outputSerial SwSerial
#endif // 0

#pragma region debug
#define dbg(cha) 	 outputSerial.println(cha);
#define dbg2(t,cha)	 outputSerial.print(t);outputSerial.println(cha);  

#pragma endregion



#define PIC_PKT_LEN    128                  //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0


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

//#define CAM_SERIAL     Serial

#define PIC_FMT        PIC_FMT_VGA


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
	//outputSerial.print("\nCMD OUT [");
	//for (char i = 0; i < cmd_len; i++) outputSerial.print(cmd[i],HEX); outputSerial.print(" ");
	//outputSerial.println("]");

}
/*********************************************************************/
/// Invia Sync 0D fino a 60 volte
void cameraSYNC()
{
	bool AckReceived = false;
	//cameraSerial.begin(9600);
	char cmd[] = { 0xaa,CMD_SYNC ,0x00,0x00,0x00,0x00 };
	uint8_t i = 0;
	cameraSerial.setTimeout(50);
	while (i<60)
	{
		//delay(100);
			//clearRxBuf();
			sendCmd(cmd, 6);
			dbg("< SYNC")

			AckReceived =whaitAck(CMD_SYNC,100);
			if (AckReceived)
			{
				if(whaitSync(1000)) break;
			}
			else {
				i++;
				//dbg2("SYNC try cnt:",i)
			}
	}
	cmd[1] = 0x0e | cameraAddr;
	cmd[2] = CMD_SYNC;
	sendCmd(cmd, 6);
	dbg("Camera SYNC done.")
}
/*********************************************************************/
/// Attende l'ACK sul comando passato
bool whaitAck(uint8_t cmd, int msTimeout) {
	cameraSerial.setTimeout(msTimeout);
	unsigned char resp[6];
	bool ack = false;
	while (!ack)
	{
		if (cameraSerial.readBytes((char *)resp, 6) != 6)
		{
			for (uint8_t i = 0; i < strlen((char *)resp); i++) { outputSerial.print(resp[i], HEX);  outputSerial.print(" "); }
			
			continue;
		}
		else
		{
			if (resp[0] == 0xaa && 
				resp[1] == 0x0e && 
				resp[2] == cmd && 
				resp[4] == 0 && 
				resp[5] == 0) 
			{
				dbg(">ACK")
				ack = true;
			}
		}
	}
	return ack;
}
/*********************************************************************/
/// Attende l'ACK sul comando passato
bool whaitSync(  int msTimeout) {
//	dbg("Wait sync")

	cameraSerial.setTimeout(msTimeout);
	unsigned char resp[6];
	bool end = false;
	while (!end)
	{
		if (cameraSerial.readBytes((char *)resp, 6) != 6)
		{
			continue;
		}
		else
		{
			if (resp[0] == 0xaa &&
				resp[1] == 0x0d &&
				resp[2] == 0 &&
				resp[3] == 0 &&
				resp[4] == 0 &&
				resp[5] == 0)
			{
				end = true;
				dbg(">SYNC")

			}
		}
	}
	return end;
}

void cameraInitialize()
{
	char cmd[] = { 0xaa,0x0d | cameraAddr,0x00,0x00,0x00,0x00 };
	unsigned char resp[6];

	cameraSerial.setTimeout(500);
	while (1)
	{
		delay(100);
		//clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6)
		{
			continue;
		}
		if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
		{
			if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
			if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
		}
	}
	cmd[1] = 0x0e | cameraAddr;
	cmd[2] = 0x0d;
	sendCmd(cmd, 6);
	cameraSerial.println("\nCamera initialization done.");
}
/*********************************************************************/
void preCapture()	// invia CMD_INITIAL
{
	char cmd[] = { CMD_PREFIX, CMD_INITIAL | cameraAddr, 0x00, COLORSETTING_JPEG, 0x03, JPEGRESOLUTION_320X240 };
	unsigned char resp[6];

	cameraSerial.setTimeout(100);
	while (1) //ripeti finchè non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK | cameraAddr) && resp[2] == CMD_INITIAL && resp[4] == 0 && resp[5] == 0) break;
	}
}


void Capture()
{
	char cmd[] = { CMD_PREFIX, CMD_SETPKGSIZE | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN >> 8) & 0xff ,0 };
	unsigned char resp[6];

	cameraSerial.setTimeout(100);

	while (1) // invia il comando finchè non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK | cameraAddr) && resp[2] == CMD_SETPKGSIZE && resp[4] == 0 && resp[5] == 0) break;
	}

	cmd[1] = CMD_SNAPSHOT | cameraAddr;
	cmd[2] = 0;
	cmd[3] = 0;
	cmd[4] = 0;
	cmd[5] = 0;
	while (1) // invia il comando finchè non ricevi ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK | cameraAddr) && resp[2] == CMD_SNAPSHOT && resp[4] == 0 && resp[5] == 0) break;
	}


	cmd[1] = CMD_GETPICTURE | cameraAddr;
	cmd[2] = GETSETTING_SNAPSHOT;
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK | cameraAddr) && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0)
		{
			cameraSerial.setTimeout(1000);
			if (cameraSerial.readBytes((char *)resp, 6) != 6)
			{
				continue;
			}
			if (resp[0] == CMD_PREFIX && resp[1] == (CMD_DATA | cameraAddr) && resp[2] == DATATYPE_SNAPSHOT)
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
void GetData_NoFile()
{


	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

	char cmd[] = { CMD_PREFIX, CMD_ACK | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
	unsigned char pkt[PIC_PKT_LEN];

	//char picName[] = "pic00.jpg";
	//picName[3] = picNameNum / 10 + '0';
	//picName[4] = picNameNum % 10 + '0';
	//if (SD.exists(picName))
	//{
	//	SD.remove(picName);
	//}
	//myFile = SD.open(picName, FILE_WRITE);
	//if (!myFile) {
	//	cameraSerial.println("myFile open fail...");
	//}
	//else {
	cameraSerial.setTimeout(1000);

	//per ogni pacchetto dati
	for (unsigned int i = 0; i < pktCnt; i++)
	{
		cmd[4] = i & 0xff;	// Package ID Byte 0
		cmd[5] = (i >> 8) & 0xff; //Package ID Byte 1

		int retry_cnt = 0;
	retry:
		delay(10);
		clearRxBuf();
		sendCmd(cmd, 6);// chiedi l'invio dei dati
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

		//outputSerial.write((const uint8_t *)&pkt[4], cnt - 6);
		//if (cnt != PIC_PKT_LEN) break;

		for (size_t j = 0; j < cnt - 6; j++)
		{

			outputSerial.print(pkt[4 + j], HEX); outputSerial.print(" ");
		}



	}
	cmd[4] = 0xf0;
	cmd[5] = 0xf0;
	sendCmd(cmd, 6);
	//}
	//myFile.close();
	//picNameNum++;
}



/*********************************************************************/
void GetData_File()
{


	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

	char cmd[] = { CMD_PREFIX, CMD_ACK | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
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
		cameraSerial.println("myFile open fail...");
	}
	else {
	cameraSerial.setTimeout(1000);

	//per ogni pacchetto dati
	for (unsigned int i = 0; i < pktCnt; i++)
	{
		cmd[4] = i & 0xff;	// Package ID Byte 0
		cmd[5] = (i >> 8) & 0xff; //Package ID Byte 1

		int retry_cnt = 0;
	retry:
		delay(10);
		clearRxBuf();
		sendCmd(cmd, 6);// chiedi l'invio dei dati
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

		//outputSerial.write((const uint8_t *)&pkt[4], cnt - 6);
		//if (cnt != PIC_PKT_LEN) break;

		for (size_t j = 0; j < cnt - 6; j++)
		{

			outputSerial.print(pkt[4 + j], HEX); outputSerial.print(" ");
		}



	}
	cmd[4] = 0xf0;
	cmd[5] = 0xf0;
	sendCmd(cmd, 6);
	//}
	//myFile.close();
	//picNameNum++;
}


/*********************************************************************/
/** cameraInitialPreview  *******************************************************************/
/*********************************************************************/
/*********************************************************************/

void cameraInitialPreview()	// invia CMD_INITIAL
{
	int repeat;

	char cmd[] = { CMD_PREFIX, CMD_INITIAL | cameraAddr, 0x00, COLORSETTING_COLOR16BIT, 0x07, JPEGRESOLUTION_80X64 };
 	unsigned char resp[6];

	cameraSerial.setTimeout(100);
	while (1) //ripeti finchè non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK | cameraAddr) && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0) break;
		repeat++;
		dbg2("repeat",repeat)

	}
}
/*********************************************************************/
/**  GetPreviewPicture_NoFile   *******************************************************************/
/*********************************************************************/
void GetPreviewPicture_NoFile()
{

#pragma region cmd "GetPicture Preview Picture" con attesa ACK
	char cmd[] = { CMD_PREFIX, CMD_GETPICTURE | cameraAddr, GETSETTING_PREVIEWPICT, 0x00 , 0x00 , 0x00 };
	unsigned char resp[6];

	cameraSerial.setTimeout(100);
	while (1) //ripeti finchè non ricevi un ACK
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (cameraSerial.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_ACK | cameraAddr) && resp[2] == CMD_INITIAL && resp[4] == 0 && resp[5] == 0) break;
	}

#pragma endregion
	while (1)
	{
		// ricezione picture size ----
		if (cameraSerial.readBytes((char *)resp, 6) != 6)
		{
			continue;
		}
		if (resp[0] == CMD_PREFIX && resp[1] == (CMD_DATA | cameraAddr) && resp[2] == DATATYPE_PREVIEW)
		{
			picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
			outputSerial.print("\nFrame Len:");
			outputSerial.println(picTotalLen);
			break;
		}

	}



	//unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	//if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

	// ricezione frame image data ----
	//for (size_t i = 0; i < picTotalLen; i++)
	//{
	//	outputSerial.print(cameraSerial.read(), HEX); outputSerial.print(" ");
	//}

	// invio ACK-------------------------------------------------------------
	cmd[1] = CMD_ACK;
	cmd[2] = CMD_DATA;
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	sendCmd(cmd, 6);

}


#pragma endregion
/*********************************************************************/

byte incomingbyte;
int a = 0x0000, j = 0, k = 0, count = 0;                    //Read Starting address       
uint8_t MH, ML;
boolean EndFlag = 0;

//void SendResetCmd();
//void SendTakePhotoCmd();
//void SendReadDataCmd();
//void StopTakePhotoCmd();

void setup()
{
	outputSerial.begin(9600);
	outputSerial.println("TestSerialCamera");
	delay(1000);
	cameraSerial.begin(115200);


	//dbg("calling cameraSYNC")
	//	cameraSYNC();

	dbg("calling cameraInitialize")
	cameraInitialize();

	dbg("calling cameraInitialPreview")

	cameraInitialPreview();// invia CMD_INITIAL


}

void loop()
{
	int n = 0;
	while (1) {
	dbg2("frame ",n);

		delay(2000);
		if (n == 0) preCapture();
		Capture();
		GetData();

		//GetPreviewPicture_NoFile();
		n++;

	}

	//while(cameraSerial.available()>0)
	// {
	//   incomingbyte=cameraSerial.read();

	// }   
	// byte a[32];

	// while(!EndFlag)
	// {  
	//    j=0;
	//    k=0;
	//    count=0;
	//    SendReadDataCmd();

	//    delay(25);
	//     while(cameraSerial.available()>0)
	//     {
	//          incomingbyte=cameraSerial.read();
	//          k++;
	//          if((k>5)&&(j<32)&&(!EndFlag))
	//          {
	//          a[j]=incomingbyte;
	//          if((a[j-1]==0xFF)&&(a[j]==0xD9))      //Check if the picture is over
	//          EndFlag=1;                           
	//          j++;
	//    count++;
	//          }
	//     }

	//     for(j=0;j<count;j++)
	//     {   if(a[j]<0x10)
	//         Serial.print("0");
	//         Serial.print(a[j],HEX);
	//         Serial.print(" ");
	//     }                                       //Send jpeg picture over the serial port
	//     Serial.println();
	// }      
	//while(1);
}

