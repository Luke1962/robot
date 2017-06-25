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
#include <Arduino.h>
//#include <MemoryFree/MemoryFree.h>
//************************************************************************
//*	http://www.nongnu.org/avr-libc/user-manual/malloc.html
//*	thanks to John O for the pointer to this info and the insperation to do it
#pragma region memory monitor
#include <Streaming/Streaming.h>
#include <malloc.h>
#if 0
void	Ram_TableDisplay(void)
{
	char stack = 1;
	extern char *__data_start;
	extern char *__data_end;

	extern char *__bss_start;
	extern char *__bss_end;
	//extern char *__brkval;
	extern char *__heap_start;
	extern char *__heap_end;
	//extern char *__malloc_heap_end;
	//extern size_t __malloc_margin;

#define  RAMEND        0x10FF  //da http://www.nongnu.org/avr-libc/user-manual/malloc.html
	int	data_size = (int)&__data_end - (int)&__data_start;
	int	bss_size = (int)&__bss_end - (int)&__data_end;
	int	heap_end = (int)&stack;// -(int)&__malloc_margin;
	//	int	heap_size	=	(int)__brkval - (int)&__bss_end;
	int	heap_size = heap_end - (int)&__bss_end;
	int	stack_size = RAMEND - (int)&stack + 1;
	int	available = (RAMEND - (int)&__data_start + 1);

	available -= data_size + bss_size + heap_size + stack_size;

	Serial.print("+----------------+  __data_start  =");	Serial.println((int)&__data_start);
	Serial.print("+      data      +");						Serial.println();
	Serial.print("+    variables   +  data_size     =");	Serial.println(data_size);
	Serial.print("+                +");						Serial.println();
	Serial.print("+----------------+  __data_end    =");	Serial.println((int)&__data_end);
	Serial.print("+----------------+  __bss_start   =");	Serial.println((int)&__bss_start);
	Serial.print("+       bss      +");						Serial.println();
	Serial.print("+    variables   +  bss_size      =");	Serial.println(bss_size);
	Serial.print("+                +");						Serial.println();
	Serial.print("+----------------+  __bss_end     =");	Serial.println((int)&__bss_end);
	Serial.print("+----------------+  __heap_start  =");	Serial.println((int)&__heap_start);
	Serial.print("+                +");						Serial.println();
	Serial.print("+       heap     +  heap_size     =");	Serial.println(heap_size);
	Serial.print("+                +");						Serial.println();
	Serial.print("+----------------+  heap_end      =");	Serial.println(heap_end);
	Serial.print("+----------------+  Current STACK =");	Serial.println((int)&stack);
	Serial.print("+                +");						Serial.println();
	Serial.print("+      stack     +  stack_size    =");	Serial.println(stack_size);
	Serial.print("+                +");						Serial.println();
	Serial.print("+----------------+  RAMEND        =");	Serial.println(RAMEND);

	//	Serial.print("__brkval      =");
	//	Serial.println((int)__brkval);

	Serial.print("available =");
	Serial.println(available);

	Serial.println();
	Serial.println();
}

#endif // 0
	extern char _end;
	extern "C" char *sbrk(int i);
void ReportfreeRam()
{
	char *ramstart = (char *)0x20070000;
	char *ramend = (char *)0x20088000;
	char *heapend = sbrk(0);
 	char *stack_ptr = (char *)alloca(0); // also works

	struct mallinfo mi = mallinfo();
	Serial << "Ram used (bytes): " << endl
		<< "  dynamic: " << mi.uordblks << endl
		<< "  static:  " << &_end - ramstart << endl
		<< "  stack:   " << ramend - stack_ptr << endl;
	Serial << "Estimation free Ram: " << stack_ptr - heapend + mi.fordblks << endl;
}

#if 0
int get_free_memory()
{
	extern char __bss_end;
	extern char *__brkval;

	int free_memory;

	if ((int)__brkval == 0)
		free_memory = ((int)&free_memory) - ((int)&__bss_end);
	else
		free_memory = ((int)&free_memory) - ((int)__brkval);

	return free_memory;
}

#endif // 0

#pragma endregion

#pragma region debug

#define dbg(cha) 	 Serial.println(F(cha));
#define dbg2(t,cha)	 Serial.print(F(t));Serial.println(cha);  
#define dbgHex(b)	 Serial.print(b,HEX);Serial.print(" ");  
//#include <MemoryFree/MemoryFree.h>

#pragma endregion
#pragma region Parametri dell'applicazione'

#define MAXJPGSIZE 7000
#include "arduino.h"

void convertToDistance(int tftX, int tftY);
void renderJPEG(int xpos, int ypos);
void subtractJPEG(  byte threshold);
void subtractJPEG2(const uint8_t jpgArray[], byte threshold);
void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos);
void JpegInfoToSerial();
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
unsigned int CaptureAdnGetImageSize(); 

void GetPicture_toLCD(byte c0, byte c1, byte c2, byte c3, byte c4, byte c5, unsigned int pictureSize_X, unsigned int pictureSize_Y);
void GetData_SD();
void GetCameraJpegData_toArray(unsigned int picTotalLen, uint8_t array[]);
//
//
uint8_t jpgImgRef[MAXJPGSIZE];//contenuto immagine senza laser acceso
uint8_t jpgImgArray[MAXJPGSIZE];
uint8_t jpgImgArray2[MAXJPGSIZE];
#define DETECT_THRESHOLD_RED 50  //Soglia per la differenza nel calae rosso per individuare il laser
const int buttonPin = A5;                 // the number of the pushbutton pin


unsigned int picTotalLen = 0;            // picture length
int picNameNum = 0;



#pragma endregion

#include <SPI.h>


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

#define PIC_PKT_LEN    720      //was 720            //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0

#define PIC_FMT        PIC_FMT_VGA

#define CAM_SERIAL     Serial3


#pragma endregion

// converte tftX in alfa e tftY in distanza (cm) e li mette in un array
void convertToDistance(int tftX, int tftY) {
//	const int lensAngle = 56;
//	int alfa =  (160-tftX )*lensAngle/320;
//	dbg2("alfa", alfa)
//	int dist = tftY;
}

#if 1
#pragma region Debug_PushColors
void dbg_spiTransfer(uint16_t _data) {
	uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS);

	SPI0->SPI_CSR[ch] = (SPI0->SPI_CSR[ch] &= 0xFFFFFF0F) | 0x00000080;	//set 16 bit
	uint32_t d = _data | SPI_PCS(ch);

	d |= SPI_TDR_LASTXFER;

	// SPI_Write(spi, _channel, _data);
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
		;
	SPI0->SPI_TDR = d;


	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
		;
	SPI0->SPI_RDR;
	SPI0->SPI_CSR[ch] &= 0xFFFFFF0F; //restore 8bit
}

void dbg_spiTransfer(uint8_t _data) {
	uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS);
	uint32_t d = _data | SPI_PCS(ch);
	d |= SPI_TDR_LASTXFER;

	// SPI_Write(spi, _channel, _data);
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
		;
	SPI0->SPI_TDR = d;

	// return SPI_Read(spi);
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
		;
	SPI0->SPI_RDR;
}
void dbg_spiTransfer(const uint8_t *_buf, uint32_t _count) {
	if (_count == 0)
		return;

	if (_count == 1) {
		dbg_spiTransfer(*_buf);
		return;
	}

	uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS);

	// Send the first byte
	uint32_t d = *_buf;

	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
		;
	SPI0->SPI_TDR = d | SPI_PCS(ch);

	while (_count > 1) {
		// Prepare next byte
		d = *(_buf + 1);

		if (_count == 2)
			d |= SPI_TDR_LASTXFER;

		// Read transferred byte and send next one straight away
		while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
			;
		SPI0->SPI_RDR;
		SPI0->SPI_TDR = d | SPI_PCS(ch);

		// Save read byte

		//*_buf = r;
		_buf++;
		_count--;
	}


	// Receive the last transferred byte
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
		;
	SPI0->SPI_RDR;
	//*_buf = r;
}
void dbg_spiTransfer(const uint16_t *_buf, uint32_t _count) {
	if (_count == 0)
		return;

	if (_count == 1) {
		dbg_spiTransfer(*_buf);
		return;
	}

	uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS);

	SPI0->SPI_CSR[ch] = (SPI0->SPI_CSR[ch] &= 0xFFFFFF0F) | 0x00000080;	//set 16 bit
																		// Send the first byte
	uint32_t d = *_buf;

	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
		;
	SPI0->SPI_TDR = d | SPI_PCS(ch);

	while (_count > 1) {
		// Prepare next byte
		d = *(_buf + 1);

		if (_count == 2)
			d |= SPI_TDR_LASTXFER;

		// Read transferred byte and send next one straight away
		while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
		SPI0->SPI_RDR;
		SPI0->SPI_TDR = d | SPI_PCS(ch);

		// Save read byte

		//*_buf = r;
		_buf++;
		_count--;
	}


	// Receive the last transferred byte
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
		;
	SPI0->SPI_RDR;
	//*_buf = r;
	SPI0->SPI_CSR[ch] &= 0xFFFFFF0F; //restore 8bit
}

inline __attribute__((always_inline))
void dbg_write_cont(uint16_t* buf, uint32_t n) {
#if SPI_MODE_NORMAL
 
	dbg_spiTransfer(buf, n);

#elif SPI_MODE_DMA
	dmaSend(buf, n);
#endif
}
//__attribute__((always_inline))
void dbg_beginTransaction() {
#ifdef ILI_USE_SPI_TRANSACTION
#if defined ARDUINO_ARCH_AVR
	SPI.beginTransaction(_spiSettings);
#elif defined (ARDUINO_SAM_DUE)
#if SPI_MODE_NORMAL
	SPI.beginTransaction(_spiSettings);
#elif SPI_MODE_EXTENDED
	SPI.beginTransaction(_cs, _spiSettings);
#elif SPI_MODE_DMA
	SPI.beginTransaction(_spiSettings);
	dmaInit(_spiClkDivider);
#endif
#endif
#endif
}

//__attribute__((always_inline))
void dbg_endTransaction() {
#ifdef ILI_USE_SPI_TRANSACTION
#if defined ARDUINO_ARCH_AVR
	SPI.endTransaction();
#elif defined (ARDUINO_SAM_DUE)
	SPI.endTransaction();
#endif
#endif
}


//// Enables CS
//inline __attribute__((always_inline))
//void dbg_enableCS() {
//#if SPI_MODE_NORMAL | SPI_MODE_DMA
//	*_csport &= ~_cspinmask;
//#endif
//}
//// Disables CS
//inline __attribute__((always_inline))
//void dbg_disableCS() {
//#if SPI_MODE_NORMAL | SPI_MODE_DMA
//	*_csport |= _cspinmask;
//	//csport->PIO_SODR  |=  cspinmask;
//#elif SPI_MODE_EXTENDED
//	writecommand_last(ILI9341_NOP);	// have to send a byte to disable CS
//#endif
//}
//// Sets DC to Data (1)
//inline __attribute__((always_inline))
//void dbg_setDCForData() {
//	*_dcport |= _dcpinmask;
//	//_dcport->PIO_SODR |= _dcpinmask;
//}
//
//// Sets DC to Command (0)	
//inline __attribute__((always_inline))
//void dbg_setDCForCommand() {
//	*_dcport &= ~_dcpinmask;
//}
void dbg_pushColors(uint16_t *colors, uint16_t offset, uint32_t len) {
	dbg_beginTransaction();
	tft.enableCS();
	tft.setDCForData();
	colors = colors + offset;
	dbg_write_cont(colors, len);
	tft.disableCS();
	dbg_endTransaction();
}

#pragma endregion

#endif // 0

#pragma region funzioni DecodeJpeg
//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void renderJPEG(int xpos, int ypos) {
	if (xpos < 0) xpos = 0;
	if (ypos < 0) ypos = 0;
	
	if (xpos >tft.width() ) return;
	if (ypos >tft.height() ) return;

	//jpegInfo(); // Print information from the JPEG file (could comment this line out)
	uint16_t tftW = tft.width();
	uint16_t tftH = tft.height();
	uint16_t *pImg;
	uint16_t mcu_w = JpegDec.MCUWidth;    // Width of MCU
	uint16_t mcu_h = JpegDec.MCUHeight;   // Height of MCU
	JpegDec.scanType = PJPG_YH1V1;
	uint32_t mcu_pixels = mcu_w * mcu_h;  // Total number of pixels in an MCU
	//dbg2("\tMCU_pixels:", mcu_pixels)
										  // Serial.print("comp size = ");Serial.println(comp_size);

//	uint32_t drawTime = millis(); // For comparison purpose the draw time is measured

								  // Fetch data from the file, decode and display
	while (JpegDec.read()) {    // While there is more data in the file

		pImg = JpegDec.pImage;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

		int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
		int mcu_y = JpegDec.MCUy * mcu_h + ypos;
		//dbg2("mcu_x", mcu_x)
		//dbg2("mcu_y", mcu_y)

		if ((mcu_x + mcu_w) <= tftW && (mcu_y + mcu_h) <= tftH)
		{
			// Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
			//tft.setWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);
			tft.setAddrWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);
/*			// Push all MCU pixels to the TFT window
			uint32_t count = mcu_pixels;
			while (count--) {
				// Push each pixel to the TFT MCU area
				tft.pushColor(*pImg++);
			}
			//uint16_t offst = 0;*/

			// Push all MCU pixels to the TFT window, ~18% faster to pass an array pointer and length to the library
			//if (JpegDec.available())
			//{
			tft.pushColors(pImg, 0, mcu_pixels); //<<<<<  si impianta qui dentro in 'decode_mcu'

			//}
			//else
			//{
			//	Serial << "@@@";
			//}
			
		}
		else
		{
			dbg("\tNo more data to read")

			if ((mcu_y + (int)mcu_h) >= (int)tftH) {
			dbg("\t\tAborting!")

				JpegDec.abort(); // Image has run off bottom of screen so abort decoding
			
			}

		}
	}

	//	showTime(millis() - drawTime); // These lines are for sketch testing only
	//Serial.print(" Draw count:");
	//Serial.println(icount++);
}
void dbg_renderJPEG(int xpos, int ypos) {
	if (xpos < 0) xpos = 0;
	if (ypos < 0) ypos = 0;

	if (xpos >tft.width()) return;
	if (ypos >tft.height()) return;

	//jpegInfo(); // Print information from the JPEG file (could comment this line out)
	uint16_t tftW = tft.width();
	uint16_t tftH = tft.height();
	uint16_t *pImg;
	uint16_t mcu_w = JpegDec.MCUWidth;    // Width of MCU
	uint16_t mcu_h = JpegDec.MCUHeight;   // Height of MCU
	uint32_t mcu_pixels = mcu_w * mcu_h;  // Total number of pixels in an MCU
										  //dbg2("MCU_pixels:", mcu_pixels)
										  // Serial.print("comp size = ");Serial.println(comp_size);

										  //	uint32_t drawTime = millis(); // For comparison purpose the draw time is measured

										  // Fetch data from the file, decode and display
	while (JpegDec.read()) {    // While there is more data in the file

		pImg = JpegDec.pImage;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

		int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
		int mcu_y = JpegDec.MCUy * mcu_h + ypos;
		//dbg2("mcu_x", mcu_x)
		//dbg2("mcu_y", mcu_y)

		if ((mcu_x + mcu_w) <= tftW && (mcu_y + mcu_h) <= tftH)
		{
			// Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
			//tft.setWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);
			delayMicroseconds(1);
			tft.setAddrWindow(mcu_x, mcu_y, mcu_x + mcu_w - 1, mcu_y + mcu_h - 1);
			delayMicroseconds(1);
			/*			// Push all MCU pixels to the TFT window
			uint32_t count = mcu_pixels;
			while (count--) {
			// Push each pixel to the TFT MCU area
			tft.pushColor(*pImg++);
			}
			//uint16_t offst = 0;*/

			// Push all MCU pixels to the TFT window, ~18% faster to pass an array pointer and length to the library


			delayMicroseconds(1);
			//tft.pushColors(pImg, 0, mcu_pixels); //<<<<<  si impianta qui
			dbg("[")
				//dbg_pushColors(pImg, 0, mcu_pixels); //<<<<<  si impianta qui

				///provo a mettere qui dbg_pushColors
				//			dbg_beginTransaction();
				tft.enableCS();
			tft.setDCForData();

			dbg_write_cont(pImg, mcu_pixels);
			tft.disableCS();
			//			dbg_endTransaction();

			//dbg_write_cont(pImg, mcu_pixels);
			delayMicroseconds(1);
			dbg("]")
				// tft.pushColor16(pImg, mcu_pixels); //  To be supported in HX8357 library at a future date
		}
		else
		{
			JpegDec.abort(); // Image has run off bottom of screen so abort decoding
			//if ((mcu_y + (int)mcu_h) >= (int)tftH) {
			//	JpegDec.abort(); // Image has run off bottom of screen so abort decoding

			//}

		}
	}

	//	showTime(millis() - drawTime); // These lines are for sketch testing only
	//Serial.print(" Draw count:");
	//Serial.println(icount++);
}
//####################################################################################################
// Subtract a JPEG in jpgArray from the image currently displayed on TFT 
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void subtractJPEG( byte threshold /*Soglia*/) {
	int xpos = 0;
	int ypos = 0;
	dbg("\n Esegue IMG_TFT - IMG_RAM")
	//jpegInfo(); // Print information from the JPEG file (could comment this line out)
	int tftX;
	int tftY;
	uint16_t pxTFT;
	uint16_t pxRAM;
	uint16_t redTFT;
	uint16_t greenTFT;
	uint16_t bluTFT;
	uint16_t redRAM;
	uint16_t greenRAM;
	uint16_t bluRAM;
	int16_t r;
	int16_t g;
	int16_t b;

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
				pxTFT = tft.readPixel(tftX, tftY);
				//estraggo le componenti r g b:  rrrrgg ggbbbb
				redTFT = pxTFT >> 10;
				greenTFT = ( pxTFT >> 5) & 0x3F;
				bluTFT = pxTFT   & 0x3F;
				//if (red>60)
				//{
				//	dbgHex(red)
				//	dbg2("@x",tftX)
				//	dbg2("@y", tftY)
				//}

				//valore immagine senza laser
				pxRAM = *pImg;
				//estraggo le componenti r g b:  XXXXrrrrggggbbbb
				redRAM = pxRAM >> 10;
				greenRAM = ( pxRAM >> 5) & 0x3F;
				bluRAM = pxRAM   & 0x3F;


#if 1
				// faccio la  differenza
				r = redTFT - redRAM; if (r < 0) r = 0;
				g = greenTFT - greenRAM;if (g < 0) g = 0;
				b = bluTFT - bluRAM;if (b < 0) b = 0;
 
				if (threshold>0) //se la soglia è > 0  esalto il rosso
				{
					if (r>threshold)
					{
						r = 0x0FFF;
						g = 0; b = 0;

					}
					else
					{
						r = 0x0;
						g = 0; b = 0;
					}

				}
 
#else
				//differenza con soglia 

				r = max(redTFT - redRAM, 0);
				g = max(greenTFT - greenRAM, 0);
				b = max(bluTFT - bluRAM, 0);

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

				// passo al pixel successivo---------------------
				pImg++;  //was *pImg++;

				tftX++;
				// fine riga ?
				if (tftX>= mcu_x + mcu_w){
					tftX = mcu_x;
					tftY++;
				}

				//-----------------------------------------------




			}


		}
		else {
			JpegDec.abort();
		//	if ((mcu_y + mcu_h) >= tft.height()) {
		//	JpegDec.abort(); // Image has run off bottom of screen so abort decoding
		//}

		}
	}

	//	showTime(millis() - drawTime); // These lines are for sketch testing only
	//Serial.print(" Draw count:");
	//Serial.println(icount++);
}
/* PROVO a passare l'array*/
void subtractJPEG2(const uint8_t jpgArray[], byte threshold /*Soglia*/) {
	int xpos = 0;
	int ypos = 0;
	dbg("\n Esegue IMG_TFT - IMG_RAM")
	//jpegInfo(); // Print information from the JPEG file (could comment this line out)
	int tftX;
	int tftY;
	uint16_t pxTFT;
	uint16_t pxRAM;
	uint16_t redTFT;
	uint16_t greenTFT;
	uint16_t bluTFT;
	uint16_t redRAM;
	uint16_t greenRAM;
	uint16_t bluRAM;
	int16_t r;
	int16_t g;
	int16_t b;

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

				//valore corrente dell'immagine su TFT con laser ON
				pxTFT = tft.readPixel(tftX, tftY);

				//estraggo le componenti r g b:  rrrrgg ggbbbb
				redTFT = pxTFT >> 10;
				greenTFT = ( pxTFT >> 5) & 0x3F;
				bluTFT = pxTFT   & 0x3F;
				//if (red>60)
				//{
				//	dbgHex(red)
				//	dbg2("@x",tftX)
				//	dbg2("@y", tftY)
				//}

				//valore immagine senza laser
				pxRAM = *pImg;
 

				//estraggo le componenti r g b:  rrrrrrgggg ggbbbbbb
				redRAM = pxRAM >> 10;
				greenRAM = ( pxRAM >> 5) & 0x3F; //0x3F = 0011 1111
				bluRAM = pxRAM   & 0x3F;


#if 0
				// faccio la  differenza
				r = redTFT - redRAM; if (r < 0) r = 0;
				g = greenTFT - greenRAM;if (g < 0) g = 0;
				b = bluTFT - bluRAM;if (b < 0) b = 0;
 
				if (threshold>0) //se la soglia è > 0  esalto il rosso
				{
					if (r>threshold)
					{
						r = 0x0FFF;
						g = 0; b = 0;

					}
					else
					{
						r = 0x0;
						g = 0; b = 0;
					}

				}
 
#endif // 1
#if 1
				// faccio la  differenza
				r = redRAM; if (r < 0) r = 0;
				g = greenRAM;if (g < 0) g = 0;
				b = bluRAM;if (b < 0) b = 0;
 
 
#endif // 1

				//dbgHex(r)
				//dbgHex(g)
				//dbgHex(b)

				// Push each pixel to the TFT MCU area

				//tft.pushColor( px-px1);
				//*pImg=tft.color565(r, g, b);
				//tft.pushColor(*pImg);		// original
				tft.drawPixel(tftX, tftY, tft.color565(r, g, b));

				// passo al pixel successivo---------------------
				pImg++;  //was *pImg++;

				tftX++;
				// fine riga ?
				if (tftX>= mcu_x + mcu_w){
					tftX = mcu_x;
					tftY++;
				}

				//-----------------------------------------------




			}


		}
		else {
			JpegDec.abort();
		//	if ((mcu_y + mcu_h) >= tft.height()) {
		//	JpegDec.abort(); // Image has run off bottom of screen so abort decoding
		//}

		}
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
void JpegInfoToSerial() {

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
void initial(byte c0, byte c1, byte c2, byte imgFormat, byte rawSetting, byte jpegResolution)
{
	dbg("\r\nInitial...")
		char cmd[6];
	cmd[0] = CMD_PREFIX;
	cmd[1] = CMD_INITIAL;
	cmd[2] = 0x00;
	cmd[3] = imgFormat; //img format
	cmd[4] = rawSetting; //RAW setting
	cmd[5] = jpegResolution; 

//	unsigned char resp[6];

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
//	unsigned char resp[6];

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
/* memorizza nel buffer della SerialCamera l'immagine */
void snapshot(byte c0, byte c1, byte snapshotType, byte c3, byte c4, byte c5)
{
	char cmd[6];
	cmd[0] = 0xaa;
	cmd[1] = CMD_SNAPSHOT;
	cmd[2] = snapshotType;//snapshot type 0=jpeg, 1=raw
	cmd[3] = c3;//skipframe Low
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
/********************************************************************/
/*1) Invia il comando SetPackageSize	0x06						*/
/*2) Invia il comando Snapshot			0x05						*/
/*3) Invia il comando GetData			0x04						*/
/*4) riceve l'immagine della SerialCamera							*/
/********************************************************************/
void Capture_orig() //Versione originale
{

	//1) invia il comando CMD_SETPKGSIZE fino a ricevimento di ACK
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
	//-------------------------------------------------------------

	//2) invia il comando SNAPSHOT 0x05 fino a ricevimento di ACK
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
	//-------------------------------------------------------------
	
	
	
	
	//3 )invia il comando CMD_GETPICTURE 0x04 fino a ricevimento di ACK
	cmd[1] = CMD_GETPICTURE ;
	cmd[2] = GETSETTING_SNAPSHOT;

	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0)
		{
			// ok ricevuto ack posso iniziare a ricevere -------------
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
/********************************************************************/
/*1) Invia il comando SetPackageSize	0x06						*/
/*2) Invia il comando Snapshot			0x05						*/
/*3) Invia il comando GetData			0x04						*/
/*4) riceve l'immagine della SerialCamera							*/
/********************************************************************/
unsigned int CaptureAdnGetImageSize() //Versione originale
{

	//1) invia il comando CMD_SETPKGSIZE fino a ricevimento di ACK
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
	//-------------------------------------------------------------

	//2) invia il comando SNAPSHOT 0x05 fino a ricevimento di ACK
	cmd[1] = CMD_SNAPSHOT;
	cmd[2] = 0;
	cmd[3] = 0;
	cmd[4] = 0;
	cmd[5] = 0;
	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == CMD_SNAPSHOT && resp[4] == 0 && resp[5] == 0) break;
	}
	//-------------------------------------------------------------




	//3 )invia il comando CMD_GETPICTURE 0x04 fino a ricevimento di ACK
	cmd[1] = CMD_GETPICTURE;
	cmd[2] = GETSETTING_SNAPSHOT;

	while (1)
	{
		clearRxBuf();
		sendCmd(cmd, 6);
		if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == 0x0e && resp[2] == CMD_GETPICTURE && resp[4] == 0 && resp[5] == 0)
		{
			// ok ricevuto ack posso iniziare a ricevere -------------
			CAM_SERIAL.setTimeout(1000);
			if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
			{
				continue;
			}
			if (resp[0] == 0xaa && resp[1] == (0x0a) && resp[2] == 0x01)
			{
				picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
				dbg2("\npicTotalLen:",picTotalLen);
				return picTotalLen;
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
	//int row = 1;
	//int	col = 1;	// riga e colonna corrente del TFT
	uint8_t  r, g, b;

	uint16_t color = 0; //colore
	//	uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)

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

/****************************************************************************/
/* da chiamare dopo CaptureAdnGetImageSize()								*/
/* calcola il numero di pacchetti e per ogni pacchetto						*/
/* invia alla serialCamera le richieste di pacchetti dati col comando 0x0e	*/
/****************************************************************************/
void GetCameraJpegData_toArray(unsigned int picTotalLen, uint8_t array[])  //versione che usa jpegDecoder
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
 
	// PULISCO IL RESTO DELL'ARRAY -------------
	for (int i = jpgImgEnd; i < MAXJPGSIZE; i++)
	{
		array[jpgImgEnd] = 0;
	}


	jpgImgEnd = 0;
	dbg("\n Fine trasf. jpg")
 

}
/*********************************************************************/

#pragma endregion

unsigned int jpgSize = 0;
uint16_t px;
byte red = 0;
byte green = 0;
byte blu = 0;
byte soglia = DETECT_THRESHOLD_RED;


void setup()
{
	Serial.begin(115200);
  // set the slaveSelectPin as an output:
  pinMode(9, OUTPUT);
  // initialize SPI:
  SPI.begin();
#pragma region ILI9341_due tft  setup
	//Serial.begin(9600);

	 tft.begin();

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

#pragma region Laser Setup
	#define PIN_LINELASER 23
	#define LASER_ON 	digitalWrite(PIN_LINELASER, 1);
	#define LASER_OFF 	digitalWrite(PIN_LINELASER, 0);
	#define LASER_TOGGLE digitalWrite(PIN_LINELASER, !digitalRead(PIN_LINELASER));
	pinMode(PIN_LINELASER, OUTPUT);

#pragma endregion

#pragma region SERIAL_CAMERA SETUP

	CAM_SERIAL.begin(115200);
	CAM_SERIAL.setTimeout(5000);


//	cameraSYNC();
	initialize();


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

#pragma endregion




}
/*********************************************************************/
int loopCnt = 0;
void loop0()  //VISUALIZZA ALTERNATE LE IMMAGINI CON E SENZA LASER ACCESO
// risultati test: si pianta solo dopo 2307 loop
{
	loopCnt++;
	if (Serial.available())
	{
		soglia = (byte)Serial.parseInt();
		Serial.print("\n OK Soglia:"); Serial.println(soglia);
	}
	tft.printAt(String(soglia), 10, 50);

	// catturo l'immagine con il laser acceso
	LASER_TOGGLE
 	//Capture_orig();// catturo l'immagine 
	jpgSize = CaptureAdnGetImageSize();//Capture_orig();

	GetCameraJpegData_toArray(jpgSize, jpgImgArray); //METTE L'IMMAGINE jpeg nell'array jpgImg
	JpegDec.decodeArray(jpgImgArray, jpgSize); // decodifica l'array in pImage

	dbg("\nRendering to LCD..")
	// catena chiamate
	// renderJPEG > JPEGdecoder::read() > JPEGDecoder::decode_mcu() >pjpeg_decode_mcu()
	renderJPEG(0, 0); //Draw a JPEG on the TFT (usa (JpegDec.read() ).

	//JpegInfoToSerial();


	tft.printAt(String(loopCnt), 10, 10);
	delay(10);

}

void loop()  //VISUALIZZA ALTERNATE LE IMMAGINI CON E SENZA LASER ACCESO
{
	loopCnt++;
	if (Serial.available())
	{
		soglia = (byte)Serial.parseInt();
		Serial.print("\n OK Soglia:"); Serial.println(soglia);
	}
	tft.printAt(String(soglia), 10, 50);

	// catturo l'immagine con il laser acceso
	LASER_TOGGLE
 	//Capture_orig();// catturo l'immagine 
	jpgSize = CaptureAdnGetImageSize();//Capture_orig();

	GetCameraJpegData_toArray(jpgSize, jpgImgArray); //METTE L'IMMAGINE jpeg nell'array jpgImg
	JpegDec.decodeArray(jpgImgArray, jpgSize); // decodifica l'array in pImage

	dbg("\nRendering to LCD..")
	// catena chiamate
	// renderJPEG > JPEGdecoder::read() > JPEGDecoder::decode_mcu() >pjpeg_decode_mcu()
	renderJPEG(0, 0); //Draw a JPEG on the TFT (usa (JpegDec.read() ).

	//JpegInfoToSerial();


	tft.printAt(String(loopCnt), 10, 10);
	delay(10);

}
/*********************************************************************/
void loop20()  //TEST DIFFERENZA IMMAGINI 
{
	loopCnt++;
	if (Serial.available())
	{
		soglia = (byte)Serial.parseInt();
		Serial.print("\n OK Soglia:"); Serial.println(soglia);
	}
	tft.printAt(String(soglia), 10, 50);
 	// catturo l'immagine con il laser acceso
	LASER_ON
	jpgSize =CaptureAdnGetImageSize();//Capture_orig();

	GetCameraJpegData_toArray(jpgSize,jpgImgArray); //METTE L'IMMAGINE jpeg nell'array jpgImg
	JpegDec.decodeArray(jpgImgArray, jpgSize); // decodifica l'array da jpgImg in pImage
	ReportfreeRam();

	dbg("\nRendering with LASER ON to LCD..")
	renderJPEG(0, 0);
	JpegInfoToSerial();


	// catturo l'immagine con il laser SPENTO
	LASER_OFF

	jpgSize = CaptureAdnGetImageSize();//Capture_orig();

	GetCameraJpegData_toArray(jpgSize,jpgImgArray);//METTE L'IMMAGINE jpeg nell'array jpgImg
	
 	 // decodeArray > decodeCommon() > decode_mcu() > pjpeg_decode_mcu() >decodeNextMCU()
	JpegDec.decodeArray(jpgImgArray, jpgSize);// decodifica l'array in pImage
	subtractJPEG2(jpgImgArray,soglia );  //Subtract a JPEG in RAM from the image currently displayed on TFT
	tft.printAt(String(loopCnt), 10,10);

	ReportfreeRam();
}
	//---------------------------------------------------------------------------
	// test di visualizzazione dell'immagine ripresa alternando il laser ON e OFF
	//---------------------------------------------------------------------------
void loop0()
{
	loopCnt++;
	dbg2("Start loop: \t", loopCnt)
	//
	// catturo l'immagine con il laser acceso
	dbg("\nLaser ON")
	digitalWrite(PIN_LINELASER, !digitalRead(PIN_LINELASER));
	delay(10);

	dbg("\n Capture_orig...")
	jpgSize = CaptureAdnGetImageSize();//Capture_orig();


	dbg("\n SerialCamera -> jpgImgArray")
	GetCameraJpegData_toArray(jpgSize,jpgImgArray);
	dbg2("jpegsize:",jpgSize)

	dbg("\n Decoding to array...")
	JpegDec.decodeArray(jpgImgArray, jpgSize);

	ReportfreeRam();//	Ram_TableDisplay();//
	dbg("\n Rendering to LCD..")
	renderJPEG(0, 0);
	dbg("\n**********EndLoop***************\n\n")

//	JpegInfoToSerial();
	delay(500);
  
}
/*********************************************************************/


