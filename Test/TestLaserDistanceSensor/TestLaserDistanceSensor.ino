/* Ref:http://embeddedprogrammer.blogspot.in/2012/07/hacking-ov7670-camera-module-sccb-cheat.html
* ov7670 sends the data in synchronous format (D0-D7).
* Supply clock signal on the xclk pin, ranging between 10MHz to 48MHz.
*
* A video is cont. stream of frames.
* A frame consists of multiple lines(i.e. height)
* A line consists of multiple pixel dots (i.e. width)
* A pixel is dot. :)
* Monochrome image ,each pixel dot is stored as 1byte.(white dot(0) , black dot(255))
* RGB: RGB888,RGB565 & RGB555 are the formats supported by ov7670.( number represent 'bits' for each color)
*      If its RGB565, each pixel dot is stored as 2bytes
* YBR:YCbCr422
* The order in which things happen after xclk is supplied.
* 1. D0-D7 must be sampled at the rising edge of pclk signal.
* 2. D0-D7 must be sampled when href is high.
* 3. Rising edge of href is the start of line and the falling edge of href is the end of the line
* 4. Falling edge of vsync is the start of frame and rising edge of vsync is the end of the frame.
*  e.g VGA (640x480), read 640 pixels to get a line, read 480 such lines to get a frame.
* 5. By default pclk is same as xclk.however prescalars can be configured to operate pclk at different freq.
*    pclk 24MHz will generate 30fps, 12MHz -> 15fps, independent of image format.
* 6. To change fps ,we need to change pclk freq. configure following registers.
*
* 7. pclk clock
* f_internal_clk =  f_clk * PLL_multiplier / 2 * ( CLRC[5-0] +1)
* clkrc[5-0]= b000001, clkrc[6] = 0, enables prescalar/2
* dblv [7-6]= 10, to enable PLL*6
*
* Reset the camera
* to change fps, change pclk freq
* Register     Address   Default     Description
* REG_CLKRC      0x11      0x80      bit[6]: 0: Apply prescalar or i/p clock
*                                            1: Use external clock directly
*                                    bit[0-5]: clock prescalar
*                                             F(internal clock) = F(input clock)/(bit[0-5]+1)
*                                             Range [00000] to [11111]
* DBLV           0x6B      0x0A      bit[7-6]: PLL control
*                                              00: Bypass PLL
*                                              01: input clock x4
*                                              10: input clock x6
*                                              11: input clock x8
*                                    bit[4]:   Regulator control
*                                              0: Enable internal regulator
*                                              1: Bypass internal regulator
*
*
* The formula is
* clkrc =1 30fps,clkrc=2 30fps, clkrc=3 15fps
* pixclk = PLLfactor * (clock_speed / (clkrc + 1))
* fps = (5/4)*pixclk for YUV/RGB and
* fps = (5/2)*pixclk for RAW.
* e.g 	set pclk to 40MHz to get 30fps, as
* setFPS(FPS30);
*/




#pragma region LIBRERIE ILI9341_due tft 


#include <SPI.h>
#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

//#define TFT_CS 10
//#define TFT_DC 9
//#define TFT_RST 8
#define TFT_CS 2
#define TFT_DC 3
#define TFT_RST 4

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion
//#include <digitalWriteFast\digitalWriteFast.h>
#include "Arduino.h"

#pragma region conversion and processing functions

inline int digitalReadDirect(int pin) {
	return !!(g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin);
}
#define CLR(x,y) (x&=(~(1<<y)))

#define SET(x,y) (x|=(1<<y))
#define togglePortBit(port,bit) (port)|=(1<<(bit))
#define toggleLed digitalWrite(13,!digitalReadDirect(13))
typedef struct {
	double r;       // percent
	double g;       // percent
	double b;       // percent
} rgb;

typedef struct {
	double h;       // angle in degrees
	double s;       // percent
	double v;       // percent
} hsv;

static hsv rgb2hsv(rgb in);
static rgb rgb565to888(uint16_t  in);
static rgb hsv2rgb888(hsv in);
static uint16_t hsv2rgb565(hsv in);
static uint16_t rgb888to565(byte r, byte g, byte b);
static hsv rgb2hsv(rgb in)
{
	hsv         out;
	double      min, max, delta;

	min = in.r < in.g ? in.r : in.g;
	min = min  < in.b ? min : in.b;

	max = in.r > in.g ? in.r : in.g;
	max = max  > in.b ? max : in.b;

	out.v = max;                                // v
	delta = max - min;
	if (delta < 0.00001)
	{
		out.s = 0;
		out.h = 0; // undefined, maybe nan?
		return out;
	}
	if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
		out.s = (delta / max);                  // s
	}
	else {
		// if max is 0, then r = g = b = 0              
		// s = 0, v is undefined
		out.s = 0.0;
		out.h = NAN;                            // its now undefined
		return out;
	}
	if (in.r >= max)                           // > is bogus, just keeps compilor happy
		out.h = (in.g - in.b) / delta;        // between yellow & magenta
	else
		if (in.g >= max)
			out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
		else
			out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

	out.h *= 60.0;                              // degrees

	if (out.h < 0.0)
		out.h += 360.0;

	return out;
}
uint16_t rgb888to565(byte r, byte g, byte b) {
	//rrrrr ggg ggg bbbbb
	uint16_t rgb565 = 0;
	rgb565 = ((r >> 3) << 11);
	rgb565 |= ((g >> 2) << 6);
	rgb565 |= (b >> 3);
	return rgb565;
}
static rgb rgb565to888(uint16_t  in) {
	rgb  out;
	uint16_t  ins = in;
	out.r = ((in) >> 3) * 255 / 31;
	out.g = (in >> 5 & 63) * 255 / 63;
	out.b = ((in) & 31) * 255 / 31;
	return out;
}

uint16_t hsv2rgb565(uint8_t y, uint8_t u, uint8_t v) {
	double      hh, p, q, t, ff;
	long        i;
	uint8_t         r, g, b;

#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)
#define C(Y) ( (Y) - 16 )
#define D(U) ( (U) - 128 )
#define E(V) ( (V) - 128 )
#define YUV2R(Y, U, V) CLIP(( 298 * C(Y) + 409 * E(V) + 128) >> 8)
#define YUV2G(Y, U, V) CLIP(( 298 * C(Y) - 100 * D(U) - 208 * E(V) + 128) >> 8)
#define YUV2B(Y, U, V) CLIP(( 298 * C(Y) + 516 * D(U) + 128) >> 8)

	//r = y + 1.4075 * (v - 128);
	//g = y - 0.3455 * (u - 128) - (0.7169 * (v - 128));
	//b = y + 1.7790 * (u - 128);

	r = YUV2R(y, u, v);
	g = YUV2G(y, u, v);
	b = YUV2B(y, u, v);

	uint16_t rgb565 = 0;
	rgb565 = ((r >> 3) << 11);
	rgb565 |= ((g >> 2) << 6);
	rgb565 |= (b >> 3);
	return rgb565;
}

uint16_t hsv2rgb565(hsv in) {
	double      hh, p, q, t, ff;
	long        i;
	rgb         out;

	hh = in.h;
	if (hh >= 360.0) hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));
	switch (i) {
	case 0:
		out.r = in.v;
		out.g = t;
		out.b = p;
		break;
	case 1:
		out.r = q;
		out.g = in.v;
		out.b = p;
		break;
	case 2:
		out.r = p;
		out.g = in.v;
		out.b = t;
		break;

	case 3:
		out.r = p;
		out.g = q;
		out.b = in.v;
		break;
	case 4:
		out.r = t;
		out.g = p;
		out.b = in.v;
		break;
	case 5:
	default:
		out.r = in.v;
		out.g = p;
		out.b = q;
		break;
	}
	uint16_t rgb565 = (((byte)out.r >> 3) << 11) | (((byte)out.g >> 2) << 5) | ((byte)out.b & 0b00011111);
	return rgb565;

}
static rgb hsv2rgb888(hsv in)
{
	double      hh, p, q, t, ff;
	long        i;
	rgb         out;

	if (in.s <= 0.0) {       // < is bogus, just shuts up warnings
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	}
	hh = in.h;
	if (hh >= 360.0) hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));

	switch (i) {
	case 0:
		out.r = in.v;
		out.g = t;
		out.b = p;
		break;
	case 1:
		out.r = q;
		out.g = in.v;
		out.b = p;
		break;
	case 2:
		out.r = p;
		out.g = in.v;
		out.b = t;
		break;

	case 3:
		out.r = p;
		out.g = q;
		out.b = in.v;
		break;
	case 4:
		out.r = t;
		out.g = p;
		out.b = in.v;
		break;
	case 5:
	default:
		out.r = in.v;
		out.g = p;
		out.b = q;
		break;
	}
	return out;
}
#pragma endregion



#pragma region OV7670 camera
#include <Wire.h>
#include "ov7670.h"

//#define ADDRESS         0x21 //Define i2c address of OV7670
//#define REGISTERS       0xC9 //Define total numbers of registers on OV7076

#define XCLK_FREQ 10 * 1000000 //10.5 Mhz slow but ok.
//#define XCLK_FREQ 8* 1000000 //21 Mhz doubles the image; too fast?
//#define XCLK_FREQ 24 * 1000000 //42 Mhz

//original
#define VSYNC_PIN		10
#define HREF_PIN 		9
#define PCLK_PIN		8
#define XCLK_PIN		7

// ATTENZIONE COORDINARE I PIN CON LA MACRO DI LETTURA DATI
#define D0_PIN			51  // C12
#define D1_PIN			50	// C13
#define D2_PIN			49	// C14
#define D3_PIN			48	// C15
#define D4_PIN			47	// C16
#define D5_PIN			46	// C17
#define D6_PIN			45	// C18
#define D7_PIN			44	// C19
#define READCAMERADATA REG_PIOC_PDSR >> 12


#define VSYNC_BIT       (REG_PIOC_PDSR & (1 << 29))
#define HREF_BIT        (REG_PIOC_PDSR & (1 << 21))
#define PCLK_BIT        (REG_PIOC_PDSR & (1 << 22)) //pin8 is port C22 as stated here -> http://www.arduino.cc/en/Hacking/PinMappingSAM3X

#define WIDTH           ILI9341_TFTWIDTH
#define HEIGHT          ILI9341_TFTHEIGHT

#define MYCOLORSPACE	RGB565	
//#define MYCOLORSPACE	BAYER_RGB	
//#define MYCOLORSPACE	YUV422	
//#define MYCOLORSPACE	RGB888	

volatile boolean onFrame = false;
volatile boolean onRow = false;
volatile boolean onPixel = false;

uint16_t AECsatPixelCounter = 0; // conta i pixel saturati

int AECcurrentValue = 0; // valore corrente dell'esposizione
int AGCcurrentValue = 0; // valore corrente del guadagno

void SetCameraAECval(uint16_t AECnewValue)
{
	if (AECnewValue > 0)
	{
		wrReg(REG_COM1, (AECnewValue && 0b0000000000000011)); //contiene AEC[1.0]
		wrReg(REG_AECH, (AECnewValue && 0b0000001111111100) >> 2); //contiene AEC[9..2]
		wrReg(REG_AECHH, AECnewValue >> 10); // AECHH[7.6] riservati ;in AECHH[5..0]  ci va AEC[15..10]  (parte alta)
		AECsatPixelCounter = 0;
	}


}
void SetCameraAGCval(uint16_t AGCnewValue)
{
	if (AGCnewValue > 0)
	{
		if (AGCnewValue>0)
		{
			AGCcurrentValue = AGCcurrentValue >> 2;
		}
		wrReg(REG_VREF, 3 + (AGCcurrentValue && 0b0000001100000000)); //VREF[7.6] contiene gain[9.8]
		wrReg(REG_GAIN, (AGCcurrentValue && 0b0000000011111111)); // GAIN[7.0]  contiene gain[7..0]
		AECsatPixelCounter = 0;
	}


}
void printRegister(byte reg, int mode)
{
	char tmpStr[80];
	byte highByte = rdReg(reg); //Read the byte as an integer

	if (mode == 0)
	{
		Serial.print(F("0x")); if (reg < 0x10) Serial.print(0, HEX); Serial.print(reg, HEX);
		Serial.print(F(" : "));
		Serial.print(F("0x")); if (highByte < 0x10) Serial.print(0, HEX); Serial.print(highByte, HEX);
	}
	if (mode == 1)
	{
		Serial.print("Register ");
		sprintf(tmpStr, "%03d", reg); Serial.print(tmpStr);
		Serial.print(" ");
		itoa(reg, tmpStr, 16); sprintf(tmpStr, "0x%02d", atoi(tmpStr)); Serial.print(tmpStr);
		Serial.print(" ");
		itoa(reg, tmpStr, 2); sprintf(tmpStr, "0b%08d", atoi(tmpStr)); Serial.print(tmpStr);
		Serial.print(": ");
		sprintf(tmpStr, "%03d", highByte); Serial.print(tmpStr);
		Serial.print(" ");
		itoa(highByte, tmpStr, 16); sprintf(tmpStr, "0x%02d", atoi(tmpStr)); Serial.print(tmpStr);
		Serial.print(" ");
		itoa(highByte, tmpStr, 2); sprintf(tmpStr, "0b%08d", atoi(tmpStr)); Serial.print(tmpStr);
	}
	Serial.print("\r\n");
}
void printAllRegisters(int mode)
{
	for (byte reg = 0x00; reg <= REGISTERS; reg++)
	{
		printRegister(reg, mode);
	}
}
void setupXCLK()
{
	pmc_enable_periph_clk(PWM_INTERFACE_ID);
	PWMC_ConfigureClocks(XCLK_FREQ * 2, 0, VARIANT_MCK); //freq * period
	PIO_Configure(
		g_APinDescription[XCLK_PIN].pPort,
		g_APinDescription[XCLK_PIN].ulPinType,
		g_APinDescription[XCLK_PIN].ulPin,
		g_APinDescription[XCLK_PIN].ulPinConfiguration);
	uint32_t channel = g_APinDescription[XCLK_PIN].ulPWMChannel;
	PWMC_ConfigureChannel(PWM_INTERFACE, channel, PWM_CMR_CPRE_CLKA, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, channel, 2);
	PWMC_SetDutyCycle(PWM_INTERFACE, channel, 1);
	PWMC_EnableChannel(PWM_INTERFACE, channel);
	//pmc_mck_set_prescaler(2);
}
void vsync_rising() //frame start
{
	onFrame = true;
}
void vsync_falling() //frame stop
{
	onFrame = false;
}
void href_rising() //row start
{
	onRow = true;
}
void href_falling() //row stop
{
	onRow = false;
}
void pclk_rising() //pixel start
{
	onPixel = true;
}
void pclk_falling() //pixel stop
{
	onPixel = false;
}
uint16_t buf[WIDTH];
uint16_t color565 = 0;
byte byteRRRRRGGG, byteGGGBBBBB;

#pragma endregion
#pragma region Pixel Processing functions

int redThreshold = 25;


uint16_t processPixelRgb565(uint16_t pxl, byte threshold) {

	//	rrrrrggggggbbbbb
	// valore massimo per canale rosso e blu =31

#define rrrrr  (pxl >> 11)
#define gggggg  ((pxl & 0b0000011111100000 )>>5)
#define bbbbb  (pxl & 0b0000000000011111 )
	if ((rrrrr > threshold)&(gggggg < 10)&(bbbbb < 10))
	{
		pxl = 0b11111; //
					   //color565 = ((byteRRRRRGGG >>5) << 11 ) ;// solo valori alti del canale rosso
	}
	if (pxl == 0xFFFF)
	{
		AECsatPixelCounter++;
		pxl = 0b0000011111100000; // Marca i pixel bruciati col verde
	}
	return pxl;
}

void regolaAECGC(uint16_t maxBurnedPixelCount) {
	if (AECsatPixelCounter > maxBurnedPixelCount)
	{
		//riduci guadagno AEC 
		SetCameraAECval(AECcurrentValue >> 2);
		SetCameraAGCval(AGCcurrentValue >> 2);
		//resetto il conteggio
		AECsatPixelCounter = 0;
	}
}

#pragma endregion

void setup()
{
	Serial.begin(115200);

	// 	while (!Serial);
	//	while (!SerialUSB);

#pragma region ILI9341_due tft  setup

	tft.begin(); //include spi.begin
	tft.fillScreen(ILI9341_BLACK);

	tft.setRotation(iliRotation0); //iliRotation0: TFT VERTICALE (CONNETTORE TFT IN BASSO) , X da sin a DX (lato corto), Y da Alto vs basso (lato lungo) 
								   //		-- iliRotation0 -
								   //		| y		x->		|
								   //		| |				|
								   //		| V				|
								   //		|				|
								   //		|				|
								   //		|				|
								   //		|				|
								   //		|				|
								   //		-----------------

								   ////	tft.setAddrWindowRect(0, 0, 320, 240);
								   //for (int y =0 ; y < HEIGHT; y++) { // 3 lines
								   //	for (int x = 0; x <WIDTH ; x++) { // 3 lines
								   //		tft.drawPixel(x,y, rgb888to565(x,0,(y>>2))); // draw the generated colors
								   //	}
								   //}

#if 0
#pragma region test pushColors
	tft.setAddrWindowRect(0, 0, 320, 240); // set the drawing window to a 200x3px rectangle
	for (byte x = 0; x < 240; x++) { // generate a simple grayscale gradient

		buf[x] = tft.color565(x, 0, 0);
	}
	for (byte y = 0; y < 300; y++) { // 3 lines
		tft.pushColors(buf, 0, 240); // draw the generated colors
	}

#pragma endregion  
#endif // 0

	tft.setFont(Arial_bold_14);
	tft.setTextLetterSpacing(5);
	tft.setTextColor(ILI9341_RED, ILI9341_WHITE);
	tft.printAligned(F("OV7670_DUE.ino"), gTextAlignMiddleCenter);

#if 1

#endif // 0

#pragma endregion


#pragma region OV7670setup
	Wire.begin();

#if 0
	//	setupXCLK();
	//Wire.setClock(400000); //should work but needs some delay after every read/write. buggy?

#pragma region 10.5MHz clock
	int32_t mask_PWM_pin = digitalPinToBitMask(7);
	REG_PMC_PCER1 = 1 << 4;				// activate clock for PWM controller
	REG_PIOC_PDR |= mask_PWM_pin;		// activate peripheral functions for pin (disables all PIO functionality)
	REG_PIOC_ABSR |= mask_PWM_pin;	// choose peripheral option B    
	REG_PWM_CLK = 0;					// choose clock rate, 0 -> full MCLK as reference 84MHz
	REG_PWM_CMR6 = 0 << 9;				// select clock and polarity for PWM channel (pin7) -> (CPOL = 0)
	REG_PWM_CPRD6 = 8;                // initialize PWM period -> T = value/84MHz (value: up to 16bit), value=8 -> 10.5MHz
	REG_PWM_CDTY6 = 4;                // initialize duty cycle, REG_PWM_CPRD6 / value = duty cycle, for 8/4 = 50%
	REG_PWM_ENA = 1 << 6;               // enable PWM on PWM channel (pin 7 = PWML6)
#pragma endregion
#else
	setupXCLK();
#endif // 1


	camInit();
	setColorSpace(MYCOLORSPACE);
	setRes(QVGA);//setRes(VGA); //ORIG
				 //wrReg(REG_CLKRC, 0x40); // 40 = 0100 0000 = USA clk esterno ( prescaler n.a.)

				 // con setupXCLK quest valore deve essere 40 altrimenti non funziona
				 //wrReg(0x6B, 0b01111010);	//XCLOCK UPSCALER  XCLOCK x 4
	wrReg(0x11, 40); //40 ok , slow divider because of slow serial limit

					 // mie personalizzazioni ---------------------------
	wrReg(REG_COM8, 0); //AEC OFF, AGC OFF, AWB OFF

#pragma region COMMENTED
						//wrReg(0x12, 0x02); //ColorBar semitransparent overlay of the image
						//wrReg(0x42, 0x08); //ColorBar (DSP color bars at COM17)
						//wrReg(0x15, 0x02); //VSYNC inverted
						//wrReg(0x11, 0x82); //Prescaler x3 (10 fps)

						//wrReg(0x11, 1 << 6);

						//F(internal clock) = F(input clock) / (Bit[0 - 5] + 1)
						/*uint8_t tmpReg = 0;
						tmpReg |= 1 << 0;
						tmpReg |= 1 << 1;
						tmpReg |= 1 << 2;*/
						//tmpReg |= 1 << 3;
						//tmpReg |= 1 << 4;
						//tmpReg |= 1 << 5;
						//wrReg(0x11, tmpReg);
						//wrReg(0x11, 20);

						// default value gives 5.25MHz pixclock
						// wrReg(0x11, 20); //slow divider because of slow serial limit 125kHz
						// wrReg(0x11, 10); //slow divider because of slow serial limit 238kHz
						// wrReg(OV_CLKRC, 30); //slow divider because of slow serial limit 84kHz
						// wrReg(0x11, 60); //slow divider because of slow serial limit 43kHz

						//code to read registers and check if they were written ok
						/*printRegister(0x01, 1);
						printRegister(0x12, 1);
						printRegister(0x15, 1);
						printRegister(0x11, 1);
						Serial.print(F("\n"));
						printAllRegisters(1);*/
#pragma endregion



	pinMode(D0_PIN, INPUT);
	pinMode(D1_PIN, INPUT);
	pinMode(D2_PIN, INPUT);
	pinMode(D3_PIN, INPUT);
	pinMode(D4_PIN, INPUT);
	pinMode(D5_PIN, INPUT);
	pinMode(D6_PIN, INPUT);
	pinMode(D7_PIN, INPUT);

	attachInterrupt(VSYNC_PIN, vsync_rising, RISING);		//onFrame = true;
	attachInterrupt(HREF_PIN, href_rising, RISING);		//onRow = true;

#pragma endregion

}
uint8_t r, g, b;
uint8_t U0, Y0, V0, Y1;
uint8_t U2, Y2, V2, Y3;

byte threshold = 20; // VALORE MINIMO Del canale rosso
rgb px;


void acquireAndDisplay_BASE() {
	//FUNZIONA CON I SEGUENTI SETTAGGI
	//#define XCLK_FREQ 10 * 1000000 //10.5 Mhz slow but ok.
	//#define MYCOLORSPACE	RGB565	


	//Serial.print(F("*FRAME_START..."));
	onFrame = false;
	while (!onFrame);
	toggleLed;
	// camera onframe = 320 linee
	// la camera fornisce da DX a SX 240 linee verticali di 320 pixel 
	for (int x = WIDTH; x >0; x--) //loop su lato corto partendo da Destra verso SX
	{

		onRow = false;
		while (!onRow); //not working w/interrupt on href
		for (int y = 0; y< HEIGHT; y++)//loop su lato lungo del TFT
		{
			switch (MYCOLORSPACE) {
			case YUV422:
				//YUV422 deve leggere 4 byte per due pixel  vedi http://embeddedprogrammer.blogspot.it/2012/07/hacking-ov7670-camera-module-sccb-cheat.html
				//N					Byte
				//1st					Cb0
				//2nd					Y0
				//3rd					Cr0
				//4th					Y1
				//5th					Cb2
				//6th					Y2
				//7th					Cr2
				//8th					Y3
				//Pixel 0	Y0 Cb0 Cr0
				//Pixel 1	Y1 Cb0 Cr0
				//Pixel 2	Y2 Cb2 Cr2
				//Pixel 3	Y3 Cb2 Cr2
				//.....in generale
				//Pixel i	=	Y(i) Cb(i) Cr(i)
				//Pixel i+1 =	Y(i+1) Cb(i) Cr(i)
				//if (HREF_BIT)
				//{
				while (PCLK_BIT);  U0 = READCAMERADATA; while (!PCLK_BIT);
				while (PCLK_BIT);  Y0 = READCAMERADATA;	 while (!PCLK_BIT);
				while (PCLK_BIT);  V0 = READCAMERADATA; while (!PCLK_BIT);
				while (PCLK_BIT);  Y1 = READCAMERADATA;	 while (!PCLK_BIT);
				tft.drawPixel(x, y, hsv2rgb565(Y0, U0, V0));
				tft.drawPixel(x, y + 1, hsv2rgb565(Y1, U0, V0));

				//if (y % 2 == 0) {

				//}
				//else {

				//}


				y++; // incrementa il contatore
				if (y == HEIGHT - 4) { y = HEIGHT; } //esce dal loop
													 //}
				break;
			case RGB565: //funziona 

						 // LETTURA DEL PRIMO BYTE ------------------
				while (PCLK_BIT); //wait for low
				byteRRRRRGGG = READCAMERADATA;
				while (!PCLK_BIT); //wait for high

								   // LETTURA DEL SECONDO BYTE----------------
				while (PCLK_BIT); //wait for low
				byteGGGBBBBB = READCAMERADATA;
				while (!PCLK_BIT); //wait for high

				color565 = ((byteRRRRRGGG << 8) | (byteGGGBBBBB));// 
																  //r = byteRRRRRGGG & 0b11111000;
																  //g = ((byteRRRRRGGG & 0b00000111) << 5) | (byteGGGBBBBB >> 5);
																  //b = (byteGGGBBBBB << 3);
				tft.drawPixel(x, y, color565);
				//tft.drawPixel(i, j, tft.color565(r, g, b));

				//color565 = ((byteRRRRRGGG >>3) << 11 ) ;// solo canale rosso
				//if ((byteRRRRRGGG >> 3) > threshold)
				//{
				//	color565 = ((byteRRRRRGGG >> 3) << 11);// solo canale rosso
				//										   //color565 = ((byteRRRRRGGG >>5) << 11 ) ;// solo valori alti del canale rosso 
				//	tft.drawPixel(x, y, color565);
				//}
				//else
				//{
				//	tft.drawPixel(i, j, 0);

				//}	
				//}

				break;
			case RGB888:
				if (HREF_BIT)
				{

					while (PCLK_BIT);  px.r = READCAMERADATA; while (!PCLK_BIT);
					while (PCLK_BIT);  px.g = READCAMERADATA; while (!PCLK_BIT);
					while (PCLK_BIT);  px.b = READCAMERADATA; while (!PCLK_BIT);
					tft.drawPixel(x, y, rgb888to565(px.r, px.g, px.b));
				}


				break;

			case BAYER_RGB:
				if (HREF_BIT)
				{

					while (PCLK_BIT);  px.r = READCAMERADATA; while (!PCLK_BIT);
					//while (PCLK_BIT);  px.g = READCAMERADATA; while (!PCLK_BIT);
					//while (PCLK_BIT);  px.b = READCAMERADATA; while (!PCLK_BIT);
					tft.drawPixel(x, y, rgb888to565((((byte)px.r >> 4) << 4), px.g, px.b));
				}

				break;




			}
		}
	}



}
// ok funziona in modalità RGB565 
void acquireAndDisplayRGB565() {
	if (Serial.available())
	{
		int in = Serial.parseInt();
		if (in> 0)
		{
			redThreshold = in;
		}

	}
	regolaAECGC(1);

#define col2 80
	tft.printAt(String(redThreshold), 1, 2);
	tft.printAt(F(" soglia:"), col2, 2);

	tft.printAt(String(AECsatPixelCounter), 1, 20);
	tft.printAt(F(" pixel saturi:"), col2, 20);
	tft.printAt(String(AECcurrentValue), 1, 36);
	tft.printAt(F(" AEC"), col2, 36);
	tft.printAt(String(AGCcurrentValue), 1, 36);
	tft.printAt(F(" AGC "), col2, 36);


	//Serial.print(F("*FRAME_START..."));
	onFrame = false;
	while (!onFrame);
	toggleLed;
	// camera onframe = 320 linee
	// la camera fornisce da DX a SX 240 linee verticali di 320 pixel 
	for (int x = WIDTH; x >0; x--) //loop su lato corto partendo da Destra verso SX
	{

		onRow = false;
		while (!onRow); //not working w/interrupt on href
		for (int y = 0; y< HEIGHT; y++)//loop su lato lungo del TFT
		{

			// LETTURA DEL PRIMO BYTE ------------------
			while (PCLK_BIT); //wait for low
			byteRRRRRGGG = READCAMERADATA;
			while (!PCLK_BIT); //wait for high

							   // LETTURA DEL SECONDO BYTE----------------
			while (PCLK_BIT); //wait for low
			byteGGGBBBBB = READCAMERADATA;
			while (!PCLK_BIT); //wait for high

			color565 = ((byteRRRRRGGG << 8) | (byteGGGBBBBB));// 
															  //r = byteRRRRRGGG & 0b11111000;
															  //g = ((byteRRRRRGGG & 0b00000111) << 5) | (byteGGGBBBBB >> 5);
															  //b = (byteGGGBBBBB << 3);
			tft.drawPixel(x, y, processPixelRgb565(color565, redThreshold));




		}
	}

}
void loop() {
	//loop_ref();
	acquireAndDisplayRGB565();
}




/*
void parse_cmd()
{
new_send = false;

if (strcmp("snap", word) == 0)
{
CameraSnap();
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_bw_VGA", word) == 0)                    // Set up for 640*480 pixels RAW
{
format = 'b';
resolution = VGA;
if (camera.Init('b', VGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_yuv_QVGA", word) == 0)                  // Set up for 320*240 pixels YUV422
{
format = 'y';
resolution = QVGA;
if (camera.Init('b', QVGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_rgb_QVGA", word) == 0)                  // Set up for 320*240 pixels RGB565
{
format = 'r';
resolution = QVGA;
if (camera.Init('r', QVGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_bw_QVGA", word) == 0)                  // Set up for 320*240 pixels YUV (Only Y)
{
format = 'b';
resolution = QVGA;
if (camera.Init('b', QVGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_yuv_QQVGA", word) == 0)                 // Set up for 160*120 pixels YUV422
{
format = 'y';
resolution = QQVGA;
if (camera.Init('b', QQVGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_rgb_QQVGA", word) == 0)                 // Set up for 160*120 pixels RGB565
{
format = 'r';
resolution = QQVGA;
if (camera.Init('r', QQVGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("init_bw_QQVGA", word) == 0)                 // Set up for 160*120 pixels YUV (Only Y)
{
format = 'b';
resolution = QQVGA;
if (camera.Init('b', QQVGA) != 1)
{
pc.printf("Init Fail\r\n");
}
pc.printf("Initializing done\r\n");
memset(word, 0, sizeof(word));
}
else
if (strcmp("reset", word) == 0)
{
mbed_reset();
}
else
if (strcmp("time", word) == 0)
{
pc.printf("Tot time acq + send (mbed): %dms\r\n", t2 - t1);
memset(word, 0, sizeof(word));
}
else
if (strcmp("reg_status", word) == 0)
{
int i = 0;
pc.printf("AD : +0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +A +B +C +D +E +F");
for (i = 0; i<OV7670_REGMAX; i++)
{
int data;
data = camera.ReadReg(i); // READ REG
if ((i & 0x0F) == 0)
{
pc.printf("\r\n%02X : ", i);
}
pc.printf("%02X ", data);
}
pc.printf("\r\n");
}

memset(word, 0, sizeof(word));

}
*/