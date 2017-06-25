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

#include "Arduino.h"
#include <string.h>
#pragma region debug

#define dbg(cha) 	 Serial.println(F(cha));
#define dbg2(t,cha)	 Serial.print(F(t));Serial.println(cha);  
#define dbgHex(b)	 Serial.print(b,HEX);Serial.print(" ");  
//#include <MemoryFree/MemoryFree.h>

#pragma endregion

#pragma region Laser Setup
	#define PIN_LINELASER 13
	#define LASER_ON 	digitalWrite(PIN_LINELASER, 1);
	#define LASER_OFF 	digitalWrite(PIN_LINELASER, 0);
	#define LASER_TOGGLE digitalWrite(PIN_LINELASER, !digitalRead(PIN_LINELASER));

#pragma endregion


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

#pragma region conversion and processing functions
int redThreshold;
int maxDiff = 0;

	inline int digitalReadDirect(int pin) {
		return !!(g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin);
	}
	#define CLR(x,y) (x&=(~(1<<y)))

	#define SET(x,y) (x|=(1<<y))
 	#define toggleLed digitalWrite(13,!digitalReadDirect(13))
	typedef struct {
		uint8_t r;       // percent
		uint8_t g;       // percent
		uint8_t b;       // percent
	} rgb;

	typedef struct {
		double h;       // angle in degrees
		double s;       // percent
		double v;       // percent
	} hsv;
	typedef struct {
		float h;       // angle in degrees
		float s;       // percent
		float v;       // percent
	} hsvF;

	static hsv rgb2hsv(rgb in);
	static rgb rgb565to888(uint16_t  in);
	static uint16_t rgb888to565(byte r, byte g, byte b);  
	static hsv rgb888toHSB(byte red, byte green, byte blue);
	static rgb hsv2rgb888(hsv in);
	static uint16_t hsv2rgb565(hsv in);
 
	static hsv rgb888toHSB(byte red, byte green, byte blue) {
		double r = red / 255.0f;
		double g = green / 255.0f;
		double b = blue / 255.0f;
		double max = max(max(r, g), b);
		double min = min(min(r, g), b);
		double delta = max - min;
		double hue = 0;
		double brightness = max;
		double saturation = max == 0 ? 0 : (max - min) / max;
		if (delta != 0) {
			if (r == max) {
				hue = (g - b) / delta;
			}
			else {
				if (g == max) {
					hue = 2 + (b - r) / delta;
				}
				else {
					hue = 4 + (r - g) / delta;
				}
			}
			hue *= 60;
			if (hue < 0) hue += 360;
		}
		hsv hsv1;
		hsv1.h = hue;
		hsv1.s = saturation;
		hsv1.v = brightness;

		return hsv1;


	}
	#define swapFloat(a, b) { float t = a; a = b; b = t; }

	static void RGB2H(uint8_t red, uint8_t green, uint8_t blue,	float &h){

		float r = red / 255.0f;
		float g = green / 255.0f;
		float b = blue / 255.0f;
		float max = fmaxf(fmaxf(r, g), b);
		float min = fminf(fminf(r, g), b);
		float delta = max - min;
		if (delta != 0)
		{
			float hue;
			if (r == max)
			{
				hue = (g - b) / delta;
			}
			else
			{
				if (g == max)
				{
					hue = 2 + (b - r) / delta;
				}
				else
				{
					hue = 4 + (r - g) / delta;
				}
			}
			hue *= 60;
			if (hue < 0) hue += 360;
			h = hue;
		}
		else
		{
			h = 0;
		}
		//s = max == 0 ? 0 : (max - min) / max;
		//b = max;
	}


	static void RGB2HSV_old(float r, float g, float b,	float &h, float &s, float &v)
	{
		// da http://lolengine.net/blog/2013/01/13/fast-rgb-to-hsv
		float K = 0.f;

		if (g < b)
		{
			swapFloat(g, b);
			K = -1.f;
		}

		if (r < g)
		{
			swapFloat(r, g);
			K = -2.f / 6.f - K;
		}

		float chroma = r - min(g, b);
		h = fabs(K + (g - b) / (6.f * chroma + 1e-20f));
		s = chroma / (r + 1e-20f);
		v = r;
	}
	hsv rgb2hsv(rgb in)
	{
		// da http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
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
		uint16_t rgb565=0;
		rgb565 =  ((r >> 3) << 11);
		rgb565 |= ((g >> 2) << 6);
		rgb565 |= (b >> 3);
		return rgb565; 
	}
	static rgb rgb565to888(uint16_t  in) {
		/* da http://stackoverflow.com/questions/2442576/how-does-one-convert-16-bit-rgb565-to-24-bit-rgb888
		R8 = ( R5 * 527 + 23 ) >> 6;
		G8 = ( G6 * 259 + 33 ) >> 6;
		B8 = ( B5 * 527 + 23 ) >> 6;
		*/
		rgb  out;
		uint16_t  ins = in;
		out.r = ((in) >> 3) * 255 / 31;
		out.g = (in >> 5 & 63) * 255 / 63;
		out.b = ((in) & 31) * 255 / 31;
		return out;
	}

	uint16_t hsv2rgb565(uint8_t y,uint8_t u, uint8_t v) { 
		double      hh, p, q, t, ff;
		long        i;
		uint8_t         r,g,b;

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


//////////////////////////////////////////////////////////////////////////
//	C L O C K  E S T E R N O											//
//////////////////////////////////////////////////////////////////////////
#define XCLK_FREQ 10 * 1000000 //10.5 Mhz slow but ok.					//
//#define XCLK_FREQ 3* 1000000 //21 Mhz doubles the image; too fast?	//
//#define XCLK_FREQ 24 * 1000000 //42 Mhz								//
//////////////////////////////////////////////////////////////////////////
#define MYCOLORSPACE	RGB565	
//#define MYCOLORSPACE	BAYER_RGB	
//#define MYCOLORSPACE	YUV422	
//#define MYCOLORSPACE	RGB888	

#if 0
#define MYRESOLUTION QVGA
#define WIDTH    120 //       ILI9341_TFTWIDTH
#define HEIGHT   160//       ILI9341_TFTHEIGHT
#else
//#define MYRESOLUTION VGA
//#define WIDTH    480 //       ILI9341_TFTWIDTH
//#define HEIGHT   640//       ILI9341_TFTHEIGHT
#define MYRESOLUTION QVGA
#define WIDTH    ILI9341_TFTWIDTH
#define HEIGHT   ILI9341_TFTHEIGHT
#endif // 0


#pragma region COLLEGAMENTI HW OV7670 usati in ov7670_DUE.ino
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

#pragma endregion




volatile boolean onFrame = false;
volatile boolean onRow = false;
volatile boolean onPixel = false;

 uint16_t AECsatPixelCounter = 0; // conta i pixel saturati
 rgb px;
 byte tolerance = 10;//soglia max per verde e blu per il riconoscimento del pixel come rosso
 uint16_t redPixelCnt = 0;

uint8_t OV7670_AEC = 0; // valore corrente dell'esposizione
uint8_t OV7670Gain = 0; // valore corrente del guadagno
uint16_t analogInput_A0 = 0;
uint16_t analogInput_A1 = 0;
void SetCameraAECval(uint16_t AECnewValue)
{
	if (AECnewValue > 0)
	{
		wrReg(REG_COM1, (AECnewValue && 0b0000000000000011)); //contiene AEC[1.0]
		wrReg(REG_AECH, (AECnewValue && 0b0000001111111100) >> 2); //contiene AEC[9..2]
		wrReg(REG_AECHH, AECnewValue >> 10); // AECHH[7.6] riservati ;in AECHH[5..0]  ci va AEC[15..10]  (parte alta)
	}


}
void SetCameraAGCval(uint16_t AGCnewValue)
{
	if (AGCnewValue > 0)
	{
		if (AGCnewValue>0)
		{
			OV7670Gain = OV7670Gain >> 2;
		}
		wrReg(REG_VREF, 3 + (OV7670Gain && 0b0000001100000000)); //VREF[7.6] contiene gain[9.8]
		wrReg(REG_GAIN, (OV7670Gain && 0b0000000011111111)); // GAIN[7.0]  contiene gain[7..0]
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

int dipSwitch[7];
#pragma endregion
#pragma region Pixel Processing functions

#define rrrrr(pxl)  (pxl >> 11)
#define gggggg(pxl)  ((pxl & 0b0000011111100000 )>>5)
#define bbbbb(pxl)  (pxl & 0b0000000000011111 )

void processPixelRgb565_base(int x, int y,uint16_t pxlIn) {
	uint16_t pxlOut;
	//	rrrrrggggggbbbbb
	// valore massimo per canale rosso e blu =31


	tolerance = map(analogInput_A0, 0, 1023, 0, 31);
	redThreshold = map(analogInput_A1, 0, 1023, 0, 0b11111);

	if (pxlIn == 0xFFFF)// Pixel saturo?
	{
		AECsatPixelCounter++;
		pxlOut = 0b0000011111100000; // marcalo in verde
	}else if ((rrrrr(pxlIn) > redThreshold) && (gggggg(pxlIn) < tolerance) && (bbbbb(pxlIn) < tolerance))
	{
		pxlOut = 0b11111; // marco in blu i pixel rossi
		//color565 = ((byteRRRRRGGG >>5) << 11 ) ;// solo valori alti del canale rosso
		redPixelCnt++;
		if (redPixelCnt>100) //allora riduco il guadagno
		{

		}
	}
	else
	{
		pxlOut = 0;
	}
	 
	if (dipSwitch[1] ) { //bypass processing
		tft.drawPixel(x,y,pxlIn);
	}
	else
	{
		tft.drawPixel(x,y, pxlOut);

	}

 
}

uint16_t processPixelRgb565_hsv(uint16_t rgb565, byte threshold, byte tolerance) {

	//	rrrrrggggggbbbbb
	// valore massimo per canale rosso e blu =31


	rgb p;
	//p = rgb565to888(rgb565);
	p.r = map(rrrrr(rgb565), 0,0b11111, 0,255);
	p.g = map(gggggg(rgb565),0,0b111111,0,255);
	p.b = map(bbbbb(rgb565), 0,0b11111, 0,255);
	hsvF hsvf;
	RGB2H(p.r,p.g,p.b,hsvf.h);
	// rosso in HSV = 0,1,1 //http://codebeautify.org/rgb-to-hsv-converter
	// test rosso in HSV  H< 0.1, S>0.6 V > 0.7


	//YCbCr(601) da "R'G'B' digitale a 8 bit "  fonte https://it.wikipedia.org/wiki/YCbCr
	//uint16_t Y = 16 + 1 / 256 * (65.738  * p.r + 129.057  * p.g + 25.064  * p.b);
	//uint16_t Cb = 128 + 1 / 256 * (-37.945  * p.r - 74.494  * p.g + 112.439  * p.b);
	//uint16_t Cr = 128 + 1 / 256 * (112.439  * p.r - 94.154  * p.g - 18.285  * p.b);

	//if (rgb565 == 0xFFFF)
	//{
	//	AECsatPixelCounter++;
	//	rgb565 = 0b0000011111100000; // Marca i pixel bruciati col verde
	//}else

	if (hsvf.h < 0.10) 
	{
		rgb565 = 0b11111<<11; // faccio il pixel rosso
					   
	}
	else
	{
		rgb565 = 0;
	}
 

	return rgb565;// hsv1.h;// ;// rgb888to565(p.r, p.g, p.b);
}

void regolaAECGC(uint16_t maxBurnedPixelCount) {
	if (AECsatPixelCounter > maxBurnedPixelCount)
	{
		//riduci guadagno AEC 
		SetCameraAECval(OV7670_AEC>>2);
		SetCameraAGCval(OV7670Gain>>2);
		//resetto il conteggio
		AECsatPixelCounter = 0;
	}
}

#pragma endregion




#pragma region Variabili globali per l'elaborazione dell'immagine
	uint8_t r, g, b;
	uint8_t U0, Y0, V0, Y1;
	uint8_t U2, Y2, V2, Y3;
	char key = 0;
	//Mtrice dell'immagine memorizzata solo 8 bit per limiti della memoria
	// memorizzo solo il canale rosso
static	uint8_t img[WIDTH][HEIGHT];

	// cursore
	# define CURSORCOLOR ILI9341_CYAN
	#define HALFCURSORSIZE 20
	int cursX = 138;		// WIDTH >>1;
	int cursY = 152;		// HEIGHT >>1;     
	// colore del pixel in corrispondenza del cursore
	int colorAtCursorR = 0;
	int colorAtCursorG = 0;
	int colorAtCursorB = 0;

#pragma endregion

void setup()
{
	Serial.begin(115200);
	SPI.begin();

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

#pragma region Laser Setup
	pinMode(PIN_LINELASER, OUTPUT);

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

	readSensors();
	camInit();//Reset the camera to default values


	setColorSpace(MYCOLORSPACE);
	setRes(MYRESOLUTION); 


	// con setupXCLK quest valore deve essere 40 altrimenti non funziona
	//wrReg(0x6B, 0b01111010);	//REG_DBLV XCLOCK UPSCALER  XCLOCK x 4
	wrReg(REG_CLKRC, 0b00101000); //wrReg(REG_CLKRC, 0b00101000); 40  Bit[6]=0: not bypass divider, Bit[5..0] internal clock prescaler vabene 0 per QVGA RGB565
//wrRegBit(REG_CLKRC, 6, true);//DON'T USE PRESCALER
//	wrReg(REG_CLKRC, 0x80);
//	wrRegBit(0x6b, 6,true);//PLL Multiplier x 8
//	wrRegBit(0x6b, 7,true); 
	//	wrReg(REG_CLKRC, 60);

	//wrReg(0x11, 0x80);
	//wrReg(0x6b, 0x0a);
	//wrReg(0x2a, 0x00);
	//wrReg(0x2b, 0x00);
	//wrReg(0x92, 0x66);
	//wrReg(0x93, 0x00);
	//wrReg(0x3b, 0x0a);



	// COLOR BAR
//	wrRegBit(REG_COM17, 3, true);// ENABLE COLORBAR 
	//wrRegBit(REG_COM7, 1, false);// COLORBAR TRANSP.



	// mie personalizzazioni ---------------------------
	wrRegBit(REG_COM8,0, dipSwitch[1]); //AEC Enable
	wrRegBit(REG_COM8,1, dipSwitch[2]); //AGC Enable
 	wrRegBit(REG_COM8,7, dipSwitch[3]); //Enable fast AGC/AEC algorithm

 	//SetCameraAECval(0);
	//SetCameraAGCval(10);

	//wrReg(REG_VSTART,45); //WINDOW START ON VERTICAL
	//wrReg(REG_VSTOP,50); //WINDOW START ON VERTICAL


#pragma region COMMENTED

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

		// collegamento al dip switch
	pinMode(22, INPUT_PULLUP);
	pinMode(23, INPUT_PULLUP);
	pinMode(24, INPUT_PULLUP);
	pinMode(25, INPUT_PULLUP);
	pinMode(26, INPUT_PULLUP);
	pinMode(27, INPUT_PULLUP);

#pragma region OV7670 HW INTERFACE & INTERUPT SETUP

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
	pinMode(13, OUTPUT);

#pragma endregion

#pragma endregion


#if 1
#pragma region prove sperimentali

//#define XCLK_FREQ 24 * 1000000 //42 Mhz								//

	setColorSpace(BAYER_RGB); //RGB888 YUV422 RGB565
	setRes(VGA); //QVGA

	while (true) {
		acquireAndDisplay_BASE();
	}


#pragma endregion

#endif // 1

	for (int i = 0; i < 3; i++)
	{
		testFillSpeed(i<<7);
	}

	for (int i = 0; i < 3; i++)
	{
		tft.printAt(F("TEST (3pass)"), 50, 120);

		acquireAndDisplay_BASE();
		readSensors();
		displayCursor();
	}
	tft.fillScreen(ILI9341_BLACK);
	tft.printAt(F("STARTING..."), 50, 120);

} //end setup

void processSerialCommand() {
	String inputstring;
#define NUMVARS 2
	String strVars[NUMVARS] = { "cursX","cursY" };// struttura comando: var=valore
	if (Serial.available())
	{
		inputstring = Serial.readString();
		int p = inputstring.indexOf("=");  //posizione del carattere "="
		if (p>0)
		{
			//presente carattere "="
			String variable = inputstring.substring(1, p);
			int val = (int)inputstring.substring(p + 1).toInt();
			if (variable.equals("cursX")) { cursX = val; Serial.print("cursX="); Serial.println(cursX);
			}
			else if (variable.equals("cursY")) { cursY = val; Serial.print("cursY="); Serial.println(cursY);
			}
			else if (variable.equals("cursDX")) { cursX += val; Serial.print("cursX="); Serial.println(cursX);
			}
			else if (variable.equals("cursDY")) { cursY += val; Serial.print("cursY="); Serial.println(cursY);
			}
		}
	}
}

void processSerialCommand_0() {
	while (Serial.available())
	{
		//int serialIn = Serial.parseInt();
		//if (serialIn> 0)
		//{
		//	tolerance = serialIn;
		//}
		key = Serial.read();   //gets one byte from serial buffer
		if (key =='l')
		{ // left byte(37)
			if (cursX > 0) {
				//disegno la nuova posizione del cursore
				// cancello il precedente
				// TBD

				// aggiorno la posizione
				cursX--;
				// disegno la nuova posizione


			}else
			{ cursX = 0; }
		}
		else if (key =='r')// right byte(39)
		{ 
			cursX++;
			if (cursX > WIDTH-1) { cursY = WIDTH-1; }

		}
		else if (key =='u' )
		{ // up byte(38)
			cursY--;
			if (cursY < 0) { cursY = 0; }


		}
		else if (key =='d')
		{ // down byte(40)
			cursY++;
			if (cursY > HEIGHT-1) { cursY = HEIGHT-1; }


		}
		else if (key =='c')
		{
		}
		//else
		//{

		//}
	}

}
void readSensors() {
	analogInput_A0 = analogRead(A0);
	analogInput_A1 = analogRead(A1);
	tolerance =map(analogInput_A0, 0, 1023, 0, 31);
	redThreshold = analogRead(A1) >> 5;
//	redThreshold=(int)map(analogInput_A1, 0, 1023, 0, 31);
	for (int i = 1; i < 7; i++)
	{
		dipSwitch[i] =digitalReadDirect(28-i);

	}
 
			////leggo i valori del dip switch
			//uint8_t in = digitalReadDirect(22) | digitalReadDirect(23) << 1 | digitalReadDirect(24) << 2 | digitalReadDirect(25) << 3 | digitalReadDirect(26) << 4 | digitalReadDirect(26) << 5 | digitalReadDirect(27) << 6;
			//in = ~in;
			////leggo il valore corrente de registro e imposto i nuovi valori
			//uint8_t newRegVal = (rdReg(REG_CLKRC) && 0b11000000)| in;

			//wrReg(REG_CLKRC, newRegVal);
			//Serial.print(newRegVal);


	wrRegBit(REG_COM8, 0, dipSwitch[2]); //Bit[0]: AEC Enable
	wrRegBit(REG_COM8, 1, dipSwitch[3]); //Bit[1]: AWB Enable
	wrRegBit(REG_COM8, 2, dipSwitch[4]); //Bit[2]: AGC Enable
	wrRegBit(REG_COM8, 7, dipSwitch[5]); //Enable fast AGC/AEC algorithm
	OV7670Gain = rdReg(REG_GAIN); //AGC – Gain control gain setting
	wrReg(REG_VREF, 3 + (OV7670Gain && 0b0000001100000000)); //VREF[7.6] contiene gain[9.8]
	wrReg(REG_GAIN, (OV7670Gain && 0b0000000011111111)); // GAIN[7.0]  contiene gain[7..0]

}
void displayStatus(){
	#define col2 60
	int row = 2;
	tft.printAt(String(redPixelCnt), 1, row);
	tft.printAt(F("redPixels"), col2, row);
	row = 20;
	tft.printAt(String(redThreshold),1, row );
	tft.printAt(F("soglia rosso"), col2, row);

	//row = 36;
	//tft.printAt(String(tolerance), 1, row);
	//tft.printAt(F("tolerance G ,B"), col2, row);

	//tft.printAt(String(AECcurrentValue), 1, 36);
	//tft.printAt(F(" AEC"), col2, 36);
	//tft.printAt(String(AGCcurrentValue), 1, 36);
	//tft.printAt(F(" AGC "), col2, 36);


	row = 48;
	tft.printAt(String(AECsatPixelCounter), 1, row);
	tft.printAt(F("pixel saturi:"), col2, row);

	row = 60;
	tft.printAt(String(OV7670Gain), 1, row);
	tft.printAt(F("AGC "), col2,row );

	row = 100;
	tft.printAt(String(maxDiff), 1, row);
	tft.printAt(F("maxDiff"), col2, row);
	//row = 100;
	//tft.printAt(String(colorAtCursorR), 1, row);
	//tft.printAt(F("curs R"), col2, row);
	//tft.printAt(String(colorAtCursorG), 1, row+20);
	//tft.printAt(F("curs G"), col2, row + 20);
	//tft.printAt(String(colorAtCursorB), 1, row+40);
	//tft.printAt(F("curs B"), col2, row + 40);

	row = 180;
	tft.printAt(String(analogInput_A0), 1, row);
	tft.printAt(F("A0"), col2, row);
	tft.printAt(String(analogInput_A1), 1, row+20);
	tft.printAt(F("A1"), col2, row+20);

	// lo stato dei diP Switch in orizzontale 
	row = 220;
	for (size_t i = 1; i < 7; i++)
	{
		if (dipSwitch[i]==0)
		{
			tft.printAt("0", 1+15*(i-1), row);

		}
		else
		{
			tft.printAt("1", 1+15*(i-1), row);

		}
	}


}
void resetValues() {
	// Resetta i valori al termine di un frame
	maxDiff = 0;
	redPixelCnt = 0;
	AECsatPixelCounter = 0;
}
//non fa elaborazione ma "processa anche YUV e RGB888
void acquireAndDisplay_BASE() { 
	//FUNZIONA CON I SEGUENTI SETTAGGI
	//#define XCLK_FREQ 10 * 1000000 //10.5 Mhz slow but ok.
	//#define MYCOLORSPACE	RGB565	


	//Serial.print(F("*FRAME_START..."));
		onFrame = false;
		while (!onFrame);
		//toggleLed;
		// camera onframe = 320 linee
		// la camera fornisce da DX a SX 240 linee verticali di 320 pixel 
		for (int x =WIDTH ; x >0 ; x--) //loop su lato corto partendo da Destra verso SX
 		{

			onRow = false;
			while (!onRow); //not working w/interrupt on href
			for (int y = 0 ; y< HEIGHT; y++)//loop su lato lungo del TFT
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
							while (PCLK_BIT);  Y0 = READCAMERADATA;	 while (!PCLK_BIT);
							while (PCLK_BIT);  U0 = READCAMERADATA; while (!PCLK_BIT);
							while (PCLK_BIT);  Y1 = READCAMERADATA;	 while (!PCLK_BIT);
							while (PCLK_BIT);  V0 = READCAMERADATA; while (!PCLK_BIT);
								//tft.drawPixel(x, y, hsv2rgb565(Y0,U0,V0));
								//tft.drawPixel(x, y+1, hsv2rgb565(Y1, U0, V0));
								tft.drawPixel(x, y, Y0);
								tft.drawPixel(x, y+1, Y1);
			
							//if (y % 2 == 0) {

							//}
							//else {
 
							//}


							y++; // incrementa il contatore
							if (y+1 == HEIGHT  ) { y = HEIGHT; } //esce dal loop
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
						//tft.drawPixel(x, y, rgb888to565(px.r,px.g,px.b));
						tft.drawPixel(x, y,   px.r >>3);
						}


						break;

					case BAYER_RGB:
						if (HREF_BIT)
						{

						while (PCLK_BIT);  px.r = (uint8_t)READCAMERADATA; while (!PCLK_BIT);
						//while (PCLK_BIT);  px.g = READCAMERADATA; while (!PCLK_BIT);
						//while (PCLK_BIT);  px.b = READCAMERADATA; while (!PCLK_BIT);
						tft.drawPixel(x,y,  px.r>>7 );
						}

						break;




				}
			}
		}



	}
// ok funziona in modalità RGB565 
void acquireRGB565AndProcess() {
	
	//regolaAECGC(1);


	//Serial.print(F("*FRAME_START..."));
	AECsatPixelCounter = 0;
	redPixelCnt = 0;
	onFrame = false;
	while (!onFrame);
	//toggleLed;
	// camera onframe = 320 linee
	// la camera fornisce da DX a SX 240 linee verticali di 320 pixel 
	for (int x = WIDTH; x >0; x--) //loop su lato corto partendo da Destra verso SX
	{

		onRow = false;
		while (!onRow); //not working w/interrupt on href
		for (int y = 0; y< HEIGHT; y++)//loop su lato lungo del TFT
		{

			// LETTURA DEL PRIMO BYTE ------------------
			while (PCLK_BIT);	byteRRRRRGGG = READCAMERADATA;	while (!PCLK_BIT);  

			// LETTURA DEL SECONDO BYTE----------------
			while (PCLK_BIT);	byteGGGBBBBB = READCAMERADATA;	while (!PCLK_BIT);

			color565 = ((byteRRRRRGGG << 8) | (byteGGGBBBBB));// 
																//r = byteRRRRRGGG & 0b11111000;
																//g = ((byteRRRRRGGG & 0b00000111) << 5) | (byteGGGBBBBB >> 5);
																//b = (byteGGGBBBBB << 3);
			//tft.drawPixel(x, y, processPixelRgb565(color565, redThreshold,5));
 
			//if ((x == cursX) && (y == cursY)) {
			//	Serial.println(colorAtCursorR);
			//	 colorAtCursorR = rrrrr(color565);
			//	 colorAtCursorG = gggggg(color565);
			//	 colorAtCursorB = bbbbb(color565);
			//	Serial.println(colorAtCursorR);

			//}
			processPxl(x, y, byteRRRRRGGG>>3);
			//processPixelRgb565_base(x, y,color565);
			//tft.drawPixel(x, y,  color565 );

			//tft.drawPixel(x, y, processPixelRgb565_ref(color565, redThreshold,tolerance));
			//tft.drawPixel(x, y, color565);
			//delay(50);

			
		}
	}

}
void acquireRGB565AndDisplay() {
	
	//regolaAECGC(1);


	//Serial.print(F("*FRAME_START..."));
	AECsatPixelCounter = 0;
	redPixelCnt = 0;
	onFrame = false;
	while (!onFrame);
//	toggleLed;
	// camera onframe = 320 linee
	// la camera fornisce da DX a SX 240 linee verticali di 320 pixel 
	for (int x = WIDTH; x >0; x--) //loop su lato corto partendo da Destra verso SX
	{

		onRow = false;
		while (!onRow); //not working w/interrupt on href
		for (int y = 0; y< HEIGHT; y++)//loop su lato lungo del TFT
		{

			// LETTURA DEL PRIMO BYTE ------------------
			while (PCLK_BIT);	byteRRRRRGGG = READCAMERADATA;	while (!PCLK_BIT);  

			// LETTURA DEL SECONDO BYTE----------------
			while (PCLK_BIT);	byteGGGBBBBB = READCAMERADATA;	while (!PCLK_BIT);

 			color565 = ((byteRRRRRGGG << 8) | (byteGGGBBBBB));// 
																//r = byteRRRRRGGG & 0b11111000;
																//g = ((byteRRRRRGGG & 0b00000111) << 5) | (byteGGGBBBBB >> 5);
																//b = (byteGGGBBBBB << 3);
			// memorizzo il canale rosso
			img[x][y]= byteRRRRRGGG>>3;
			if (color565==0xFFFF)//saturo ?
			{
				tft.drawPixel(x, y, 0b0000011111100000);// pixel verde
				AECsatPixelCounter++;
			}
			else
			{
				tft.drawPixel(x, y, color565);

			}

			
		}
	}

}
void displayCursor() {

	tft.drawFastVLine(cursX, cursY- HALFCURSORSIZE, HALFCURSORSIZE<<1, CURSORCOLOR);// linea lungo asse y
	tft.drawFastHLine(cursX- HALFCURSORSIZE, cursY, HALFCURSORSIZE << 1, CURSORCOLOR);
}
void testFillSpeed(uint16_t col) {
 	for (int x = WIDTH; x >0; x--) //loop su lato corto partendo da Destra verso SX
	{

 		for (int y = 0; y< HEIGHT; y++)//loop su lato lungo del TFT
		{

			tft.drawPixel(x, y, col);
 

			//tft.drawPixel(x, y, processPixelRgb565_ref(color565, redThreshold,tolerance));
 
			
		}
	}

}
// converte tftX in alfa e tftY in distanza (cm) e li mette in un array
void convertToDistance(int tftX, int tftY, int diff) {
	//	const int lensAngle = 56;
	//	int alfa =  (160-tftX )*lensAngle/320;
	//	dbg2("alfa", alfa)
	//	int dist = tftY;
}
void processPxl(int x, int y, uint8_t red5bitLaserOn) {
	int diff;
	//faccio la differenza tra valore corrente dell'immagine con Laser acceso e
	// valore precedente su matrice ( laser spento)
	diff = red5bitLaserOn - img[x][y];


	//clip su valori positivi o nulli
	if (diff < 0) { diff = 0; }

	// registro la differenza massima
	if (diff > maxDiff) {maxDiff = diff;}

	//tft.drawFastHLine(x, y,-diff, ILI9341_AZURE);
	//redThreshold = analogRead(A1) >> 5;
	if (diff > redThreshold) //se la differenza supera la soglia il pixel è riconducibile al laser
	{
		tft.drawPixel(x, y, 0b1111100000000000);//evidenzio 
		convertToDistance(x, y, diff);
		//Serial.print(diff);Serial.print("\t@x: "); Serial.print(x); Serial.print(",y:"); Serial.println(y);
		// visualizzo solo la differenza x 2
	}
	else
	{
		tft.drawPixel(x, y, diff);


	}

	if ((x == cursX) && (y == cursY)) {
		Serial.print("diff @curs:"); Serial.println(diff);
		Serial.print("red5bitLaserOn:"); Serial.println(red5bitLaserOn);
	}


}



 void loop(){
	resetValues();
	 readSensors();
	 // catturo l'immagine con il laser acceso
	 //dbg("\nLaser ON")
		// digitalWrite(PIN_LINELASER, !digitalRead(PIN_LINELASER));
	 //delay(10);

	processSerialCommand();

	//acquireAndDisplay_BASE();

	LASER_OFF 
	acquireRGB565AndDisplay();	//Salva l'immagine nell'array img[][] senza Laser
	displayStatus();
	displayCursor();

	LASER_ON
	resetValues();
	acquireRGB565AndProcess();
	displayCursor();
	displayStatus();
}
 long readNumber(int numDigits) {
#define MAXDIGITS 10
	 if (numDigits<MAXDIGITS)
	 {
		 char buffer[MAXDIGITS + 1];
		 Serial1.readBytes(buffer, numDigits);
		 buffer[numDigits] = '\0';
		 return atol(buffer);
	 }
	 else
	 {
		 return -1;

	 }
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