#include <Wire.h>
#include "ov7670.h"
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
#pragma region OV7670 camera

//#define ADDRESS         0x21 //Define i2c address of OV7670
//#define REGISTERS       0xC9 //Define total numbers of registers on OV7076

//#define XCLK_FREQ 10 * 1000000 //10.5 Mhz slow but ok.
#define XCLK_FREQ 20 * 1000000 //21 Mhz doubles the image; too fast?
//#define XCLK_FREQ 24 * 1000000 //42 Mhz

//3v3					3v3
//gnd                   gnd
//reset                 3v3
//pwdn                  gnd
//SIOD					SDA1 + 1k pullup
//SIOC					SCL1 + 1k pullup
//#define VSYNC_PIN		6
//#define HREF_PIN 		30
//#define PCLK_PIN		5
//#define XCLK_PIN		7
//#define D0_PIN			33
//#define D1_PIN			34
//#define D2_PIN			35
//#define D3_PIN			36
//#define D4_PIN			37
//#define D5_PIN			38
//#define D6_PIN			39
//#define D7_PIN			40

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

//#define VSYNC_PIN		52
//#define HREF_PIN 		6
//#define XCLK_PIN		7
//#define PCLK_PIN		32
//
//#define D0_PIN			44
//#define D1_PIN			45
//#define D2_PIN			46
//#define D3_PIN			47
//#define D4_PIN			48
//#define D5_PIN			49
//#define D6_PIN			50
//#define D7_PIN			51

#define VSYNC_BIT       (REG_PIOC_PDSR & (1 << 29))
#define HREF_BIT        (REG_PIOC_PDSR & (1 << 21))
#define PCLK_BIT        (REG_PIOC_PDSR & (1 << 22)) //pin8 is port C22 as stated here -> http://www.arduino.cc/en/Hacking/PinMappingSAM3X

#define WIDTH           ILI9341_TFTWIDTH
#define HEIGHT          ILI9341_TFTHEIGHT

volatile boolean onFrame = false;
volatile boolean onRow = false;
volatile boolean onPixel = false;

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

static hsv   rgb2hsv(rgb in);
static rgb   hsv2rgb(hsv in);

hsv rgb2hsv(rgb in)
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


rgb hsv2rgb(hsv in)
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

void setup()
{
	Serial.begin(115200);
 	SerialUSB.begin(0);
	Wire.begin();
// 	while (!Serial);
//	while (!SerialUSB);

#pragma region ILI9341_due tft  setup
//	pinMode(9, OUTPUT);

	// initialize SPI:
	SPI.begin();

	tft.begin();

	tft.setRotation(iliRotation0);
	tft.fillScreen(ILI9341_ORANGE);

	tft.setFont(Arial_bold_14);
	tft.setTextLetterSpacing(5);
	tft.setTextColor(ILI9341_RED, ILI9341_WHITE);
	tft.printAligned(F("OV7670_DUE.ino"), gTextAlignMiddleCenter);

#if 0
#pragma region test pushColors
	tft.setAddrWindowRect(0, 0, 240, 320); // set the drawing window
	for (byte x = 0; x < 200; x++) { // generate a simple grayscale gradient
		uint16_t color = (uint16_t)(255 * x) / 200;
		buf[x] = tft.color565(color, color, color);
	}
	tft.setAddrWindowRect(60, 200, 200, 3); // set the drawing window to a 200x3px rectangle
	for (byte y = 0; y < 3; y++) { // 3 lines
		tft.pushColors(buf, 0, 200); // draw the generated colors
	}

#pragma endregion

#endif // 0

#pragma endregion


#pragma region OV7670setup

#pragma region 10.5MHz clock
	//setupXCLK();
	//Wire.setClock(400000); //should work but needs some delay after every read/write. buggy?
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


	camInit();
	setRes(QVGA);//setRes(VGA); //ORIG
	setColorSpace(RGB565);//setColorSpace(BAYER_RGB);//setColorSpace(YUV422);

	//wrReg(0x12, 0x02); //ColorBar semitransparent overlay of the image
	//wrReg(0x42, 0x08); //ColorBar (DSP color bars at COM17)

	//wrReg(0x15, 0x02); //VSYNC inverted
	//wrReg(0x11, 0x82); //Prescaler x3 (10 fps)
	wrReg(0x11, 40); //con 40 sono 6 secondi per immagine
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

	pinMode(D0_PIN, INPUT);
	pinMode(D1_PIN, INPUT);
	pinMode(D2_PIN, INPUT);
	pinMode(D3_PIN, INPUT);
	pinMode(D4_PIN, INPUT);
	pinMode(D5_PIN, INPUT);
	pinMode(D6_PIN, INPUT);
	pinMode(D7_PIN, INPUT);

	attachInterrupt(VSYNC_PIN, vsync_rising, RISING);
	attachInterrupt(HREF_PIN, href_rising, RISING);

#pragma endregion

}



void loop0()
{
	//tft.clearDisplay();

	Serial.print(F("*FRAME_START..."));
	onFrame = false;
	while (!onFrame);
	for (int i = 0; i < HEIGHT; i++)//loop su lato lungo del TFT
	{
//		Serial.println(i);
		onRow = false;
		while (!onRow); //not working w/interrupt on href
		for (int j = 0; j < WIDTH; j++) //loop su lato corto
		{
			// LETTURA DEL PRIMO BYTE ------------------
			while (PCLK_BIT); //wait for low
			//while (!onPixel); // non working interrupt driven pclk
 			byteRRRRRGGG = READCAMERADATA;
			//while (onPixel); // non working interrupt driven pclk
			while (!PCLK_BIT); //wait for high

			// LETTURA DEL SECONDO BYTE----------------
			while (PCLK_BIT); //wait for low
			byteGGGBBBBB = READCAMERADATA;
			while (!PCLK_BIT); //wait for high

			//color565 = (bh << 8) | (bl);
		//	tft.drawPixel(i,j, color565);

			buf[j]=(byteRRRRRGGG << 8) | (byteGGGBBBBB);

		}
		//while (onRow); //not working w/interrupt on href
 		tft.pushColors(buf,0,WIDTH);
		

	}
	Serial.println(F("...STOP*"));

}
uint8_t r, g, b;
byte threshold = 20;
void loop() //funziona ma è lentino
{
	if (Serial.available())
	{
		threshold = Serial.parseInt();
	}

	//Serial.print(F("*FRAME_START..."));
	onFrame = false;
	while (!onFrame);
	//for (int i = 0; i < WIDTH; i++)//loop su lato corto del TFT
	for (int i = WIDTH; i >0; i--)//loop su lato corto del TFT
	{
		//		Serial.println(i);
		onRow = false;
		while (!onRow); //not working w/interrupt on href
		for (int j = 0; j <HEIGHT ; j++) //loop su lato lungo
		{
			// LETTURA DEL PRIMO BYTE ------------------
			while (PCLK_BIT); //wait for low
							  //while (!onPixel); // non working interrupt driven pclk
			byteRRRRRGGG = READCAMERADATA;
			//while (onPixel); // non working interrupt driven pclk
			while (!PCLK_BIT); //wait for high

							   // LETTURA DEL SECONDO BYTE----------------
			while (PCLK_BIT); //wait for low
			byteGGGBBBBB = READCAMERADATA;
			while (!PCLK_BIT); //wait for high

			//color565 = ((byteRRRRRGGG << 8) | (byteGGGBBBBB) ) ;// 
			//r = byteRRRRRGGG & 0b11111000;
			//g = ((byteRRRRRGGG & 0b00000111) << 5) | (byteGGGBBBBB >> 5);
			//b = (byteGGGBBBBB << 3);
			//tft.drawPixel(i, j, tft.color565(r, g, b));


			//color565 = ((byteRRRRRGGG >>3) << 11 ) ;// solo canale rosso
			if ((byteRRRRRGGG >> 3)>threshold)
			{
				color565 = ((byteRRRRRGGG >>3) << 11 ) ;// solo canale rosso
				//color565 = ((byteRRRRRGGG >>5) << 11 ) ;// solo valori alti del canale rosso 
				tft.drawPixel(i,j, color565);
			}
			else
			{
				tft.drawPixel(i,j, 0);

			}


		}
		//while (onRow); //not working w/interrupt on href
		//		Serial.write(buf, WIDTH);

	}
	//Serial.println(F("...STOP*"));
}