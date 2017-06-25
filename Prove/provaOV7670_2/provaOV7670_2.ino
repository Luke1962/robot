/*
 * OV7670 + SSD1289 or UART
 * Author: Szymon Kłos
 *
 * Atmega16 with 16 MHz crystal oscillator requied !!!
 * You should power MCU and camera with 3,3V.
 * SDA and SCL connect by resistor to VCC.
 *
 * You can receive the image on the PC using my tool: https://github.com/sk94/cam_capture
 *https://github.com/eszkadev/ov7670
*/

#include "Arduino.h"

#include <Wire.h>
#include <SPI.h>
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


#include <ov7670c/ov7670c.h>
#ifdef WITH_LCD
//#include <ov7670c/hardware/lcd.h>
inline static void lcd_init(void) __attribute__((always_inline));
inline static void draw_pixel(uint16_t x, uint16_t y, uint16_t color) __attribute__((always_inline));

inline static void lcd_init(void)
{
	//SSD1289_Init();
	//FillScreen(RGB(0, 0, 0));
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
}

inline static void draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
	tft.drawPixel(x, y, color);
}

#else
#include "hardware/uart.h"
#endif

void set_registers(void);
#define ADDRESS         0x21 //Define i2c address of OV7670

void wrReg(uint8_t reg, uint8_t dat)
{
	delay(5);
	Wire.beginTransmission(ADDRESS);  //Start communication
	Wire.write(reg);				   //Set the register
	Wire.write(dat);				   //Set the value
	Wire.endTransmission();		   //Send data and close communication
}
void OV7670_init(void)
{
	// IO PINS
	//set_data_in();
	//set_pclk_in();
	//set_vsync_in();
	//set_hsync_in();
	//init_xclk();

	//i2c_init();

	wrReg(0x12, 0x80); // reset the camera registers
	delay(200);

	wrReg(REG_CLKRC, 0x3F); // frequency divider - 3F max

	set_registers();

	set_resolution(res_QQVGA);
	set_color(color_RGB565);
}

void set_resolution(uint8_t res)
{
	switch (res)
	{
	case res_QQVGA:
		wrReg(REG_HSTART, 0x16);
		wrReg(REG_HSTOP, 0x04);
		wrReg(REG_HREF, 0x24);
		wrReg(REG_VSTART, 0x02);
		wrReg(REG_VSTOP, 0x7a);
		wrReg(REG_VREF, 0x0a);
		wrReg(REG_COM14, 0x1a);
		wrReg(0x72, 0x22);
		wrReg(0x73, 0xf2);
		break;

	case res_QVGA:
		wrReg(REG_HSTART, 0x16);
		wrReg(REG_HSTOP, 0x04);
		wrReg(REG_HREF, 0x24);
		wrReg(REG_VSTART, 0x02);
		wrReg(REG_VSTOP, 0x7a);
		wrReg(REG_VREF, 0x0a);
		wrReg(REG_COM14, 0x19);
		wrReg(0x72, 0x11);
		wrReg(0x73, 0xf1);
		break;
	}
}

void set_color(uint8_t color)
{
	switch (color)
	{
	case color_RGB565:
		wrReg(REG_COM7, 0x04); // RGB mode
		wrReg(REG_COM15, 0xD0); // RGB565 full range
		break;
	}
}

void capture_image(uint16_t w, uint16_t h)
{
	uint8_t x = 0;
	uint8_t y = 0;

	y = 0;
	while (!get_vsync());
	while (y < h)
	{
		x = 0;
		while (!get_hsync());
		while (get_hsync())
		{
			while (!get_pclk());

#ifdef WITH_LCD
			uint16_t color = ((uint16_t)get_data()) << 8;
#else
			uint8_t c = get_data();
			uart_put(c);
#endif

			while (get_pclk());
			while (!get_pclk());

#ifdef WITH_LCD
			color += get_data();
			if (x<200 && y<200)
			{
				tft.drawPixel(x, y, color);
			}
#else
			c = get_data();
			uart_put(c);
			while (get_pclk());
			while (!get_pclk());

#endif

			++x;
		}
		++y;
	}
}

void set_registers(void)
{
	wrReg(REG_TSLB, 0x04);
	wrReg(REG_COM3, 0x04);

	// test pattern
	wrReg(0x70, 0x4a);
#ifdef test_pattern
	wrReg(0x71, 0x35 | (1 << 7));
#else
	wrReg(0x71, 0x35);
#endif

	// colors
	wrReg(MTX1, 0x80);
	wrReg(MTX2, 0x80);
	wrReg(MTX3, 0x00);
	wrReg(MTX4, 0x22);
	wrReg(MTX5, 0x5e);
	wrReg(MTX6, 0x80);
	wrReg(MTXS, 0x9e);
	wrReg(AWBC7, 0x88);
	wrReg(AWBC8, 0x88);
	wrReg(AWBC9, 0x44);
	wrReg(AWBC10, 0x67);
	wrReg(AWBC11, 0x49);
	wrReg(AWBC12, 0x0e);
	wrReg(REG_GFIX, 0x00);
	wrReg(AWBCTR3, 0x0a);
	wrReg(AWBCTR2, 0x55);
	wrReg(AWBCTR1, 0x11);
	wrReg(AWBCTR0, 0x9f);

	wrReg(REG_COM8, 0x8F); // AGC AWB AEC Unlimited step size
	wrReg(0xAA, 0x14);
	wrReg(REG_BRIGHT, 0x00);
	wrReg(REG_CONTRAS, 0x40);

	/*wrReg(MTX1, 0xc0);
	wrReg(MTX2, 0xc0);
	wrReg(MTX3, 0x00);
	wrReg(MTX4, 0x33);
	wrReg(MTX5, 0x8d);
	wrReg(MTX6, 0xc0);
	wrReg(MTXS, 0x9e);*/
}


// To turn off test pattern comment the test_pattern definition in ov7670_config.h
void setup()
{  
	Serial.begin(115200);
#pragma region ILI9341_due tft  setup

  // initialize SPI:
  SPI.begin();

  tft.begin();

  tft.setRotation(iliRotation90);
  tft.fillScreen(ILI9341_ORANGE);

  tft.setFont(Arial_bold_14);
  tft.setTextLetterSpacing(5);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.printAligned(F("provaOV7670_2"), gTextAlignMiddleCenter);

  //for (size_t x = PICT_OFFSET_X; x < PICT_OFFSET_X + PICT_SIZE_X; x++)
  //{
  //	for (size_t y = PICT_OFFSET_Y; y < PICT_OFFSET_Y + PICT_SIZE_Y; y++)
  //	{
  //		tft.drawPixel(x, y, ILI9341_RED);

  //	}
  //}


#pragma endregion

	OV7670_init();

}
void loop()
{

			capture_image(160, 120);
			//capture_image(640, 480);


}
