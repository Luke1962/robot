//#include "sam.h"
#include "WebCameraCapturing.h"
#include "UART.h"
#include "HCD.h"
#include "USBD.h"
#include "SSD1289.h"

#include "arduino.h"
#include <VM_DBG.h>

uint16_t arColors[5] = { WHITE, RED, GREEN, BLUE, BLACK };
uint8_t ColorIndex;
uint32_t SysTickCounter;

void ToggleLEDs(void)
{
	uint32_t Pin_LEDRX = 1 << (PIO_PC30_IDX & 0x1Fu);
	uint32_t Pin_LEDTX = 1 << (PIO_PA21_IDX & 0x1Fu);

	if (PIOC->PIO_PDSR & Pin_LEDRX)
	{
		PIOC->PIO_CODR = Pin_LEDRX;	//Lights on
		PIOA->PIO_CODR = Pin_LEDTX;
	}
	else
	{
		PIOC->PIO_SODR = Pin_LEDRX;	//Lights off
		PIOA->PIO_SODR = Pin_LEDTX;
	}
}

void SysTick_Handler(void)
{
	SysTickCounter++;
	if (1000 == SysTickCounter)
	{
		WDT->WDT_CR = WDT->WDT_CR | WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;	//Re-load watchdog

		SysTickCounter = 0;
		ToggleLEDs();
	}
}


void setup()
{

  /* add setup code here */
	SystemInit();	
	UART_Init();	
	LCD_Init();	
	
	LCD_SetArea(0, 0, 239, 319);	//Setting working area
	LCD_CLR_CS();					//Enable LCD
	LCD_SET_RS();					//Data write mode
	
	//Painting screen blue
	for(uint32_t i=0; i<76800; i++)
	{
		LCD_SET_DB(arColors[3]);
		LCD_CLR_WR();
		LCD_SET_WR();
	}
	
	//Setting frame area in the middle
	LCD_SetArea(LEFT_PIXEL, TOP_PIXEL, RIGHT_PIXEL, BOTTOM_PIXEL);
	
	//Cursor at left upper conner
	LCD_SetCursor(RIGHT_PIXEL, TOP_PIXEL);
	LCD_CLR_CS();
	
	//Painting frame white
	for(uint32_t i=0; i<TOTAL_PIXELS; i++)
	{
		LCD_SET_DB(arColors[0]);
		LCD_CLR_WR();
		LCD_SET_WR();
	}
	
	HCD_SetEnumerationStartFunction(USBD_GetDeviceDescriptorBegin);
	HCD_Init();
}

void loop()
{

  /* add main program code here */

}
