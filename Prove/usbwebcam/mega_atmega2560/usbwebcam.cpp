
#include "WebCameraCapturing.h"
#include "UART.h"
#include "HCD.h"
#include "USBD.h"
#include "SSD1289.h"


#include "arduino.h"
#include <VM_DBG.h>

void ToggleLEDs(void);
void SysTick_Handler(void);
//
//
uint16_t arColors[5] = { WHITE, RED, GREEN, BLUE, BLACK };
uint8_t ColorIndex;
uint32_t SysTickCounter;

void ToggleLEDs(void)
{
uint32_t Pin_LEDRX = 1 << (PIO_PC30_IDX & 0x1Fu);
uint32_t Pin_LEDTX = 1 << (PIO_PA21_IDX & 0x1Fu);

if (PIOC->PIO_PDSR & Pin_LEDRX)
{
PIOC->PIO_CODR = Pin_LEDRX;
PIOA->PIO_CODR = Pin_LEDTX;
}
else
{
PIOC->PIO_SODR = Pin_LEDRX;
PIOA->PIO_SODR = Pin_LEDTX;
}
}

void SysTick_Handler(void)
{
SysTickCounter++;
if (1000 == SysTickCounter)
{
WDT->WDT_CR = WDT->WDT_CR | WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;

SysTickCounter = 0;
ToggleLEDs();
}
}


void setup() { MicroDebug.init(2000); MicroDebug.begin(&Serial,115200);MicroDebug.outPacketStart();MicroDebug.transport->print("VMDPV_1|1_VMDPV\r\n");delay(10); MicroDebug.start(false,true);



SystemInit();
UART_Init();
LCD_Init();

LCD_SetArea(0, 0, 239, 319);
LCD_CLR_CS();
LCD_SET_RS();


for(uint32_t i=0; i<76800; i++)
{
LCD_SET_DB(arColors[3]);
LCD_CLR_WR();
LCD_SET_WR();
}


LCD_SetArea(LEFT_PIXEL, TOP_PIXEL, RIGHT_PIXEL, BOTTOM_PIXEL);


LCD_SetCursor(RIGHT_PIXEL, TOP_PIXEL);
LCD_CLR_CS();


for(uint32_t i=0; i<TOTAL_PIXELS; i++)
{
LCD_SET_DB(arColors[0]);
LCD_CLR_WR();
LCD_SET_WR();
}

HCD_SetEnumerationStartFunction(USBD_GetDeviceDescriptorBegin);
HCD_Init();
}

void loop() {




}




