#ifndef SSD1289_H_
#define SSD1289_H_

#include "sam.h"

#define LCD_CLR_CS()			PIOC->PIO_CODR = PIO_PC8
#define LCD_SET_CS()			PIOC->PIO_SODR = PIO_PC8

#define LCD_CLR_RS()			PIOC->PIO_CODR = PIO_PC6
#define LCD_SET_RS()			PIOC->PIO_SODR = PIO_PC6

#define LCD_CLR_WR()			PIOC->PIO_CODR = PIO_PC7
#define LCD_SET_WR()			PIOC->PIO_SODR = PIO_PC7

#define LCD_CLR_RST()			PIOC->PIO_CODR = PIO_PC9
#define LCD_SET_RST()			PIOC->PIO_SODR = PIO_PC9

#define LCD_SET_DB(x)			PIOA->PIO_ODSR =  ((x & PIO_PA6) << 1)\
												| ((x & (PIO_PA9 | PIO_PA10)) << 5);\
								PIOB->PIO_ODSR =  ((x & PIO_PB8) << 18);\
								PIOC->PIO_ODSR =  ((x & PIO_PC4) >> 3)\
												| ((x & PIO_PC3) >> 1)\
												| ((x & PIO_PC2) << 1)\
												| ((x & PIO_PC1) << 3)\
												| ((x & PIO_PC0) << 5);\
								PIOD->PIO_ODSR =  ((x & (PIO_PA11 | PIO_PA12 | PIO_PA13 | PIO_PA14)) >> 11)\
												| ((x & PIO_PA15) >> 9)\
												| ((x & PIO_PD7) << 2) \
												| ((x & PIO_PD5) << 5);\
												
#define RGB(red, green, blue)	((uint16_t)(((red >> 3) << 11) | ((green >> 2) << 5) | (blue  >> 3)))

void LCD_WrCmd(uint16_t);
void LCD_WrDat(uint16_t);
void LCD_WaitMs(uint32_t ms);
void LCD_Init(void);
void LCD_SetCursor(uint16_t x, uint16_t y);
void LCD_SetArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

#endif /* SSD1289_H_ */