#ifndef UART_H_
#define UART_H_

#include "sam.h"

void UART_Init(void);
void PrintStr(char *Text);

void PrintHEX(uint8_t* ptrBuffer, uint32_t Length);
void PrintHEX16(uint16_t Number);
void PrintHEX32(uint32_t Number);

void PrintBIN8(uint8_t Number);
void PrintBIN16(uint16_t Number);
void PrintBIN32(uint32_t Number);

void PrintDEC(uint32_t Number);

void Print_UOTGHS_HSTPIPISR(uint32_t);
void Print_UOTGHS_HSTPIPINRQ(uint32_t);

#endif