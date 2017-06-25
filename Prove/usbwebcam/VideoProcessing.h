#ifndef VIDEOPROCESSING_H_
#define VIDEOPROCESSING_H_

#include "UART.h"
#include "SSD1289.h"

typedef struct
{
	uint16_t TotalSize;
	uint8_t HeaderSize;
	uint8_t HeaderBitfield;
	uint32_t Frame;
	uint32_t CurrentBank;
} VP_HeaderInfoType;

void VP_ProcessPartialPayload_Buffering(uint16_t ByteReceived, uint32_t* ptrBuffer);
void VP_ProcessPartialPayload_Logging(uint16_t ByteReceived, uint32_t* ptrBuffer);


#endif /* VIDEOPROCESSING_H_ */