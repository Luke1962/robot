#ifndef USBD_H_
#define USBD_H_

#include <string.h>
#include "USB_Specification.h"
#include "UART.h"
#include "HCD.h"
#include "VideoProcessing.h"

#define DeviceAddress		0x05
#define DeviceConfiguration 1
uint8_t Buffer[1024];

void USBD_GetDeviceDescriptorBegin(void);

void PrintDeviceDescriptor(uint8_t* ptrBuffer);
void PrintConfigurationDescriptor(uint8_t* ptrBuffer);
void PrintInterfaceAssociationDescriptor(uint8_t* ptrBuffer);
void PrintInterfaceDescriptor(uint8_t* ptrBuffer, uint32_t* Is_VS_Descriptors);
void PrintVCHeaderDescriptor(uint8_t* ptrBuffer);
void PrintVCInputTerminalDescriptor(uint8_t* ptrBuffer);
void PrintVCProcessingUnitDescriptor(uint8_t* ptrBuffer);
void PrintVCOutputTerminalDescriptor(uint8_t* ptrBuffer);
void PrintVCExtensionUnitDescriptor(uint8_t* ptrBuffer);
void PrintEndpointDescriptor(uint8_t* ptrBuffer);
void PrintClassSpecificInterruptEndpointDescriptor(uint8_t* ptrBuffer);
void PrintVSInputHeaderDescriptor(uint8_t* ptrBuffer);
void PrintVSUncompressedVideoFormatDescriptor(uint8_t* ptrBuffer);
void PrintVSUncompressedVideoFrameDescriptor(uint8_t* ptrBuffer);
void PrintVSStillImageFrameDescriptor(uint8_t* ptrBuffer);
void PrintVSColorMatchingDescriptors(uint8_t* ptrBuffer);

void PrintVideoProbeAndCommitControls(uint8_t* ptrBuffer);

#endif /* USBD_H_ */