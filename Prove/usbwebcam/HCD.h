#ifndef HCD_H_
#define HCD_H_

#include "sam.h"
#include "USB_Specification.h"

//Definitions to store control transfer state machine
typedef enum
{
	USB_CTRL_REQ_STAGE_SETUP = 0,
	USB_CTRL_REQ_STAGE_DATA_OUT = 1,
	USB_CTRL_REQ_STAGE_DATA_IN = 2,
	USB_CTRL_REQ_STAGE_ZLP_IN = 3,
	USB_CTRL_REQ_STAGE_ZLP_OUT = 4,
} hcd_ControlRequestStageType;

typedef struct
{
	uint8_t SOFCount;									//To count every 1ms
	
	uint8_t Direction;
	hcd_ControlRequestStageType ControlRequestStage;	//Current stage of control transaction
	
	uint8_t* ptrBuffer;									//Receive/send buffer 
	uint8_t ByteCount;									//Receive/send data size
	uint16_t Index;										//Current position in *ptrBuffer
	
	uint16_t Pause;										//Over how many ms to call TransferStart or ResetEnd	
	void (*ResetEnd)(void);								//Called after reset happened
	void (*TransferStart)(void);						//Called after Pause elapsed
	void (*TransferEnd)(uint16_t);						//Called when transfer is finished
} hcd_ControlStructureType;

typedef struct
{
	uint32_t* ptrBuffer;								//Receive buffer
	void (*TransferEnd)(uint16_t, uint32_t*);			//Called when transfer is finished
} hcd_IsochronousControlStructureType;
hcd_IsochronousControlStructureType hcd_IsochronousControlStructure;

void HCD_Init(void);
void HCD_SetEnumerationStartFunction(void (*ResetEnd)(void));
void HCD_StartDelayed(void (*TransferStart)(void), uint16_t Pause);
uint32_t HCD_InitiatePipeZero(uint8_t Address);
uint32_t HCD_InitiateIsochronousINPipeOne(uint8_t Address, uint8_t EndpointNumber);
void HCD_SendCtrlSetupPacket(uint8_t EndpAddress, uint8_t Direction, usb_setup_req_t SetupPacketData, uint8_t *ptrBuffer, uint16_t TransferByteCount, void (*TransferEnd)(uint16_t ByteReceived));



#endif /* HCD_H_ */