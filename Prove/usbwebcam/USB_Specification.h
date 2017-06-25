#ifndef USB_SPECIFICATION_H_
#define USB_SPECIFICATION_H_

#include "sam.h"

//USB request data transfer direction (bmRequestType)
#define  USB_REQ_DIR_OUT					(0<<7) //Host to device
#define  USB_REQ_DIR_IN						(1<<7) //Device to host

//USB request types (bmRequestType)		
#define  USB_REQ_TYPE_STANDARD				(0<<5) //Standard request
#define  USB_REQ_TYPE_CLASS					(1<<5) //Class-specific request
#define  USB_REQ_TYPE_VENDOR				(2<<5) //Vendor-specific request

//USB recipient codes (bmRequestType)
#define  USB_REQ_RECIP_DEVICE				(0<<0) //Recipient device
#define  USB_REQ_RECIP_INTERFACE			(1<<0) //Recipient interface
#define  USB_REQ_RECIP_ENDPOINT				(2<<0) //Recipient endpoint
#define  USB_REQ_RECIP_OTHER				(3<<0) //Recipient other

//Standard USB requests (bRequest)
#define  USB_REQ_GET_STATUS					0
#define  USB_REQ_CLEAR_FEATURE				1
#define  USB_REQ_SET_FEATURE				3
#define  USB_REQ_SET_ADDRESS				5
#define  USB_REQ_GET_DESCRIPTOR				6
#define  USB_REQ_SET_DESCRIPTOR				7
#define  USB_REQ_GET_CONFIGURATION			8
#define  USB_REQ_SET_CONFIGURATION			9
#define  USB_REQ_GET_INTERFACE				10
#define  USB_REQ_SET_INTERFACE				11
#define  USB_REQ_SYNCH_FRAME				12

//Class-specific UVC requests
#define  USB_REQ_SET_CUR					0x01
#define  USB_REQ_GET_CUR					0x81

#define  USB_DT_DEVICE						1
#define  USB_DT_CONFIGURATION				2
#define  USB_DT_STRING						3
#define  USB_DT_INTERFACE					4
#define  USB_DT_ENDPOINT					5
#define  USB_DT_DEVICE_QUALIFIER			6
#define  USB_DT_OTHER_SPEED_CONFIGURATION	7
#define  USB_DT_INTERFACE_POWER				8
#define  USB_DT_OTG							9
#define  USB_DT_IAD							0x0B
#define  USB_DT_BOS							0x0F
#define  USB_DT_DEVICE_CAPABILITY			0x10

//SETUP request
typedef struct
{
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} usb_setup_req_t;

//Standard USB device descriptor structure
typedef struct
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} usb_dev_desc_t;

//Standard USB configuration descriptor structure
typedef struct
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} usb_conf_desc_t;

typedef struct
{
	uint16_t bmHint;
	uint8_t bFormatIndex;
	uint8_t bFrameIndex;
	uint32_t dwFrameInterval;
	uint16_t wKeyFrameRate;
	uint16_t wPFrameRate;
	uint16_t wCompQuality;
	uint16_t wCompQualitySize;
	uint16_t wDelay;	
	uint32_t dwMaxVideoFrameSize;
	uint32_t dwMaxPayloadTransferSize;
	//uint32_t dwClockFrequency;
	//uint8_t bmFramingInfo;
	//uint8_t bPreferedVersion;
	//uint8_t bMinVersion;
	//uint8_t bMaxVersion;
} uvc_video_probe_and_commit_controls_t;

#endif /* USB_SPECIFICATION_H_ */