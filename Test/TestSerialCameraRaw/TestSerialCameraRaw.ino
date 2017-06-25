///Memoria SRAM Arduino DUE: (per le variabili del programma):  96 kB

#include <CameraC328R.h>
#ifndef __SAM3X8E__
//#include <NewSoftSerial\NewSoftSerial.h>
#endif // __SAM3X8E__
#define LED_PIN 13
#define PAGE_SIZE 64
#define USB_BAUD 115200
#define CAMERA_BAUD 115200


#ifdef __SAM3X8E__
///#define cameraSerial Serial3
CameraC328R camera(CAMERASERIALPORT);

//#else
//SoftwareSerial cameraSerial(2, 3);
//CameraC328R camera(&cameraSerial);

#endif // __ARM__
uint16_t pictureSizeCount = 0;
/**
* This callback is called EVERY time a JPEG data packet is received.
*/
void getJPEGPicture_callback( uint16_t pictureSize, uint16_t packageSize,uint16_t packageCount, byte* package )
{
	// packageSize is the size of the picture part of the package
	pictureSizeCount += packageSize;
	Serial.write(package,packageSize);
	if( pictureSizeCount >= pictureSize )
	{
		digitalWrite( LED_PIN, LOW );
		Serial.flush();
	}
}
uint16_t cameraProcessDelayMsec = 100;
#define IMGSIZE 19200
uint16_t cameraImgBuffSize = IMGSIZE; //160X120 Dimensione del buffer immagine non comressa. Se minore della dimensione dell'immagine si verifica errore
byte imgArray[IMGSIZE];
void setup()
	{
	Serial.begin( USB_BAUD );
	CAMERASERIALPORT.begin(CAMERA_BAUD);
	pinMode( LED_PIN, OUTPUT );
	digitalWrite( LED_PIN, LOW );
	}
void loop(){
	if( Serial.available() ){
	while(Serial.read() != -1);
	digitalWrite( LED_PIN, HIGH );
	if( !camera.sync() )
	{

		Serial.println( "Sync failed." );
		return;
	}
	if( !camera.initial( CameraC328R::CT_JPEG, CameraC328R::PR_160x120,	CameraC328R::JR_640x480 ) )
	{
		Serial.println( "Initial failed." );
		return;
	}
	if( !camera.setPackageSize( 64 ) )
	{
		Serial.println( "Package size failed." );
		return;
	}
	if( !camera.setLightFrequency( CameraC328R::FT_50Hz ) )
	{
		Serial.println( "Light frequency failed." );
		return;
	}
#if 0
	if( !camera.snapshot( CameraC328R::ST_COMPRESSED, 0 ) )
	{
		Serial.println( "Snapshot failed." );
		return;
	}  
	pictureSizeCount = 0;
	if( !camera.getJPEGPicture( CameraC328R::PT_JPEG, PROCESS_DELAY,&getJPEGPicture_callback ) )
	{
		Serial.println( "Get JPEG failed." );
		return;
	}

#else
	if( !camera.snapshot( CameraC328R::ST_UNCOMPRESSED,0 ) )
	{
		Serial.println( "Snapshot ST_UNCOMPRESSED failed." );
		return;
	}
	pictureSizeCount = 0;
	if( !camera.getRawPicture( CameraC328R::PT_PREVIEW, imgArray ,cameraImgBuffSize, cameraProcessDelayMsec) )
	{
		Serial.println( "Get JPEG failed." );
		return;
	}

	for (size_t i = 0; i < cameraImgBuffSize; i++)
	{
		Serial.println(imgArray[i]);
	}
#endif // 0

}
}
