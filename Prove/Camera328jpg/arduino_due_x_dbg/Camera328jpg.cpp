/// provo a far dunzionare la libreria CameraC328R

#pragma region LIBRERIE ILI9341_due tft 


//#include <SPI.h>
#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

#include "arduino.h"

void getJPEGPicture_callback( uint16_t pictureSize, uint16_t packageSize, uint16_t packageCount, byte* package );
//
//
ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion


#include <CameraC328R.h>
//#include <NewSoftSerial.h>
/// #include <SoftwareSerial.h>
#define LED_PIN 23		//was 13
#define PAGE_SIZE 64
#define USB_BAUD 115200
#define CAMERA_BAUD 115200

#define TFTROWSPACING 15	//Altezza in pixel di una riga di testo
#define TFTROW(r) r*TFTROWSPACING
#define TFTDATACOL 100
//SoftwareSerial mySerial(2, 3);	//NewSoftSerial mySerial(2, 3);
///CameraC328R camera(&mySerial);
CameraC328R camera(Serial3);
 
uint16_t pictureSizeCount = 0;
 
/**
 * This callback is called EVERY time a JPEG data packet is received.
 */
void getJPEGPicture_callback( uint16_t pictureSize, uint16_t packageSize, uint16_t packageCount, byte* package )
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
 
void setup()
{
#pragma region ILI9341_due tft  setup
	//Serial.begin(9600);

	bool result = tft.begin();

	tft.clearDisplay();

	tft.setRotation(iliRotation90);
	tft.fillScreen(ILI9341_BLUE);

	tft.setFont(Arial_bold_14);
	tft.setTextLetterSpacing(5);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
	tft.printAt(F("### Test lib CameraC328R ###"), 1,0);


	//for (size_t x = PICT_OFFSET_X; x < PICT_OFFSET_X + PICT_SIZE_X; x++)
	//{
	//	for (size_t y = PICT_OFFSET_Y; y < PICT_OFFSET_Y + PICT_SIZE_Y; y++)
	//	{
	//		tft.drawPixel(x, y, ILI9341_RED);

	//	}
	//}


#pragma endregion


  Serial.begin( USB_BAUD );
  ///mySerial.begin(CAMERA_BAUD);
  Serial3.begin(CAMERA_BAUD);
 
  Serial.println("CameraC328R library test ");

  pinMode( LED_PIN, OUTPUT );
  digitalWrite( LED_PIN, LOW );
  delay(1000);
  for (int i = 0; i <5; i++)
  {
  digitalWrite( LED_PIN, HIGH );  delay(100);
  digitalWrite( LED_PIN, LOW);  delay(100);

  }
}
 
void loop()
{

 
		tft.printAt(F("Sync ..."), 1, TFTROW(1));
		if( !camera.sync() )
		{
			tft.printAt(F("Failed"), TFTDATACOL, TFTROW(1));
			Serial.println( "Sync failed." );
			return;
		}
		else
		{
			tft.printAt(F("OK"), TFTDATACOL, TFTROW(1));
		}
 

		tft.printAt(F("initial ...     "), 1, TFTROW(2));
		if( !camera.initial( CameraC328R::CT_JPEG, CameraC328R::PR_160x120, CameraC328R::JR_640x480 ) )
		{
			tft.printAt(F("Failed"), TFTDATACOL, TFTROW(2));
			Serial.println( "Initial failed." );
			return;
		}
		else
		{
			tft.printAt(F("OK"), TFTDATACOL, TFTROW(2));
		}
 
		if( !camera.setPackageSize( 64 ) )
		{
			Serial.println( "Package size failed." );
			return;
		}
 
		//if( !camera.setLightFrequency( CameraC328R::FT_50Hz ) )
		//{
		//  Serial.println( "Light frequency failed." );
		//  return;
		//}
 
		if( !camera.snapshot( CameraC328R::ST_COMPRESSED, 0 ) )
		{
			Serial.println( "Snapshot failed." );
			return;
		}
 
		pictureSizeCount = 0;
		if( !camera.getJPEGPicture( CameraC328R::PT_JPEG, PROCESS_DELAY, &getJPEGPicture_callback ) )
		{
			Serial.println( "Get JPEG failed." );
			return;
		}

}

