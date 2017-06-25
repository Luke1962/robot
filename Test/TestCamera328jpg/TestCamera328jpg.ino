#include <CameraC328R.h>
#include <NewSoftSerial.h>
#define LED_PIN 13
#define PAGE_SIZE 64
#define USB_BAUD 115200
#define CAMERA_BAUD 14400
NewSoftSerial mySerial(2, 3);
CameraC328R camera(&mySerial);
uint16_t pictureSizeCount = 0;
/**
 * This callback is called EVERY time a JPEG data packet is received.
 */
void getJPEGPicture_callback( uint16_t pictureSize, uint16_t packageSize,
uint16_t packageCount, byte* package )
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
 Serial.begin( USB_BAUD );
 mySerial.begin(CAMERA_BAUD);
 pinMode( LED_PIN, OUTPUT );
 digitalWrite( LED_PIN, LOW );
}
void loop()
{
 if( Serial.available() ){
  while(Serial.read() != -1);
  digitalWrite( LED_PIN, HIGH );
  if( !camera.sync() )
  {
    Serial.println( "Sync failed." );
    return;
 }
 if( !camera.initial( CameraC328R::CT_JPEG, CameraC328R::PR_160x120,CameraC328R::JR_640x480 ) )
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
 }
}
