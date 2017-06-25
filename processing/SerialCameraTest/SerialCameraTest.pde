import processing.serial.*;
Serial myPort;
String filename = "photo.jpg";
byte[] photo = {};
Boolean readData = false;

void setup()
{
  println( Serial.list() );
  myPort = new Serial( this, Serial.list()[0], 115200 );
}
void draw()
{
  byte[] buffer = new byte[64]; //<>//
  if( readData )
  {
    while( myPort.available() > 0 ) //<>//
    {
      int readBytes = myPort.readBytes( buffer );
      print( "Read " );
      print( readBytes );
      println( " bytes ..." );
      for( int i = 0; i < readBytes; i++ )
      {
        photo = append( photo, buffer[i] );
      }
    }
  }
  else
  {
    while( myPort.available() > 0 )
    {
      print( "COM Data: " );
      println( myPort.readString() );
    }
  }
}
void keyPressed()
{
  if( photo.length > 0 ) {
    readData = false;
    print( "Writing to disk " );
    print( photo.length );
    println( " bytes ..." );
    saveBytes( filename, photo );
    println( "DONE!" );
  }
  else
  {
    readData = true;
    myPort.clear();
    println( "Waiting for data ..." );
  }
}