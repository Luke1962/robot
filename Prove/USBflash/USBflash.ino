/*
** Wire Master for Hobbytronics USB Host Flash Drive
** Created 16 Oct 2014
**
** This example code is in the public domain.
** www.hobbytronics.co.uk
*/

#include <Wire.h>

// I2C Register Addresses
#define I2C_FILENAME		0
#define I2C_WRITE		1
#define I2C_APPEND		2
#define I2C_READ		3
#define I2C_READ_SETLINE	4
#define I2C_FLASH_READY		99

const char  flash_address=41;   // I2C Address

void setup()
{
  Serial.begin(9600);
  Wire.begin();              // join i2c bus (address optional for master) 
  
  flash_write(flash_address, I2C_FILENAME, "i2c.txt");
  flash_write(flash_address, I2C_WRITE, "Test File\r\n");  // Overwrite file
}

void loop()
{
  // In this loop we keep writing "Hello" to the file, and we read and print 
  // the first line which should be "Test File" 
  flash_write(flash_address, I2C_APPEND, "Hello\r\n"); // Append to file
  flash_read_line(flash_address, 1);             // read line 1 of the file
  delay(5000);                                   // Wait 5 seconds

}

// Check if flash drive is available (i.e. not currently reading/writing)
byte flash_ready(char host_address)
{
  byte f_ready;
  // This function reads one byte over I2C
  Wire.beginTransmission(host_address);
  Wire.write(I2C_FLASH_READY); 
  Wire.endTransmission(false);         // restart
  Wire.requestFrom(host_address, 1);   // Request the data...
  f_ready = Wire.read();
  if(f_ready==0) delay(20);            // A small delay before we try again
  return f_ready;
}

void flash_write(char host_address, char h_command, char *pstring)
{
    unsigned char i=0;
    
    while(!flash_ready(host_address));     // Wait until Flash drive is ready 
      
    Wire.beginTransmission(host_address);  // transmit to device
    Wire.write((byte) h_command);          // send command - WRITE or APPEND
    do{ 
        Wire.write((byte) pstring[i]);
        i++;
    } while(pstring[i]);    

    Wire.endTransmission(); 
}

void flash_read_line(char host_address, unsigned char line_num){
    // Read in a line from flash drive
    // This is a 2-stage process. First we tell the host board
    // which line number we want, so it has time to fetch the data
    // Next we request the data
    unsigned char i=0;
    
    while(!flash_ready(host_address));       // Wait until Flash drive is ready 
      
    Wire.beginTransmission(host_address);    // transmit to device
    Wire.write((byte) I2C_READ_SETLINE);     // send SETLINE command
    Wire.write((byte) line_num);             // send line number    
    Wire.endTransmission(); 

    // Wait until line is fetched from flash drive into buffer
    while(!flash_ready(host_address));       // Wait until Flash drive is ready     

    Wire.beginTransmission(host_address);    // transmit to device
    Wire.write((byte) I2C_READ);             // send READ command
    Wire.endTransmission(false);             // restart
    Wire.requestFrom(host_address, 255);     // request max 255 bytes from slave 
                                             // device (Arduino wont do this many)

    while(Wire.available())    // slave may send less than requested
    {
       char c = Wire.read();    // receive a byte as character
       if(c!='\0') Serial.print(c);         // print the character
    }
}
