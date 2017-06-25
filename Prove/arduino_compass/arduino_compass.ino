/*           Ardumotive Project Corner
   Arduino digital magnetic Compass with two display modes
   using 3-Axis Digital Compass IC HMC5883L board
   Code,libraries,circuit and more info here: http://bit.ly/1CqSsMF
   Dev. Vasilakis Michalis / Ver. 1.0 / Date:17/3/2015      */

//Include libraries
#include <SPI.h>
#include <Wire.h>
#include <Button.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>

//Init Display (SCLK, DIN, D/C, CS, RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);
//Init HMC5883L sensor
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

//Constants
const int lcdLight = 13; //BL pin of LCD to arduino pin 13 (BL = Back Light)
Button buttonA = Button(2,PULLUP); //Button to turn on/off display light
Button buttonB = Button(8,PULLUP); //Button for changing display mode
// For 1st display mode:
static unsigned char Leters[] = { 'N' , 'E' , 'S' , 'W' }; 
static unsigned char PROGMEM arrow_bmp[] ={B00100000, B00100000, B01110000, B01110000, B11111000,};
// For 2nd display mode:
int r = 24; // radius of compass rose
int x0= 60; // x-origin
int y0= 24; // y-origin

//Variables
int flag=0;//Store buttonA condition 0:1st display mode / 1: 2nd display mode
int i=1;   //Store buttonB condition 1: light on / 0: light off

void setup() {
  pinMode(lcdLight, OUTPUT); // lcd led
  compass.begin();
  Wire.begin();
  display.begin();
  display.setContrast(60);
  //Print a welcome message in startup for 6sec.//////////
  digitalWrite(lcdLight,HIGH);
  display.clearDisplay();   // clears the screen and buffer
  display.setTextColor(BLACK);
  display.setCursor(0,2);
  display.print("  ARDUMOTIVE  ");
  display.setCursor(0,13);
  display.print(" ArduinoBased ");
  display.setCursor(0,23);
  display.print(" compassn.Compass ");
  display.setCursor(0,33);
  display.print(" Michalis Vas.");  
  display.display(); // show splashscreen
  digitalWrite(lcdLight,LOW);
  delay(6000);
  /////////////////////////////////////////////////////////
  display.clearDisplay();   // clears the screen and buffer
}

void loop() {

  //if buttonB is pressed, 'flag' will change between 0 and 1
    if(buttonB.isPressed()){
      delay(100);//small delay
    if(flag==0){ flag=1; }
    else{ flag=0; }
  }
  //if buttonA is pressed, 'i' will change between 0 and 1
  if(buttonA.isPressed()){
    if(i==0){ i=1; }
    else{ i=0; }
  }
  //if i is equal with 0, set display light on, else set display light off
    if (i==0){  
	digitalWrite(lcdLight,HIGH);}
    else{
	digitalWrite(lcdLight,LOW);
   }
  //Reading and calculate degrees:   /////////////////////////////////////
  //Get new sensor value every time that loop starting again
  sensors_event_t event; 
  compass.getEvent(&event);
  //Variable heading stores value in radians
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the compassnetic field in your location.
  // Find yours here: http://www.compassnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  float declinationAngle = 0.22;  //<--Change 0.22 with yours. If you can't find your declination juct delete those lines ;)
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)   heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)heading -= 2*PI;
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  //Convert float to int
  int angle=(int)headingDegrees;
  //////////////////////////////////////////////////////////////////////////

  display.clearDisplay(); // Just in case...
  
  //if flag is equal with 0 - 1st Display Mode 
  if (flag==0){
  DrawRow(angle); //call DrawRow function
  display.drawBitmap(40, 24,  arrow_bmp, 5, 5, 1);
  display.drawLine(42, 0, 42, 24, BLACK);
  display.setCursor(32,34);  
  display.print(angle);
  display.setTextSize(1);
  display.print("o");
  display.display();
  delay(500); //print new value every 0.5sec
  }
  
  //if flag is equal with 1 - 2nd Display Mode
  if (flag==1){
  // Calculation of compass needle on lcd pointing to the direction 
  DrawCircle(angle); //call DrawCircle function
  // Display actual heading
  display.setTextSize(2);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.println(angle);
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(x0-2,0);
  display.println("N");
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor((x0+r)-5,y0-3);
  display.println("E");
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(x0-2,y0+r-8);
  display.println("S");
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor((x0-r)+5,y0-3);
  display.println("W");
  
  // Triangle for direction
  display.drawTriangle(0, 46, 20, 46, 10, 18, BLACK);
  display.fillTriangle (0, 46, 20, 46, 10, 18, BLACK);
  display.display();
  delay(500); //print new value every 0.5sec
  }
}/// end of program

////////////      Functions    ////////////

//Draw rows and print letters to display - 1st Display Mode
void DrawRow(int angle) {

  display.drawLine(0, 0, 84, 0, BLACK);
  display.drawLine(0, 1, 84, 1, BLACK);

  display.drawLine(0, 22, 91, 22, BLACK);
  display.drawLine(0, 23, 95, 23, BLACK);
  
  display.setTextSize(2);
  display.setTextColor(BLACK);  
  
  int start = 42 - angle / 3 ;
  if (start > 120) start += -120 ;

  int x = 0 ;
  int y = 18 ;
  for (int i=0; i<4; i++) {
      x = start + (i*30) -1;
      if (x>119) x += -120;
      display.drawPixel(x+1, y-2, 1);
      display.drawPixel(x, y, 1);
      display.drawPixel(x+1, y, 1);
      display.drawPixel(x+2, y, 1);
      display.drawPixel(x, y-1, 1);
      display.drawPixel(x+1, y-1, 1);
      display.drawPixel(x+2, y-1, 1);
      display.setCursor((x-4),(y-16)); 
      display.write(Leters[i]);
   }
  for (int i=0; i<24; i++) {
      x = start + (i*5) -1;
      if (x>119) x += -120;
      display.drawPixel(x+1, y+1, 1);
      display.drawPixel(x, y+2, 1);
      display.drawPixel(x+1, y+2, 1);
      display.drawPixel(x+2, y+2, 1);
      display.drawPixel(x, y+3, 1);
      display.drawPixel(x+1, y+3, 1);
      display.drawPixel(x+2, y+3, 1);
   };
   
  for (int i=0; i<8; i++) {
      x = start + (i*15)-1;
      if (x>119) x += -120;
      display.drawPixel(x+1, y-1, 1);

      display.drawPixel(x, y, 1);
      display.drawPixel(x+1, y, 1);
      display.drawPixel(x+2, y, 1);
      display.drawPixel(x, y+1, 1);
      display.drawPixel(x+1, y+1, 1);
      display.drawPixel(x+2, y+1, 1);

   };
}

////Draw circle and print letters to display - 1st Display Mode
void DrawCircle(int angle) {
  if (angle >= 0 && angle <=45) {    // 0-45 degrees
    int angle = angle/2;
    display.drawLine(x0, y0, x0+angle, y0-r, BLACK);
  } 
  else if (angle >45 && angle <=90) {// 46-90 degrees
    int angle = (angle-44)/2 ;
    display.drawLine(x0, y0, x0+r, (y0-r)+angle, BLACK);
  } 
  else if (angle >90 && angle <=135) {// 91-135 degrees
    int angle = (angle-90)/2;
    display.drawLine(x0, y0, x0+r, y0+angle, BLACK);
  } 
  else if (angle >135 && angle <=180){// 136-180 degrees
    int angle = (angle-134)/2;
    display.drawLine(x0, y0, (x0+r)-angle, y0+r, BLACK);
  } 
  else if (angle >180 && angle <=225){// 181-225 degrees
    int angle = (angle-178)/2; 
    display.drawLine(x0, y0, x0-angle, y0+r, BLACK);
  } 
  else if (angle >225 && angle <=270){// 226-270 degrees
    int angle = (angle-222)/2; 
    display.drawLine(x0, y0, x0-r, (y0+r)-angle, BLACK);
  } 
  else if (angle >270 && angle <=315){// 271-315 degrees
    int angle = (angle-270)/2;
    display.drawLine(x0, y0, x0-r, y0-angle, BLACK);
  } 
  else if (angle >315 && angle <=360){// 316-360 degrees
    int angle = (angle-312)/2;
    display.drawLine(x0, y0, (x0-r)+angle, y0-r, BLACK);
  }
}
