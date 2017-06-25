// UTFT_Demo with Font and icon Flash IC 
// (C) 2012 Coldtears electronics
// For use with CTE 3.6" TFT LCD Module w/Font and Icon Flash IC
// in combination with CTE TFT/SD Shield for Arduino MEGA2560
// web: http://stores.shop.ebay.com/coldtears-electronics-store
//
// Demo code and UTFT library freely modified from Henning Karlsen's UTFT Library
// Original by
// UTFT_Demo_400x240 (C)2012 Henning Karlsen
// web: http://www.henningkarlsen.com/electronics
//
// The first part of the demo is to show how to use the font IC to draw font to the LCD,
// The IC also include some commonly used and open source icons
// great for custom application development.
//
// This second part of the demo is by Henning Karlsen,
// showing how to use most of the functions
// of the library with a supported display modules.
//
// This demo was made for modules with a screen resolution 
// of 400x240 pixels.
//
// This program requires the UTFT library which is modified by Coldtears electronics
//

#include "SPI.h"
#include <UTFT\UTFT.h>
#define FLASH_CS_PIN 45 

//Font selection 
//BVS= Bitstream vera sans, suffix = font size in pixel (height)
#define BVS_13 10
#define BVS_15 12
#define BVS_19 14
#define BVS_22 18
#define BVS_28 22
#define BVS_34 28
#define BVS_43 38
#define BVS_52 53
#define BVS_74 78
#define BVS_112 122

#include "arduino.h"

//
//
extern uint8_t SmallFont[];

//UTFT myGLCD(CTE36,38,39,40,41);   // Remember to change the model parameter to suit your display module!
UTFT myGLCD(ILI9481,38,39,40,41);   // Remember to change the model parameter to suit your display module!

void setup()
{
  delay(1000);
  myGLCD.SPI_Flash_init(FLASH_CS_PIN);
  SPI.begin();
  //sbi(SPCR,SPR0);   sbi(SPCR,SPR1); // SPI Speed, For compatibility, uncomment this, For Speed, comment this
  Serial1.begin(9600);
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  //myGLCD.Send_Flash_information_to_UART(); //This will send Flash and related information to UART1
}


void loop()
{

  myGLCD.show_color_bar();
  delay(1000);
  
  myGLCD.clrScr();
  myGLCD.Load_image(0,0,237);
  myGLCD.setColor(255, 222, 111);  //Set font color
  myGLCD.setBackColor(0, 0, 0);  //Set background color
  myGLCD.Set_character_spacing(1); 
  myGLCD.Put_Text("Coldtears electronics",5,210,BVS_15);
  myGLCD.Put_Text("TFT LCD w/Font IC Demo",5,226,BVS_13);
  delay(2000);
  
  myGLCD.clrScr();
  myGLCD.Set_character_spacing(3); 
  myGLCD.Put_Text("Coldtears electronics",10,5,BVS_34);
  myGLCD.Set_character_spacing(1); myGLCD.
  setColor(166, 256, 211);  myGLCD.setBackColor(0, 255, 0);
  myGLCD.Put_Text("3.6 TFT LCD Module w/Font IC Demo",10,40,BVS_22);
  myGLCD.setColor(255, 255, 255);  myGLCD.setBackColor(0, 0, 0); 
  myGLCD.Put_Text("123456789",10,65,BVS_15);  
  myGLCD.setColor(0, 255, 255);
  myGLCD.Put_Text("abcdefg",10,80,BVS_34);   
  myGLCD.setColor(255, 255, 0);
  myGLCD.Put_Text("hijklmnopgrstuvwxy",10,115,BVS_43);
  myGLCD.setColor(255, 0, 255);
  myGLCD.Put_Text("123457890AB",10,165,BVS_74);
  
  delay(2000);
  myGLCD.clrScr();   
  myGLCD.setColor(255, 255, 255);
  static char buff[33];
  for(unsigned char temp=32; temp<65; temp++)  { buff[temp-32]=temp; }  buff[33]=0;
   myGLCD.Put_Text_array(buff,0,0,BVS_22);
  for(unsigned char temp=65; temp<92; temp++)  { buff[temp-65]=temp; }  buff[27]=0;
  myGLCD.Put_Text_array(buff,0,30,BVS_22); 
  for(unsigned char temp=92; temp<127; temp++)  { buff[temp-92]=temp; }  buff[33]=0;
  myGLCD.Put_Text_array(buff,0,60,BVS_22); 
  for(unsigned char temp=127; temp<154; temp++)  { buff[temp-127]=temp; }  buff[27]=0;
  myGLCD.Put_Text_array(buff,0,90,BVS_22);   
  for(unsigned char temp=154; temp<181; temp++)  { buff[temp-154]=temp; }  buff[27]=0;
  myGLCD.Put_Text_array(buff,0,120,BVS_22); 
   for(unsigned char temp=181; temp<204; temp++)  { buff[temp-181]=temp; }  buff[23]=0;
  myGLCD.Put_Text_array(buff,0,150,BVS_22);   
  for(unsigned char temp=204; temp<227; temp++)  { buff[temp-204]=temp; }  buff[23]=0;
  myGLCD.Put_Text_array(buff,0,180,BVS_22); 
   for(unsigned char temp=227; temp<255; temp++)  { buff[temp-227]=temp; }  buff[27]=0;
  myGLCD.Put_Text_array(buff,0,210,BVS_22);  
  delay(2000);

unsigned int k=5,location=1;
while(k>0)
{ 
  myGLCD.clrScr(); 
  for (unsigned temp1 =0; temp1<5; temp1++)
  {
    for (unsigned temp2 =0; temp2<9; temp2++)
    {
      myGLCD.Load_image(temp2*45,temp1*45,location); 
      location++; 
      if(location==236) break;
    }
  if(location==236) break; 
  }
  delay(2000);
  k--;
}

myGLCD.clrScr();
myGLCD.Put_Text("Countdown for LCD Display",0,0,BVS_22);
myGLCD.Put_Text("Demo by Henning Karlsen",0,25,BVS_22);
myGLCD.setColor(150, 222, 255);
k=30;
while(k>0)
{  
  k--;
  String test_string = String(k);
  test_string = "000"+ test_string + "       "; 
  myGLCD.Put_Text(test_string,30,60,BVS_112);
  int temp;
  if (k%3==2) temp=275;  else if (k%3==1) temp=277;  else temp=279;
  myGLCD.Load_image(50,175,temp); myGLCD.Load_image(150,175,temp+6);
}


// Default demo by Henning Karlsen Starts Here:

  int buf[398];
  int x, x2;
  int y, y2;
  int r;

  myGLCD.clrScr();
  
  myGLCD.setColor(255, 0, 0);
  myGLCD.fillRect(0, 0, 399, 13);
  myGLCD.setColor(64, 64, 64);
  myGLCD.fillRect(0, 226, 399, 239);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 0, 0);
  myGLCD.print("* Universal Color TFT Display Library *", CENTER, 1);
  myGLCD.setBackColor(64, 64, 64);
  myGLCD.setColor(255,255,0);
  myGLCD.print("<http://electronics.henningkarlsen.com>", CENTER, 227);
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRect(0, 14, 399, 225);

// Draw crosshairs
  myGLCD.setColor(0, 0, 255);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.drawLine(199, 15, 199, 224);
  myGLCD.drawLine(1, 119, 398, 119);
  for (int i=9; i<390; i+=10)
    myGLCD.drawLine(i, 117, i, 121);
  for (int i=19; i<220; i+=10)
    myGLCD.drawLine(197, i, 201, i);

// Draw sin-, cos- and tan-lines  
  myGLCD.setColor(0,255,255);
  myGLCD.print("Sin", 5, 15);
  for (int i=1; i<398; i++)
  {
    myGLCD.drawPixel(i,119+(sin(((i*0.9)*3.14)/180)*95));
  }
  
  myGLCD.setColor(255,0,0);
  myGLCD.print("Cos", 5, 27);
  for (int i=1; i<398; i++)
  {
    myGLCD.drawPixel(i,119+(cos(((i*0.9)*3.14)/180)*95));
  }

  myGLCD.setColor(255,255,0);
  myGLCD.print("Tan", 5, 39);
  for (int i=1; i<398; i++)
  {
    y=119+(tan(((i*0.9)*3.14)/180));
    if ((y>15) && (y<224))
    myGLCD.drawPixel(i,y);
  }

  delay(2000);

  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);
  myGLCD.setColor(0, 0, 255);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.drawLine(199, 15, 199, 224);
  myGLCD.drawLine(1, 119, 398, 119);

// Draw a moving sinewave
 x=1;
  for (int i=1; i<(398*20); i++) 
  {
    x++;
    if (x==399)
      x=1;
    if (i>399)
    {
      if ((x==199)||(buf[x-1]==119))
        myGLCD.setColor(0,0,255);
      else
        myGLCD.setColor(0,0,0);
      myGLCD.drawPixel(x,buf[x-1]);
    }
    myGLCD.setColor(0,255,255);
    y=119+(sin(((i)*3.14)/180)*(90-(i / 100)));
    myGLCD.drawPixel(x,y);
    buf[x-1]=y;
  }

  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

// Draw some filled rectangles
   for (int i=1; i<6; i++)
  {
    switch (i)
    {
      case 1:
        myGLCD.setColor(255,0,255);
        break;
      case 2:
        myGLCD.setColor(255,0,0);
        break;
      case 3:
        myGLCD.setColor(0,255,0);
        break;
      case 4:
        myGLCD.setColor(0,0,255);
        break;
      case 5:
        myGLCD.setColor(255,255,0);
        break;
    }
    myGLCD.fillRect(110+(i*20), 30+(i*20), 170+(i*20), 90+(i*20));
  }

  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);
  

// Draw some filled, rounded rectangles
  for (int i=1; i<6; i++)
  {
    switch (i)
    {
      case 1:
        myGLCD.setColor(255,0,255);
        break;
      case 2:
        myGLCD.setColor(255,0,0);
        break;
      case 3:
        myGLCD.setColor(0,255,0);
        break;
      case 4:
        myGLCD.setColor(0,0,255);
        break;
      case 5:
        myGLCD.setColor(255,255,0);
        break;
    }
    myGLCD.fillRoundRect(230-(i*20), 30+(i*20), 290-(i*20), 90+(i*20));
  }
  
  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

// Draw some filled circles
  for (int i=1; i<6; i++)
  {
    switch (i)
    {
      case 1:
        myGLCD.setColor(255,0,255);
        break;
      case 2:
        myGLCD.setColor(255,0,0);
        break;
      case 3:
        myGLCD.setColor(0,255,0);
        break;
      case 4:
        myGLCD.setColor(0,0,255);
        break;
      case 5:
        myGLCD.setColor(255,255,0);
        break;
    }
    myGLCD.fillCircle(110+(i*30),60+(i*20), 30);
  }
  
  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

// Draw some lines in a pattern
  myGLCD.setColor (255,0,0);
  for (int i=15; i<224; i+=5)
  {
    myGLCD.drawLine(1, i, (i*1.77)-10, 224);
  }
  myGLCD.setColor (255,0,0);
  for (int i=224; i>15; i-=5)
  {
    myGLCD.drawLine(398, i, (i*1.77)-11, 15);
  }
  myGLCD.setColor (0,255,255);
  for (int i=224; i>15; i-=5)
  {
    myGLCD.drawLine(1, i, 411-(i*1.77), 15);
  }
  myGLCD.setColor (0,255,255);
  for (int i=15; i<224; i+=5)
  {
    myGLCD.drawLine(398, i, 410-(i*1.77), 224);
  }
  
  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,225);

// Draw some random circles
  for (int i=0; i<100; i++)
  {
    myGLCD.setColor(random(255), random(255), random(255));
    x=32+random(336);
    y=45+random(146);
    r=random(30);
    myGLCD.drawCircle(x, y, r);
  }

  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

// Draw some random rectangles
  for (int i=0; i<100; i++)
  {
    myGLCD.setColor(random(255), random(255), random(255));
    x=2+random(396);
    y=16+random(207);
    x2=2+random(396);
    y2=16+random(207);
    myGLCD.drawRect(x, y, x2, y2);
  }

  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

// Draw some random rounded rectangles
  for (int i=0; i<100; i++)
  {
    myGLCD.setColor(random(255), random(255), random(255));
    x=2+random(396);
    y=16+random(207);
    x2=2+random(396);
    y2=16+random(207);
    myGLCD.drawRoundRect(x, y, x2, y2);
  }

  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

  for (int i=0; i<100; i++)
  {
    myGLCD.setColor(random(255), random(255), random(255));
    x=2+random(396);
    y=16+random(209);
    x2=2+random(396);
    y2=16+random(209);
    myGLCD.drawLine(x, y, x2, y2);
  }

  delay(2000);
  
  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(1,15,398,224);

  for (int i=0; i<10000; i++)
  {
    myGLCD.setColor(random(255), random(255), random(255));
    myGLCD.drawPixel(2+random(396), 16+random(209));
  }

  delay(2000);

  myGLCD.fillScr(0, 0, 255);
  myGLCD.setColor(255, 0, 0);
  myGLCD.fillRoundRect(120, 70, 279, 169);
  
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 0, 0);
  myGLCD.print("That's it!", CENTER, 93);
  myGLCD.print("Restarting in a", CENTER, 119);
  myGLCD.print("few seconds...", CENTER, 132);
  
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 255);
  myGLCD.print("Runtime: (msecs)", CENTER, 210);
  myGLCD.printNumI(millis(), CENTER, 225);
  
  delay (1000);
  myGLCD.clrScr();
}



