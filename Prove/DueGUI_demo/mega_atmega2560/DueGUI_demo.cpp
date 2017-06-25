//  (c) 2013 Darren Hill (Cowasaki)
//
#define VERSION "0.13"

#include "arduino.h"

void TC3_Handler();
//
//
extern uint8_t SmallFont[];
#include <UTFT.h>
#include <UTouch.h>
#include <DUEGUI.h>
//#include "SPI.h"

#define PIN_CS 10
#define PIN_DC 9
#define PIN_RESET 8

UTFT tft(ILI9341_S5P, MOSI, SCK, PIN_CS, PIN_RESET, PIN_DC);   // Remember to change the model parameter to suit your display module!

																  ///UTFT          myGLCD(ITDB32S,25,26,27,28); // CTE Shield!
																  //UTouch        myTouch(6,5,32,3,2);         // CTE Shield!    
UTouch        myTouch(30, 31, 32, 33, 34);         // CTE Shield!    

DUEGUI DueGUI(&tft, &myTouch);
 //
//  Setup for Timer
//
#define ticksPerSecond 20
#define URNnull 0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Setup for the external RTC library
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#include <rtc_clock.h>
RTC_clock rtc_clock(RC);
char* daynames[] = {
	"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
int hh, mm, ss, dow, dd, mon, yyyy;

//Setup variables for GUI
#define maxbuttons 100
#define popupbuttonstart 50
#define stateDrawn 1
#define statePressed 2

int found;
int clockDigital,clockDate,clockAnalogue,background,pnlTitle;
int btnTB1,btnTB2,btnRemove,btnAdd,btnRefresh;

//
//  Interupt setup
//
volatile int l;
void TC3_Handler()
{
  //
  // We don't want timer called whilst it's being handled.
  //
  DueGUI.stopTimer(TC3_IRQn);
  //
  // routine is called several times per second ie "ticksPerSecond".
  // l is incremented each time and by checking this against "ticksPerSecond"
  // we can make sure that once a second events are only called once per second.
  //
  l+=1;
  //
  // This is the "once per second" event routine.
  //
  if (l==ticksPerSecond){
    // second passed
    l=0;
    if (DueGUI.anyClockVisible) {
      rtc_clock.get_time(&hh,&mm,&ss);
      rtc_clock.get_date(&dow,&yyyy,&mon,&dd);
      DueGUI.setObjectTime(clockDigital,hh,mm,ss);
      DueGUI.setObjectDate(clockDate,dd,mon,yyyy,true,true);
      DueGUI.setObjectTime(clockAnalogue,hh,mm,ss);
      DueGUI.redrawObject(clockDigital);
      DueGUI.redrawObject(clockDate);
      DueGUI.drawHands(clockAnalogue);
    }
  }
  //
  // Ok interupt handler is finished. 
  //
  TC_GetStatus(TC1, 0);
  DueGUI.restartTimer(TC3_IRQn);
}

void setup(){ 
  //
  // Initialise serial on the programming port
  //
  Serial.begin(115200);
  Serial.println("\n\nSERIAL CONNECTED AT 115200\n\n");

  tft.InitLCD();
  tft.clrScr();
  tft.setColor(255, 0, 0);
  tft.fillRect(0, 0, 319, 13);
  tft.setColor(0, 0, 64);
  tft.fillRect(0, 226, 319, 239);
  tft.setColor(255, 255, 255);
  tft.setBackColor(255, 0, 0);

  tft.print("UTFT  Library", 0,0, 0);
  delay(2000);

  DueGUI.db_St(1, "\n\nSERIAL CONNECTED AT 115200\nCompiled at: ");
  DueGUI.db_St(1, __TIME__);
  DueGUI.db_St(1, " on: ");
  DueGUI.db_St(1, __DATE__);
  DueGUI.db_St(1, "\nVersion number: ");
  DueGUI.db_St(1, VERSION);
  DueGUI.db_St(1, "");


  //
  // Initialise for RTC
  //
  rtc_clock.init();
  rtc_clock.set_time(__TIME__);
  rtc_clock.set_date(__DATE__);
  
  //
  // Initialise TFT display & Touch, clear screen and select font
  //
  delay(500);
  DueGUI.InitLCD();
  DueGUI.clrScr();
  DueGUI.setFont(SmallFont);
///  DueGUI.SPI_Flash_init(52,2);

  ///DueGUI.UTouch(6,5,32,3,2);
  ///DueGUI.InitTouch();
 /// DueGUI.setPrecision(PREC_MEDIUM);
 
  //background= DueGUI.addImage(0,0,287,true,1);
  //pnlTitle=DueGUI.addPanel(0,0,300,200,0x0000FF,0xFFFFFF,0xFFFFFF,2,"Board   control   menu",280,8,BVS_28,true, 1);
 
  ///int DUEGUI::addButton(word x,word y,word xs,word ys,long colour,long borcolour,long textcolour,long presscolour,long presstextcolour,byte borwidth,String top,word xo,word yo,int font,bool visible,int URN)
  btnRefresh=DueGUI.addButton(5,5,50,30,0x0000FF,0xFFFFFF,0xFFFFFF,0xFF0000,0xFFFFFF,2,"XX",20,12,BVS_28,true,1);
 
 /*
  btnTB1=DueGUI.addButton(50,100,249,50,0x0000FF,0xFFFFFF,0xFFFFFF,0xFF0000,0xFFFFFF,2,"Test button 1",50,12,BVS_28,true,1);
  btnTB2=DueGUI.addButton(50,175,249,50,0x0000FF,0xFFFFFF,0xFFFFFF,0xFF0000,0xFFFFFF,2,"Test button 2",50,12,BVS_28,true,1);
  btnAdd=DueGUI.addButton(50,250,249,50,0x00FFFF,0xFFFFFF,0x000000,0xFF0000,0xFFFFFF,2,"Bring back 1&2",50,12,BVS_28,false,1);
  btnRemove=DueGUI.addButton(50,350,249,50,0x00FF00,0xFFFFFF,0x000000,0xFF0000,0xFFFFFF,2,"Remove  1&2",50,12,BVS_28,true,1);
  clockDigital=DueGUI.addDigitalClock_Time(681,11,0x0000FF,0xFFFFFF,0,BVS_34,true, 1);
  clockDate   =DueGUI.addDigitalClock_Date( 15,11,0x0000FF,0xFFFFFF,0,BVS_34,true, 1);
 
                   //addAnalogueClock(x  ,y  ,clocksize,centresize,facecolour,borcolour,hourcolour,hourlen,mincolour,minlen,seccolour,seclen,borwidth,options,visible){
  clockAnalogue=DueGUI.addAnalogueClock(500,225,270      ,15        ,0x000000  ,0xFFFF00 ,0xFF0000  ,80     ,0x00FF00 ,90    ,0xFFFFFF ,100   ,15      ,24     ,true, 1);
*/     
  DueGUI.redrawAllObjects();
  
  //
  // Start timer
  //
 // DueGUI.startTimer(TC1,0,TC3_IRQn,ticksPerSecond);
  
 }


void loop(){
  //
  //  Main program control loop
  //
	DueGUI.redrawAllObjects();
/*
  found=DueGUI.checkAllButtons();
  
  if (found==btnRefresh){
    Serial.println("Refresh");
    DueGUI.redrawAllObjects();
  }
  
  if (found==btnTB1){
    Serial.println("btn TB1");
  }
  
  if (found==btnTB2){
    Serial.println("btn TB2");
  }
  
  if (found==btnRemove){
    Serial.println("btn Remove");
    DueGUI.makeObjectInvisible(btnRemove,true); 
    DueGUI.makeObjectInvisible(btnTB1,true);
    DueGUI.makeObjectInvisible(btnTB2,true); 
    DueGUI.makeObjectVisible(btnAdd,true); 
  }

  if (found==btnAdd){
    Serial.println("btn Add");
    DueGUI.makeObjectInvisible(btnAdd,true); 
    DueGUI.makeObjectVisible(btnTB1,true);
    DueGUI.makeObjectVisible(btnTB2,true); 
    DueGUI.makeObjectVisible(btnRemove,true); 
  }
  */
}




