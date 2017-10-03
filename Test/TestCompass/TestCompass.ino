/* Author = helscream (Omer Ikram ul Haq)
Last edit date = 2014-06-22
Website: http://hobbylogs.me.pn/?p=17
Location: Pakistan
Ver: 0.1 beta --- Start
Ver: 0.2 beta --- Debug feature included
*/
///////////////////////////////////////////////
// INCLUDE HARDWARE MINIMO					///
// Chiamare  initRobotBaseHW();				///
///////////////////////////////////////////////
#include <MyRobotLibs\hw_config\hw_config.h>
///////////////////////////////////////////////
#include <MyRobotLibs\msgCore\msgCore.h>

#include <Wire.h>
//#include <compass\compass.h>

//#include <HMC5883L/HMC5883L.h>//
//HMC5883L compass; //MyCompass_c compass;

#include <MyRobotLibs\MyCompass\MyCompass.h>		
MyCompass_c compass;

//#include <MyRobotLibs\robot.h>

#define Task_t 10          // Task Time in milli seconds

int dt=0;
unsigned long t;
// Main code -----------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  Serial.print("Setting up I2C ........\n");
  Wire.begin();
  compass.compass_x_offset = 122.17;
  compass.compass_y_offset = 230.08;
  compass.compass_z_offset = 389.85;
  compass.compass_x_gainError = 1.12;
  compass.compass_y_gainError = 1.13;
  compass.compass_z_gainError = 1.03;
  
  
  
  compass.begin(1);  // original gain =2
  compass.compass_debug = 1;
//  compass.compass_offset_calibration(3);


}

// Main loop 
// Main loop -----------------------------------------------------------------
void loop(){
  
  t = millis();
 
  float load;
 
  
  
  compass.compass_scalled_reading();
  
  Serial.print("\nx = ");
  Serial.println(compass.compass_x_scalled);
  Serial.print("y = ");
  Serial.println(compass.compass_y_scalled);
  Serial.print("z = ");
  Serial.println(compass.compass_z_scalled);
  

  compass.getBearing();
  Serial.print ("Heading angle = ");
  Serial.print (compass.bearing);
  Serial.println(" Degree");
  
  dt = millis()-t; //mediamente 3mS
  Serial.print ("Load on processor ms= ");
  Serial.println(dt);
   

  
  delay(1000);
  
  
}











