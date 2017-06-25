//********************************************************\
//Developers:    James Callender, Jovan Munroe
//Date:          January 23, 2014
//Bot Type:      Land Rover
//Motor Config:  4WD, Tank Control
//Discription:   MavLink Communication via CommChannel with the
//               Arduino Mega.
//********************************************************\            
//#include <Adafruit_CC3000.h>
//#include <FastSerial.h>			//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
//#include <ccspi.h>
//#include <SPI.h>
#include <math.h>
#include <timerone.h>
//#include <C:\Users\Luca\Documents\Arduino\SW\libraries\GCS_MAVLink\GCS_MAVLink.h>
#include <C:\Users\Luca\Documents\Arduino\SW\libraries\GCS_MAVLink\include\mavlink\v1.0\common\mavlink.h>
//#include	<GCS_MAVLink\include\mavlink\v1.0\ardupilotmega\mavlink.h>	//	 <mavlink\include\common\mavlink.h>
//#include <mavlink\include\mavlink.h>
#include <Servo.h>
#include <inttypes.h>
#define CommChannel Serial
#define MonitorChannel Serial

//ADAFRUIT DEFINITIONS
//-------------------- 

char packetBuffer[] = "acknowledged";

const unsigned long
connectTimeout  = 15L * 1000L, // Max time to wait for server connection
responseTimeout = 15L * 1000L; // Max time to wait for data from server

int cc;

//GPS SETUP
#define GPSECHO  true
//TinyGPSPlus GPS();
boolean usingInterrupt = false;
void useInterrupt(boolean);
int32_t Latitude;
int32_t Longitude;
int32_t Altitude;
int16_t Velocity;
int64_t Microseconds;

/************************************************************************************************************/

/**DEFINE ROBOT ID HERE**/
#define BOT_ID  22
//Servo motor;

//Rover 5 Pin Assignments
//Channel 1
#define CH1_PWM 2
#define CH1_DIR 48
//Channel 2
#define CH2_PWM 4
#define CH2_DIR 49
//Channel 3
#define CH3_PWM 7 
#define CH3_DIR 50
//Channel 4
#define CH4_PWM 8
#define CH4_DIR 51

//MavLink Fields
uint8_t system_type = MAV_TYPE_GROUND_ROVER; //MAV_TYPE_HELICOPTER;//MAV_TYPE_FIXED_WING;
uint8_t autopilot_type =MAV_AUTOPILOT_ARDUPILOTMEGA ; //MAV_AUTOPILOT_ARDUPILOTMEGA MAV_AUTOPILOT_GENERIC
uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; //MAV_MODE_MANUAL_ARMED; //MAV_MODE_GUIDED_ARMED //MAV_MODE_GUIDED_DISARMED
uint8_t system_state = MAV_STATE_ACTIVE;

// Initialize the required buffers 
mavlink_message_t receivedMsg; 
mavlink_message_t heartbeatMsg;
mavlink_message_t gpsMsg;
mavlink_manual_control_t manual_control;
mavlink_set_mode_t mode;
mavlink_heartbeat_t heartbeat;
mavlink_status_t mav_status;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t buf0[MAVLINK_MAX_PACKET_LEN]; 
mavlink_system_t mavlink_system;
float position[6];
bool ManualMode = false;

//Helper Fields
int16_t YControl = 0;
int16_t XControl = 0;
int DEADZONE = 100;
bool MovingBackward = false;

//Initialize Timers
unsigned long heartbeatTimer = millis();
unsigned long gpsTimer = millis();
unsigned long heartbeatInterval = 0.5L * 1000L;
unsigned long gpsInterval = 2L * 1000L;

//FastSerialPort0(Serial);

void comm_receive(); //Read Message



void setup() {
  
  //GPS SETUP

  pinMode(13, OUTPUT);
  Serial.begin(115200);

  //GPS.begin(9600);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(false);
  delay(1000);
  //mySerial.println(PMTK_Q_RELEASE);
  
  ////Rover 5: Setup Pins
  //pinMode(CH1_PWM, OUTPUT);
  //pinMode(CH1_DIR, OUTPUT);
  //pinMode(CH2_PWM, OUTPUT);
  //pinMode(CH2_DIR, OUTPUT);
  //pinMode(CH3_PWM, OUTPUT);
  //pinMode(CH3_DIR, OUTPUT);
  //pinMode(CH4_PWM, OUTPUT);
  //pinMode(CH4_DIR, OUTPUT);

//  displayDriverMode();

 

  //uint16_t firmware = checkFirmwareVersion();
  //if ((firmware != 0x113) && (firmware != 0x118)) {
  //  Serial.println(F("Wrong firmware version!"));
  //  for(;;);
  //}



  //Initial heartbeat
  heartbeat.base_mode = MAV_MODE_GUIDED_DISARMED; //MAV_MODE_MANUAL_DISARMED;
  heartbeat.custom_mode = 0;
  heartbeat.system_status = MAV_STATE_STANDBY;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
//SIGNAL(TIMER0_COMPA_vect) {
//  char c = GPS.read();
//   if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) UDR0 = c;  
//     writing direct to UDR0 is much much faster than Serial.print 
//     but only one character can be written at a time. 
//}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}



void Hearthbeat(){
		mavlink_system.sysid = BOT_ID;
		mavlink_system.compid = MAV_COMP_ID_IMU;
    
		// Pack the message 
		mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, system_type, autopilot_type, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
  
		// Copy the message to send buffer 
		uint16_t len = mavlink_msg_to_send_buffer(buf, &heartbeatMsg);
  
		//Write Message    
		CommChannel.write(buf, len);


		digitalWrite(13,1);
			delay(100);
		digitalWrite(13,0);
}


void loop() {
  
	//Reset Buffer
//	memset(buf, 0xFF, sizeof(buf));
  
	//Pack and send heartbeat at specific interval 
	//if((millis() - heartbeatTimer) > heartbeatInterval)
	//{
		Hearthbeat();
		heartbeatTimer = millis();
		Serial.println("Heartbeat");  
	//}
  
/* 
	// Pack and send GPS reading at the set interval
  if((millis() - gpsTimer) > gpsInterval)
  {
    gpsTimer = millis(); // reset the timer     
    
    // Update GPS reading if new reading is available
    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA()))
    { 
      prepareNewGPSMessage();      
      //Serial.print("Location: ");     
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print(Latitude, DEC);
      Serial.print(", "); 
      Serial.print(Longitude, DEC);
      Serial.print(", "); 
      Serial.print(Altitude, DEC);
      Serial.print(", "); 
      Serial.println(Velocity, DEC);
    }
    
    // As long as we have a fix, proceed with packing and sending GPS data
    if(GPS.fix)
    {
     mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &gpsMsg,
			             Microseconds, 3, Latitude, Longitude, Altitude, 0xFFFF, 0xFFFF, Velocity, 0xFFFF, GPS.satellites); //fix_type must be 3 for some odd reason
  
        /// Copy the message to send buffer
        uint16_t len = mavlink_msg_to_send_buffer(buf, &gpsMsg);
  
        //Write Message    
        CommChannel.write(buf, len);
    }


  }
*/

          //receive data over serial 

		  
//	memset(buf, 0xFF, sizeof(buf));//Reset Buffer
  
	comm_receive(); //Read Message

	
}

/*
///*****************************************************************************
/// Name: prepareNewGPSMessage
/// Type: Helper function
/// Parameters:  None
/// Description: Get GPS data into the format required by MavLink/QGroundControl
///*****************************************************************************
void prepareNewGPSMessage()
{
  float tempLat = 0.0;
  float tempLong = 0.0;
  
  tempLat = GPS.latitude * 10000.0;
  while((int)(log10(tempLat) + 1) < 9)
  {
    tempLat = tempLat * 10.0;
  }
  if(GPS.lat == 'S')
  {
    tempLat = tempLat * -1.0;
  }
    
  tempLong = GPS.longitude * 10000.0;
  while((int)(log10(tempLong) + 1) < 9)
  {
    tempLong = tempLong * 10.0;
  }
  if(GPS.lon == 'W')
  {
    tempLong = tempLong * -1.0;
  }
  
  Altitude = GPS.altitude * 1000;
  Velocity = GPS.speed * 0.5144444 * 100; //knots to m/s * 100
  Microseconds = GPS.milliseconds * 1000;
}
*/


///*****************************************************************************
/// Name: printXY
/// Type: Helper function
/// Parameters:  X and Y coordinates to print
/// Description: Print X and Y coordinates to the serial monitor. Used for
///              debugging (especially with the manual control messages).
///*****************************************************************************
void printXY(float X, float Y)
{
      MonitorChannel.print("(");
      MonitorChannel.print(X, DEC);
      MonitorChannel.print(",");
      MonitorChannel.print(Y, DEC);
      MonitorChannel.print(")");
      MonitorChannel.print("\n");
}


///*****************************************************************************
/// Name: printXY
/// Type: Helper function
/// Parameters:  X and Y position of the joystick 
/// Description: Move the robot according to the coordinates of the user's
///              joystick. (Need to optimize this)
///*****************************************************************************
void moveBot(int X, int Y)
{
  //zero point turn if within deadzone
  if (abs(Y) <= DEADZONE)
  {
    if (X < (-1 * DEADZONE))//left- zero point
    {
      MonitorChannel.println("Zero Point Turn Left");
      //Left Wheels
      directMotor(1, abs(X), 0); //motor 1, move at speed of turn opposite direction
      directMotor(2, abs(X), 0); //motor 2, move at speed of turn opposite direction
      //Right Wheels
      directMotor(3, abs(X), 1); //motor 3, move at speed of turn
      directMotor(4, abs(X), 1); //motor 4, move at speed of turn
    }
    else if (X > DEADZONE) //right- zero point
    {
      MonitorChannel.println("Zero Point Turn Right");
      //Left Wheels
      directMotor(1, abs(X), 1); //motor 1, move at speed of turn
      directMotor(2, abs(X), 1); //motor 2, move at speed of turn
      //Right Wheels
      directMotor(3, abs(X), 0); //motor 3, hold
      directMotor(4, abs(X), 0); //motor 4, hold
    }
    else//stop
    {
      MonitorChannel.println("STOPPED!");
      //Left Wheels
      directMotor(1, 0, 0); //motor 1, hold
      directMotor(2, 0, 0); //motor 2, hold
      //Right Wheels
      directMotor(3, 0, 0); //motor 3, hold
      directMotor(4, 0, 0); //motor 4, hold
    }
  }

  //forward
  else if (Y > DEADZONE)
  {
    if (X < (-1 * DEADZONE))// forward left
    {
      MonitorChannel.println("forward left");
      if(abs(X) < abs(Y))
      {
         //Left Wheels
         directMotor(1, abs(Y)-abs(X), 1); //motor 1, at +X speed
         directMotor(2, abs(Y)-abs(X), 1); //motor 2, at +X speed
         //Right Wheels
         directMotor(3, Y, 1); //motor 3, at +X speed
         directMotor(4, Y, 1); //motor 4, at +X speed
      }
      else
      {
        //Left Wheels
        directMotor(1, abs(Y), 1); //motor 1, at +X speed
        directMotor(2, abs(Y), 1); //motor 2, at +X speed
        //Right Wheels
        directMotor(3, abs(X), 1); //motor 3, at +Y speed
        directMotor(4, abs(X), 1); //motor 4, at +Y speed
      }
    }
    else if (X > DEADZONE) //forward right
    {
      Serial.println("forward right");
      if(abs(X) < abs(Y))
      {
         //Left Wheels
         directMotor(1, Y, 1); //motor 1, at +X speed
         directMotor(2, Y, 1); //motor 2, at +X speed
         //Right Wheels
         directMotor(3, abs(Y)-abs(X), 1); //motor 3, at +X speed
         directMotor(4, abs(Y)-abs(X), 1); //motor 4, at +X speed
      }
      else
      {
        //Left Wheels
        directMotor(1, abs(X), 1); //motor 1, at +X speed
        directMotor(2, abs(X), 1); //motor 2, at +X speed
        //Right Wheels
        directMotor(3, abs(Y), 1); //motor 2, at +Y speed
        directMotor(4, abs(Y), 1); //motor 2, at +Y speed
      }
    }
    else //straight forward
    {     
      MonitorChannel.println("Forward");  
      //Left Wheels
      directMotor(1, abs(Y), 1); //motor 1, at +Y speed
      directMotor(2, abs(Y), 1); //motor 2, at +Y speed
      //Right Wheels
      directMotor(3, abs(Y), 1); //motor 3, at +Y speed
      directMotor(4, abs(Y), 1); //motor 4, at +Y speed
    }
  }

  //backwards
  else if (Y < (-1* DEADZONE))
  {
     if (X < (-1 * DEADZONE))// backward left
    {
      Serial.println("backward left");
      if(abs(X) < abs(Y))
      {
        //Left Wheels
        directMotor(1, Y, 0); //motor 1, at +X speed
        directMotor(1, Y, 0); //motor 2, at +X speed
        //Right Wheels
        directMotor(3, abs(Y)-abs(X), 1); //motor 3, at +X speed
        directMotor(4, abs(Y)-abs(X), 1); //motor 4, at +X speed
      }
      else
      {
        //Left Wheels
        directMotor(1, abs(X), 0); //motor 1, at +X speed
        directMotor(2, abs(X), 0); //motor 2, at +X speed
        //Right Wheels
        directMotor(3, abs(Y), 0); //motor 2, at +Y speed
        directMotor(4, abs(Y), 0); //motor 2, at +Y speed
      }
    }
    else if (X > DEADZONE) //forward right
    {
      MonitorChannel.println("backward right");
      if(abs(X) < abs(Y))
      {
        //Left Wheels
        directMotor(1, abs(Y)-abs(X), 0); //motor 1, at +X speed
        directMotor(2, abs(Y)-abs(X), 0); //motor 2, at +X speed
        //Right Wheels
        directMotor(3, Y, 0); //motor 3, at +X speed
        directMotor(4, Y, 0); //motor 4, at +X speed
      }
      else
      {
        //Left Wheels
        directMotor(1, abs(Y), 0); //motor 1, at -Y speed
        directMotor(2, abs(Y), 0); //motor 2, at -Y speed
        //Right Wheels
        directMotor(3, abs(X), 0); //motor 3, at -X speed
        directMotor(4, abs(X), 0); //motor 4, at -X speed
      }
    }
    else //straight forward
    {     
      MonitorChannel.println("Backward");
      //Left Wheels
      directMotor(1, abs(Y), 0); //motor 1, at -Y speed
      directMotor(2, abs(Y), 0); //motor 2, at -Y speed
      //Right Wheels
      directMotor(3, abs(Y), 0); //motor 3, at -Y speed
      directMotor(4, abs(Y), 0); //motor 4, at -Y speed
      
    }    
  }
}

///*****************************************************************************
/// Name: directMotor
/// Type: Helper fuction
/// Parameters:  The channel to control, the speed and direction to send. 
/// Description: Displays the driver mode (tiny of normal), and the buffer size
///              if tiny mode is not being used. The buffer size and driver mode
///              are defined in cc3000_common.h
///*****************************************************************************
void directMotor(int channel, int speed, int direction){
  if(channel == 1){
    digitalWrite(CH1_DIR, direction);
    analogWrite(CH1_PWM, speed);
  }else if(channel == 2){
    digitalWrite(CH2_DIR, direction);
    analogWrite(CH2_PWM, speed);
  }else if(channel == 3){
    digitalWrite(CH3_DIR, direction);
    analogWrite(CH3_PWM, speed);
  }else if(channel == 4){
    digitalWrite(CH4_DIR, direction);
    analogWrite(CH4_PWM, speed);
  }
}





 void comm_receive() {
//       mavlink_message_t receivedMsg;  //sono globali
//	mavlink_status_t mav_status;


	while(CommChannel.available() > 0 ) 
	{
		uint8_t c = CommChannel.read();
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &receivedMsg, &mav_status)) {

			CommChannel.print("  Sys ID: ");
			CommChannel.print(receivedMsg.sysid, DEC);
			CommChannel.print("  Comp ID: ");
			CommChannel.print(receivedMsg.compid, DEC);
			CommChannel.print("  Len ID: ");
			CommChannel.print(receivedMsg.len, DEC);
			CommChannel.print("  Msg ID: ");
			CommChannel.print(receivedMsg.msgid, DEC);
			CommChannel.print("\n");

			// Handle message--------------------------------------------------------
 

			switch(receivedMsg.msgid)
			{   
				//case MAVLINK_MSG_ID_ACTION: 
				//		// EXECUTE ACTION 
				//	break; 

		//      case MAVLINK_MSG_ID_MANUAL_CONTROL: //[IMPORTANT: TEMPORARY FIX EXPLAINED BELOW]
		//           mavlink_msg_manual_control_decode(&receivedMsg, &manual_control);
		//           Serial.print("Manual Control (Thrust, Pitch): ");
		//           printXY(manual_control.thrust, manual_control.pitch);
		//           Serial.print("\n ");
		//           printXY(manual_control.thrust_manual, manual_control.pitch_manual);
		//           Serial.print("\n");
      
				  //Direction
		//          XControl = (int16_t)(buf[6] | (buf[7] << 8));
		//          XControl = map(XControl, -1000, 1000, -255, 255);
		//          
		//          //Process Forward and Backward Motion
		//          YControl = (int16_t)(buf[10] | (buf[11] << 8));
		//          YControl = map(YControl, 0, 1000, -255, 255);
		//          
		//          moveBot(XControl , YControl);
		//          break;
         
				case MAVLINK_MSG_ID_SET_MODE: //Get new base mode
					{
					   mavlink_msg_set_mode_decode(&receivedMsg, &mode);
					   Serial.print("Target System: ");
					   Serial.print(mode.target_system);
					   Serial.print("\n");
					   Serial.print("New Base Mode: ");
					   Serial.print(mode.base_mode);
					   heartbeat.base_mode = mode.base_mode;
					   Serial.print("\n");
					   Serial.print("New Custom Mode: ");
					   Serial.print(mode.custom_mode);
					   heartbeat.custom_mode = mode.custom_mode;
					   Serial.print("\n");
						break;   
					}
				case MAVLINK_MSG_ID_HEARTBEAT: //Get new heartbeat 
											 //[IMPORTANT: Mavlink C++ generator decodes the heartbeat incorrectly (parameters out of order)]
		           mavlink_msg_heartbeat_decode(&receivedMsg, &heartbeat);
		           Serial.println("heartbeat Received");
		           Serial.print("New Custom Mode: ");
		           Serial.print(heartbeat.custom_mode);
		           Serial.print("\n");
		           Serial.print("New Type: ");
		           Serial.print(heartbeat.type);
		           Serial.print("\n");
		           Serial.print("New Autopilot: ");
		           Serial.print(heartbeat.autopilot);
		           Serial.print("\n");
		           Serial.print("New Base Mode: ");
		           Serial.print(heartbeat.base_mode);
		           Serial.print("\n");
		           Serial.print("New System Status: ");
		           Serial.print(heartbeat.system_status);
		           Serial.print("\n");
		           Serial.print("New Mavlink Version: ");
		           Serial.print(heartbeat.mavlink_version);
		           Serial.print("\n");
				break;
       
				case MAVLINK_MSG_ID_COMMAND_LONG:
				// EXECUTE ACTION
					break;

			  case MAVLINK_MSG_ID_MANUAL_CONTROL: //[IMPORTANT: TEMPORARY FIX EXPLAINED BELOW]



				   mavlink_msg_manual_control_decode(&receivedMsg, &manual_control);
				   Serial.print("Manual Control (x, y): ");
				   printXY(manual_control.x, manual_control.y);
				   //printXY(manual_control.thrust_manual, manual_control.pitch_manual);
				   Serial.print("\n");
      
				  //Direction
				  //XControl = (int16_t)(buf[6] | (buf[7] << 8));
				  XControl = map(manual_control.x, -1000, 1000, -255, 255);
          
				  //Process Forward and Backward Motion
				  //YControl = (int16_t)(buf[10] | (buf[11] << 8));
				  YControl = map(manual_control.y, 0, 1000, -255, 255);
          
				  moveBot(XControl , YControl);
				  break;


				default: 
						//Do nothing 
					break; 	


			}
			// end Handle message-----------------------------------------------


			  //mavlink_parse_char will not catch manual control messsages so we have to handle these separately for now when necessary 
			  if((heartbeat.base_mode == MAV_MODE_GUIDED_ARMED || heartbeat.base_mode == MAV_MODE_MANUAL_ARMED) && buf[0] == MAVLINK_MSG_ID_DEBUG)
			  {
				switch(buf[5]) //Need to look in buffer since we are not able to decode manual control messages
				{   
					case MAVLINK_MSG_ID_MANUAL_CONTROL:            
						//(X,Y) 
						XControl = (int16_t)(buf[6] | (buf[7] << 8));
						YControl = (int16_t)(buf[10] | (buf[11] << 8));
                                
						XControl = map(XControl, -1000, 1000, -255, 255);
						YControl = map(YControl, -1000, 1000, -255, 255);
                
						moveBot(XControl , YControl);
					break;
            
					//HERE: Add cases for other messages that we cannot decode using mavlink_parse_char
				}
			  } 


		}	//end if mavlink_parse_char
 
		// And get the next one
	} // end while 

   



  }