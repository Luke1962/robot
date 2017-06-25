// ============================================================================================
// ===																						===
// ===       LIBRERIE																		===
// ===		Aggiungere ciascun percorso nelle propriet� del progetto in Visual Studio 		===
// ===		Configuration Properties >C++ > Path											===
// ============================================================================================
#define SERIAL_BAUD_RATE 9600
 
#include <Dbg.h>

#include <MemoryFree.h>
#include <FlexiTimer2.h>	
#include <FlexiTimer2\FlexiTimer2.h>  //#include <TimerThree/TimerThree.h>
//#include <encoder/Encoder.h>
#include <robot.h>  //#include <robot/robot.h>

robot_c robot;
// ------------------ M A I N  ----------------------



unsigned delayms=2000;
int cm =20;
double CmPercorsi=0.0;

// Setup function
void setup() 
{
	Serial.begin(SERIAL_BAUD_RATE); // Listen on serial connection for messages from the pc
	delay(1000); 
	dbg( "1,Starting TestRobotMoterEncoders.ino in sec ...;", 5 );
	delay(5000); 
	
				attachInterrupt(Pin_EncRa, robot_c.ISRencR, CHANGE);
				attachInterrupt( Pin_EncLa, robot.ISRencL, CHANGE );

	
	dbg("Free memory= %d",freeMemory());
	 	while( !robot.isPowerOn()){
		Serial.println("Accendi main power....");
		delay(1000);
	}
}

//=========================================================================================
// Loop function
//=========================================================================================
void loop() 
{
	
	while (1)
	{
		interrupts();
		dbg( "1,EncL %d; ", (int)robot..encL.position );
		delay(2000);
	}
/*
	CmPercorsi=(double)robot.moveCm(cm);
	dbg("CmPercorsi= %d ",(int)CmPercorsi);		//dbg("CmPercorsi= %0.2f",CmPercorsi);
	delay(delayms);
	CmPercorsi=(double)robot.moveCm(-cm);
	dbg("CmPercorsi= %d ",(int)CmPercorsi);
	delay(delayms);
	
	*/
}
