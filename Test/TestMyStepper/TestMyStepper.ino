// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_BAUD_RATE 115200
//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
//#define delay(ms) chThdSleepMilliseconds(ms) 

#include <MyRobotLibs\dbg.h>

#include <MyRobotLibs\systemConfig.h>
#include <MyRobotLibs\hw_config.h>

#pragma endregion
// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIBRERIE
 #include <digitalWriteFast.h>

#include <Arduino.h>	//per AttachInterrupt

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
 
 #include <robot.h>
  
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{

    robot.status.cmd.stepsDone++;
    //digitalWriteFast(Pin_LED_TOP_B, !digitalReadFast(Pin_LED_TOP_B));
}
//Dichiarazione di funzione che punta all’indirizzo zero

 
 #include <FlexiTimer2\FlexiTimer2.h>
//#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
#include <avr/wdt.h>
#include <TinyGPSplus/TinyGPS++.h>

 


int 	 targetDistCm = 50;
float cmPercorsi = 0.0;

//void toggle(byte pinNum) { 
//  byte pinState = !digitalRead(pinNum);
//  digitalWrite(pinNum, pinState); 
//}

//long newLeft, newRight;
void countDown(int seconds) {
    MSG3("CountDown in ", seconds, " sec...");
    for (size_t i = seconds; i > 0; i--)
    {
        MSG2("  -", i);
        delay(1000);
    }
}

void setup() 
{
    countDown(15);
 
  Serial.begin(SERIAL_BAUD_RATE); // Listen on serial connection for messages from the pc
   Serial.println("TestMyStepper : controllo motori stepper con encoder");
   Serial.println("va avanti e indietro ciclicamente usando gli encoder");
   //  ck =robot.currCommand.clock ;
   countDown(5);
}



float distanceCm = 30.0;

//=========================================================================================
// Loop function
//=========================================================================================
void loop() 
{
    LEDTOP_B_ON

    Serial.print( "\n1,Moving Forward ;" );
    robot.go(commandDir_e::GOF, robotSpeed_e::MEDIUM);
    //cmPercorsi= robot.moveCm(30);
        while (robot.status.cmd.stepsDone/ ROBOT_MOTOR_STEPS_PER_CM < distanceCm)
        {
            MSG2("Stp:",robot.status.cmd.stepsDone)
            delay(100);
        }
        robot.stop();

    Serial.print("\n cm percorsi FW:" ); Serial.println(robot.status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_CM);

    LEDTOP_B_OFF
    delay(2000);


    distanceCm *= -1; //inverte
 /*
  // Process incoming serial data, and perform callbacks
    float radPercorsi = 0.0;
    robot.setRele(1,0);
     Serial.print( "1, rotation CW;" );
     radPercorsi = robot.rotateRadiants( PI/10 );
     delay( 1000 );
     robot.setRele(1,1);
 

     // Ruota a destra e sinistra
     Serial.print( "1, rotation CCW;" );
     radPercorsi = robot.rotateRadiants( -PI/10 );
     delay( 1000 );
*/



    

}

// Returns if it has been more than interval (in ms) ago. Used for periodic actions
//bool hasExpired(unsigned long &prevTime, unsigned long interval) {
//  if (  millis() - prevTime > interval ) {
//    prevTime = millis();
//    return true;
//  } else     
//    return false;
//}

