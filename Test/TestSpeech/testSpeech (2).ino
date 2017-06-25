#define SPEECH_SERIAL Serial2


#define SERIAL_BAUD_RATE 9600
#define ROBOT_STARTING_MODE AUTONOMOUS	//  SLAVE	


//#include <FrequencyTimer2\FrequencyTimer2.h>	
#include <TimerThree\TimerThree.h>
#include <FlexiTimer2\FlexiTimer2.h>
#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
#include <avr/wdt.h>
#include <robot/robot.h>

#if OPT_SERVOSONAR
// va messo prima dell'istanza del robot
Servo servoSonar;
NewPing Sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif

struct robot_c robot;

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger/CmdMessenger.h>
CmdMessenger cmd = CmdMessenger(Serial1);
#include <robot/RobotInterfaceCommands.h>

void setup()
{
	Serial.begin(9600);
	Serial.print("test controllo speech module v1");
  /* add setup code here */
	SPEECH_SERIAL.begin(9600);
}
#define T 2000
void loop()
{

	SPEAK_CIAO
	delay(T);
	SPEECH_SERIAL.print("h");
	delay(T);
	SPEECH_SERIAL.print("ei ei");
	delay(T);
	SPEECH_SERIAL.print("b");
	delay(T);
	SPEECH_SERIAL.print("l");
	delay(T);
	SPEECH_SERIAL.print("r");
	delay(T);
	SPEECH_SERIAL.print("o");
	delay(T);
}
