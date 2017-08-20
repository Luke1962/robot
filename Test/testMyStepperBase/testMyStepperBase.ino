#include <Arduino.h>
#include <digitalWriteFast\digitalWriteFast.h>
#include <MyStepperBase\MyStepperBase.h>
#include <MyRobotLibs\hw_config.h>

StepperBase_c robotMotors;

void setup()
{
	initRobotBaseHW();
}
void loop()
{
	LEDTOP_R_ON
	robotMotors.goCmdVel(0, 0.40);
	delay(4000);
	LEDTOP_R_OFF
	robotMotors.stop();
	delay(1000);

	LEDTOP_B_ON
	robotMotors.goCmdVel(0, -0.40);
	delay(4000);
	LEDTOP_B_OFF
	robotMotors.stop();
	delay(1000);

}