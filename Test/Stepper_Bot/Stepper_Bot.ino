#include <stdlib.h>
#include <math.h>
#include <FiniteStateMachine/FiniteStateMachine.h>
#include <Servo.h> 
#include <AccelStepper/AccelStepper.h>
#include <Streaming/Sreaming.h>
#include <digitalWriteFast.h>
#include <EnableInterrupt/EnableInterrupt.h>
#include <FastRunningMedian/FastRunningMedian.h>
//#include <IRRanger.h>

/*****************************************************************/
/****************** Sensoren       *******************************/
/*****************************************************************/

//Sonar
unsigned long PingIntervalTimer = millis();

long PingStartTime = micros();
long PingReceiveTime = micros();
int PingRoundtrip = 0;
boolean PingReceived = false;
boolean triggerRunning = false;
boolean echoEnabled = false;
int PingCurrentSensor = 1;

#define SONAR_NUM 3
#define PingLPin  A15
#define PingRPin  A14
#define PingSPin  A8
#define MAX_DISTANCE_SONAR 150 
#define temperature 21

int PING_INTERVAL = 40;

int PingCM[SONAR_NUM];
int PingMS[SONAR_NUM];

uint8_t currentSensor = 0;



//servoSense1
Servo servoSense1;
#define SizeOfservoSense1Array 9
int servoSense1Array_out[SizeOfservoSense1Array] = {2360,2140,1950,1760,1600,1380,1250,1050,830}; //timing
int servoSense1Array_in[SizeOfservoSense1Array]  = {10,30,50,70,90,110,130,150,170}; //angels

#define NrOfIRSensors 5
#define servoSense1Pin 28
#define SharpIRFront A10
#define SharpIRLeft  A9 //A9
#define SharpIRRight A7 //A7

#define servoSense1msPerDeg 5
int servoSense1pos = 89;
int servoSense1oldPos = 89;
int servoMoveDelayCounter = 0;
int Scantype = 0;
int CurrentScanrounds = 0;
int Scanrounds = 2;

boolean sweepdir = 1;
boolean ServoMoving = false;
int sweepspeed = 3; //10
#define IRcycleForValidRead 3
int IRreadCount = 0;
float IRRead[NrOfIRSensors] = {0,0,0};

FastRunningMedian<unsigned int,IRcycleForValidRead, 0> newMedian_IR_left;
FastRunningMedian<unsigned int,IRcycleForValidRead, 0> newMedian_IR_right;
FastRunningMedian<unsigned int,IRcycleForValidRead, 0> newMedian_IR_front;

//IRRanger IR_left(GP2Y0A21YK, SharpIRLeft, 0);
//IRRanger IR_right(GP2Y0A21YK, SharpIRRight, 0);
//IRRanger IR_front(GP2Y0A02YK0F, SharpIRFront, 0);

/*****************************************************************/
/****************** Dead Reckoning *******************************/
/*****************************************************************/
float MUL_COUNTl;
float MUL_COUNTr;
float MUL_COUNT_avg;
#define PULSES_PER_REVOLUTION 800

#define WHEEL_DIAMETERl 7.96005
#define WHEEL_DIAMETERr 7.95995
#define AXLE_LENGTH 13.83

#define stepper_highspeed_l 249.998429658105
#define stepper_highspeed_r 250.001570361623
#define stepper_lowspeed_l 99.9993718632421
#define stepper_lowspeed_r 100.000628144649
#define stepper_acceleration_l 499.99685931621
#define stepper_acceleration_r 500.003140723246

#define	MUL_COUNTl  PI * WHEEL_DIAMETERl / PULSES_PER_REVOLUTION
#define	MUL_COUNTr  PI * WHEEL_DIAMETERr / PULSES_PER_REVOLUTION
#define	MUL_COUNT_avg (MUL_COUNTl + MUL_COUNTr) / 2

#define startPosX 0.0
#define startPosY 0.0

float theta; 
float X_pos = startPosX;               
float Y_pos = startPosY;

long last_left = 0.0, last_right = 0.0;
long lsamp = 0.0, rsamp = 0.0;

unsigned long DRIntervalTimer = millis();

/*****************************************************************/
/****************** Stepper **************************************/
/*****************************************************************/
#define StepLReset 29
#define StepLEnable 30
#define StepLStep 31
#define StepLDir 32

#define StepREnable 33
#define StepRReset 34
#define StepRStep 35
#define StepRDir 36

AccelStepper stepperL(1,StepLStep,StepLDir);
AccelStepper stepperR(1,StepRStep,StepRDir);

boolean SteppersEnabled = false;


/*****************************************************************/
/****************** FSM_Drive ************************************/
/*****************************************************************/
byte Drive_returnstate;
#define Drive_Return_GoTo 1
#define Drive_Return_Turn 2
#define Drive_Return_Align 3

int theta_goal = 0;
int IRTargetReachedCounter = 0;

float X_pos_target;
float Y_pos_target;
float theta_target;

int stopProcedureStep = 0;

int PingFrontEmergancyStop = 0;
#define  EmergancyStopThreshhold 2

//allign
int PingR = 0;
int PingL = 0;
int SonarDiff = 0;
unsigned int pingCounter = 0;


/*****************************************************************/
/****************** Output ***************************************/
/*****************************************************************/
boolean output_stream_on = true;
unsigned long OutputIntervalTimer = millis();
int thetaLastinDeg=9999;
int xLast=0;
int yLast=0;


/*****************************************************************/
/****************** Serial Com ***********************************/
/*****************************************************************/
String inputString = "";  boolean stringComplete = false;
#define endl "\n"
#define XRF_RTS 40


/*****************************************************************/
/****************** Aim and Shoot *******************************/
/*****************************************************************/
int AimXError = 0;
int AimDistance_Hight = 0;
int AimDist = 0;

boolean newAimInfoReceived = false;
boolean FireSequence = false;

#define SizeOfAimArray 3
int AimArray_CM[SizeOfAimArray] = {25,45,65}; //dist
int AimArray_X_out[SizeOfAimArray]  = {87,55,40}; //offset x-axis

int AimArray_Y_20_in[SizeOfAimArray]  = {85,162,198}; //pixels y-axis 20 cm
int AimArray_Y_20_out[SizeOfAimArray]  = {99,74,69}; //angle y-axis 20 cm

int AimArray_Y_40_in[SizeOfAimArray]  = {87,139,150}; //pixels y-axis 40 cm
int AimArray_Y_40_out[SizeOfAimArray]  = {99,83,81}; //angle y-axis 40 cm

int AimArray_Y_60_in[SizeOfAimArray]  = {106,125,140}; //pixels y-axis 60 cm
int AimArray_Y_60_out[SizeOfAimArray]  = {98,93,90}; //angle y-axis 60 cm

/*****************************************************************/
/****************** Roaming *******************************/
/*****************************************************************/

#define Roam_Dist_start_Turn 30
int Roam_Dist_start_Turn_counter = 0;
int Roam_Dist_start_Turn_check_counter = 0;

#define Roam_Dist_start_Turn_threshold 3
#define Roam_degreeToTurn -45


/*****************************************************************/
/****************** MISC *****************************************/
/*****************************************************************/
int MapOffsetCalc[2];

/*****************************************************************/
/****************** State Machines *******************************/
/*****************************************************************/

State Drive_Idle = State(Drive_Idle_Entry_F , Drive_Idle_F, Drive_Idle_Exit_F);
State Drive_PrepareIdle = State(NULL , Drive_PrepareIdle_F, NULL);
State Drive_GoTo = State(Drive_GoTo_Entry_F, Drive_GoTo_F, NULL);
State Drive_Prepare_GoTo = State(NULL, Drive_Prepare_GoTo_F, NULL);
State Drive_Turn = State(Drive_Turn_Entry_F, Drive_Turn_F, NULL);
State Drive_Align = State(Drive_Align_Entry_F, Drive_Align_F, NULL);
FSM FSM_Drive = FSM(Drive_Idle);
unsigned long Drive_waitCounter = millis();

State servoSense1_Idle = State(servoSense1_Idle_Entry_F , servoSense1_Idle_F, servoSense1_Idle_Exit_F);
State servoSense1_MoveServo = State(servoSense1_reset_waitCounter , servoSense1_MoveServo_F, NULL);
State servoSense1_Sweep = State(NULL , servoSense1_Sweep_F, NULL);
State servoSense1_CheckIRSensors = State(servoSense1_CheckIRSensors_Entry_F , servoSense1_CheckIRSensors_F, NULL);
FSM FSM_servoSense1 = FSM(servoSense1_Idle);
unsigned long servoSense1_waitCounter = millis();

State Roaming_Idle = State(NULL , Roaming_Idle_F, NULL);
State Roaming_Drive = State(NULL, Roaming_Drive_F, NULL);
State Roaming_CheckAhead = State(NULL, Roaming_CheckAhead_F, Roaming_CheckAhead_Exit_F);
State Roaming_WaitingTurn = State(NULL, Roaming_WaitingTurn_F, NULL);
FSM FSM_Roaming = FSM(Roaming_Idle);
unsigned long Roaming_waitCounter = millis();

/*****************************************************************/
/****************** Setup  ***************************************/
/*****************************************************************/

void setup()
{
	Serial.begin(57600); inputString.reserve(200); delay(100);
	configurePins();
	sendMSG("Initializing Robot");

	initialize_odometry();

	setupStepper();

	//Sharp sensors
	//IR_left.setModel(EXPS, 38.688995278123514, -0.64847473617945017, 154.28355424972492, -3.3970300340588917);

	enableEcho();

	//enable Scan by Default
	Scantype = 1; FSM_servoSense1.transitionTo(servoSense1_CheckIRSensors);

	sendMSG("Initialization done"); sendMSG("-------------"); sendMSG("Free Memory: " + String(freeRam()) + " bytes"); sendMSG("-------------");

}

void(* resetFunc) (void) = 0; /****************** RESET Function */

/*****************************************************************/
/****************** Main Loop  ***********************************/
/*****************************************************************/

void loop() {
	FSM_servoSense1.update();
	stepperUpdate();
	FSM_Roaming.update();
	FSM_Drive.update();
	stepperUpdate();
	dead_reconing();
	stepperUpdate();
	outputPos();
	stepperUpdate();
	triggerEcho();
}

void configurePins(){
	for (int myPin = 0; myPin <=69; myPin++){pinMode(myPin, INPUT_PULLUP);}

	//sonar
	pinMode(PingSPin,INPUT);
	pinMode(PingLPin,INPUT);
	pinMode(PingRPin,INPUT);
	//ir
	pinMode(SharpIRFront,INPUT);
	pinMode(SharpIRLeft,INPUT);
	pinMode(SharpIRRight,INPUT);
	//stepper
	for (int i=29;i<=36;i++){pinMode(i,OUTPUT);}
	//XRF
	//pinMode(XRF_RTS, INPUT); //RTS for XRF
}

void setupStepper(){
	digitalWriteFast(StepLReset,HIGH);
	digitalWriteFast(StepRReset,HIGH);

	stepperL.setMaxSpeed(stepper_highspeed_l);
	stepperL.setAcceleration(stepper_acceleration_l);
	stepperL.setEnablePin(StepLEnable);
	digitalWriteFast(StepLEnable,LOW);
	stepperL.setMinPulseWidth(1);

	stepperR.setMaxSpeed(stepper_highspeed_r);
	stepperR.setAcceleration(stepper_acceleration_r);
	stepperR.setEnablePin(StepREnable);
	digitalWriteFast(StepREnable,LOW);
	stepperR.setMinPulseWidth(1);
}