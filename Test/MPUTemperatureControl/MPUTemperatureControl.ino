// *** TemperatureControl ***

// This example expands the previous ArduinoController example. The PC will now send a start command to the controller,
// the controller will now start measuring the temperature and controlling the heater. The controller will also start 
// sending temperature and heater steer data which the PC will plot in a chart. With a slider we can set the goal 
// temperature, which will make the PID library on the controller adjust the setting of the heater.
 

#include <CmdMessenger.h>  

#include <DoEvery.h>   

// ========================================================================
// ===																	===
// ===       CONFIGURAZIONE   HW:   SENSORE INERZIALE + BLUETOOTH       ===
// ===																	===
// ========================================================================
#define LED_HB 13 // LED INTERNO > HearthBeat
#define LED_MPU 9 //LED ACCESO=MPU CONNESSO
#define LED_TEST 7 //LED DI TEST

// SENSORE MPU6050 -----------------
#define IMU_INT_PIN 2  //PIN al quale collegare il segnale di interupt da MPU
// Collegare segnali SDA e SLC


// ================================================================
// ================================================================
// ===             SETUP  IMU MPU6050 			                ===
// ================================================================
// ================================================================
#pragma region // SETUP  IMU MPU6050: DFINES, VARIABLES, IRS

	#include <I2Cdev.h>
	#include <MPU6050_6Axis_MotionApps20.h>
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		#include "Wire.h"
	#endif
	MPU6050 mpu;
	#define IMU_SAMPLING_RATE 30	//frequenza di acquisizione e riempimento FIFO
	#define ACC_OFFSET_X	-5239	
	#define ACC_OFFSET_Y	1794	
	#define ACC_OFFSET_Z	1595	// 1688
	#define GYRO_OFFSET_X	21	// -35
	#define GYRO_OFFSET_Y	-1	// -50
	#define GYRO_OFFSET_Z	-18	// 75


	// ----------------------------------------------------------------
	// ---            VARIABILI GLOBALI								---
	// ----------------------------------------------------------------
	const  char s=',';	// separatore dei dati sulla seriale
	const char DECIMALS='4';	//numero di decimali
	String sMsgMPU ;			// stringa contenente i dati inviati al PC


	bool blinkState = false;
	bool blinkState1 = false;

	// MPU control/status vars
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	double vel;
	double vlast;
	double kmp;
	double aax;
	float thr;
	double dtt;
	double RC;
	double alpha;
	uint32_t timer = 0;
	uint32_t timer1 = 0;
	uint32_t dt = 0;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

 
	// ----------------------------------------------------------------
	// ===               INTERRUPT DETECTION ROUTINE                ===
	// ----------------------------------------------------------------
	volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

	void  IRS_mpuDataReady() {
		mpuInterrupt = true;
	}
#pragma endregion


// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);


// ----------------------------------------------------------------
// ===               IMPOSTAZIONI DI CAMPIONAMENTO              ===
// ----------------------------------------------------------------
const int measureInterval          = 500;  // Interval between measurements
DoEvery tempTimer(measureInterval);




double y2                        = 0;
double y3                        = 0;

// Thermocouple pins
int thermoDO                       = 3;
int thermoCS                       = 4;
int thermoCLK                      = 5;

// Solid State switch pin
const int switchPin                = 4;

bool acquireData                   = false; // Logging start/stop flag
bool controlHeater 	           = false; // Heater start/stop flag

long startAcqMillis                = 0;

double CurrentTemperature          = 20;    // Measured temperature
double goalTemperature             = 20;    // Goal temperature

bool switchState                   = false; // Initial binary heater state 
double heaterSteerValue            = 0;     // Initial normalized heater value

// Initialize thermocouple library
//Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

// Initialize PID library
//PID pid(&CurrentTemperature, &heaterSteerValue, &goalTemperature,pidP,pidI,pidD,DIRECT);

// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
  // Commands
  kAcknowledge         , // Command to acknowledge a received command
  kError               , // Command to message that an error has occurred
  kStartLogging        , // Command to request logging start              
  kStopLogging         , // Command to request logging stop               
  kPlotDataPoint       , // Command to request datapoint plotting  
  kSetGoalTemperature  , // Command to set the goal temperature 
};

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kStartLogging, OnStartLogging);
  cmdMessenger.attach(kStopLogging, OnStopLogging);
  cmdMessenger.attach(kSetGoalTemperature, OnSetGoalTemperature);
}

// ------------------  C A L L B A C K S -----------------------

// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError,"Command without attached callback");
}

// Callback function that responds that Arduino is ready (has booted up)
void OnArduinoReady()
{
  cmdMessenger.sendCmd(kAcknowledge,"Arduino ready");
}

// Start data acquisition
void OnStartLogging()
{
  // Start data acquisition
  acquireData = true;
  cmdMessenger.sendCmd(kAcknowledge,"Start Logging");
}

// Stop data acquisition
void OnStopLogging()
{
  acquireData    = false;
  cmdMessenger.sendCmd(kAcknowledge,"Stop Logging");
}

// Callback function that sets leds blinking frequency
void OnSetGoalTemperature()
{
  // Read led state argument, interpret string as float
  float newTemperature = cmdMessenger.readBinArg<float>();
  
  // Make sure that the argument is valid before we change
  // the goal temperature
  if (cmdMessenger.isArgOk()) {
    goalTemperature = newTemperature;
    
    // Enable heater control (was disabled at intialization)
    controlHeater = true;  
  
    // Send acknowledgement back to PC
    cmdMessenger.sendBinCmd(kAcknowledge,goalTemperature); 
  } else {
    // Error in received goal temperature! Send error back to PC
    cmdMessenger.sendCmd(kError,"Error in received new goal temperature");
  }
}
// ========================================================================
// ===																	===
// ===                SEND MESSAGE										===
// ===																	===
// ========================================================================
//invia una string codificata come cmdAck
	void SendMsg(String s){
		cmdMessenger.sendCmd(kAcknowledge,s); //Serial.println(s);
	}

// ------------------ M A I N  ----------------------

// Setup function
void setup() 
{
  // Listen on serial connection for messages from the pc
  Serial.begin(115200); 
  
  // Do not print newLine at end of command, 
  // in order to reduce data being sent
  cmdMessenger.printLfCr(false);
  
  
  
    	// =================================//
	// SETUP BUS SPI (Per IMU)			//
	// =================================//
	Wire.begin();
	mpu.initialize();
	mpu.setTempSensorEnabled(true);
	mpu.setDMPEnabled(true);

	// supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Send the status to the PC that says the Arduino has booted
  cmdMessenger.sendCmd(kAcknowledge,"Arduino has started!");
  
  
}

// Loop function
void loop() 
{
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
 
  // Every 100 ms, update the temperature
  if(tempTimer.check()) measure();
 


}

// simple readout of two Analog pins. 
void measure() {
  if (acquireData) {
     // Calculate time
     float seconds     = (float) (millis()-startAcqMillis) /1000.0 ;

    // read raw accel/gyro measurements from device
    //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // read raw accel/gyro measurements from device
    mpu.getRotation(&gx, &gy, &gz);


     // Measure temperature using thermocouple
     CurrentTemperature =mpu.getTemperature()/340 + 36.53;


     // Send data to PC    
     cmdMessenger.sendCmdStart(kPlotDataPoint);  
     cmdMessenger.sendCmdBinArg((float)seconds);                           // Time    
     cmdMessenger.sendCmdBinArg((float)gx);                // rosso  
     cmdMessenger.sendCmdBinArg((float)gy);                // verde  
     cmdMessenger.sendCmdBinArg((float)gz);                // blu 
 
	 cmdMessenger.sendCmdBinArg((float)CurrentTemperature); //  grafico sotto


//     cmdMessenger.sendCmdBinArg((bool)switchState);                        // On / off state during PWM cycle
     cmdMessenger.sendCmdEnd();    
  }  
} 


