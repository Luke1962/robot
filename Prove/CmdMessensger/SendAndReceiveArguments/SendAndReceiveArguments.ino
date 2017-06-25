// *** SendandReceiveArguments ***

// This example expands the previous SendandReceive example. The Arduino will now receive multiple 
// and sent multiple float values. 
// It adds a demonstration of how to:
// - Return multiple types status; It can return an Acknowlegde and Error command
// - Receive multiple parameters,
// - Send multiple parameters
// - Call a function periodically

#include <CmdMessenger.h>  // CmdMessenger

// Blinking led variables 
unsigned long previousToggleLed = 0;   // Last time the led was toggled
bool ledState                   = 0;   // Current state of Led
const int kBlinkLed             = 13;  // Pin of internal Led

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);



// ------------------  C A L L B A C K S -----------------------


// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
        Ack,
        kError,
        kSetLed, // Command to request led to be set in specific state
        kSetLedFrequency,
        Status,
        DataMPU,
        StatusMPU,
        ErrorMPU,
        CmdMPUReset,
        CmdMPUSetRate,
        CmdMPUStart,
        CmdMPUStop,
        CmdMPUCalibrate,
        CmdRobotStop, CmdRobotMoveF,CmdRobotMoveB,CmdRobotMoveL, CmdRobotMoveR,  CmdRobotStartMoving,   //
        CmdGetSensorsHighRate, CmdGetSensorsLowRate,
         CmdRobotRele,
};

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kSetLed, OnSetLed);
  cmdMessenger.attach(kSetLedFrequency, OnSetLedFrequency);
}

// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError,"Command without attached callback");
}

// Callback function that sets led on or off
void OnSetLed()
{
  // Read led state argument, interpret string as boolean
//  ledState = cmdMessenger.readBoolArg();
	  digitalWrite(13,!digitalRead(13)); 
  cmdMessenger.sendCmd(Ack,ledState);
}

// Callback function that sets leds blinking frequency
void OnSetLedFrequency()
{
  int ledFrequency,intervalOn,intervalOff;
  // Read led state argument, interpret string as boolean
  ledFrequency = cmdMessenger.readFloatArg();
  // Make sure the frequency is not zero (to prevent divide by zero)
  if (ledFrequency < 0.001) { ledFrequency = 0.001; }
  // translate frequency in on and off times in miliseconds
  intervalOn  = (500.0/ledFrequency);
  intervalOff = (1000.0/ledFrequency);
  cmdMessenger.sendCmd(Ack,10);
}

//-----------------------------------------------------------fine codice sotto test


// ------------------ M A I N  ----------------------

// Setup function
void setup() 
{
  // Listen on serial connection for messages from the pc
  Serial.begin(115200); 

  // Adds newline to every command
  cmdMessenger.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Send the status to the PC that says the Arduino has booted
  cmdMessenger.sendCmd(Ack,"Arduino has started!");

  // set pin for blink LED
  pinMode(kBlinkLed, OUTPUT);
}

// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval) {
  if (  millis() - prevTime > interval ) {
    prevTime = millis();
    return true;
  } else     
    return false;
}

// Loop function
void loop() 
{
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();

  // Toggle LED periodically. If the LED does not toggle every 2000 ms, 
  // this means that cmdMessenger are taking a longer time than this  
  if (hasExpired(previousToggleLed,2000)) // Toggle every 2 secs
  {
 //   toggleLed();  
   // Send the status to the PC that says the Arduino has booted
  cmdMessenger.sendCmd(Status,"ok");

  } 
}

// Toggle led state 
void toggleLed()
{  
  ledState = !ledState;
  digitalWrite(kBlinkLed, ledState?HIGH:LOW);
}  
