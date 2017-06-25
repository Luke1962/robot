/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Due (Programming Port), Platform=sam, Package=arduino
*/

#define ARDUINO 165
#define ARDUINO_MAIN
#define F_CPU 84000000L
#define printf iprintf
#define __SAM__
#define printf iprintf
#define F_CPU 84000000L
#define ARDUINO 165
#define ARDUINO_SAM_DUE
#define ARDUINO_ARCH_SAM
#define __SAM3X8E__
#define USB_VID 0x2341
#define USB_PID 0x003e
#define USBCON
extern "C" void __cxa_pure_virtual() {;}

//
//
void configurePins();
void setupStepper();

#include "C:\Users\Luca\AppData\Roaming\arduino15\packages\arduino\hardware\sam\1.6.3\variants\arduino_due_x\pins_arduino.h" 
#include "C:\Users\Luca\AppData\Roaming\arduino15\packages\arduino\hardware\sam\1.6.3\variants\arduino_due_x\variant.h" 
#include "C:\Users\Luca\AppData\Roaming\arduino15\packages\arduino\hardware\sam\1.6.3\cores\arduino\arduino.h"
#include <Stepper_Bot.ino>
