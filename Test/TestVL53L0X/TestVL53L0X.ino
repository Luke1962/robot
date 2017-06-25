/* This example shows how to get single-shot range
 measurements from the VL53L0X. The distanceSensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */
#define SERIAL_PC Serial			//  MMI > Serial3  Test > Serial
#define SERIAL_PC_BAUD_RATE 115200
#define SERIAL_MMI Serial1			//  MMI > Serial3  Test > Serial
#define SERIAL_MMI_BAUD_RATE 115200
#define SERIAL_MSG Serial

#define MSG(s)  	SERIAL_MSG.print("1,"); SERIAL_MSG.print(F(s)); SERIAL_MSG.println(";");SERIAL_MMI.print("1,"); SERIAL_MMI.print(F(s)); SERIAL_MMI.println(";");
#define MSG2(s,v)  	SERIAL_MSG.print("1,"); SERIAL_MSG.print(F(s)); SERIAL_MSG.print(v); SERIAL_MSG.println(";"); SERIAL_MMI.print("1,"); SERIAL_MMI.print(s); SERIAL_MMI.print(v); SERIAL_MMI.println(";");

#define MSG3(s,v,t) SERIAL_MSG.print("1,"); SERIAL_MSG.print(F(s)); SERIAL_MSG.print(v); SERIAL_MSG.print(F(t)); SERIAL_MSG.println(";");SERIAL_MMI.print("1,"); SERIAL_MMI.print(F(s)); SERIAL_MMI.print(v); SERIAL_MMI.print(F(t)); SERIAL_MMI.println(";");


// utilizzo di Wire - bloccante se il sensore è spento o non risponde
#include <Wire\Wire.h>
// verione non bloccante da 
//#include <arduino.h>
//#include <nbWire\nbWire.h>
//#include <I2Cdev\I2Cdev.h>
#include <VL53L0X.h>

VL53L0X LDS;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the distanceSensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY


void setup()
{
	SERIAL_PC.begin(SERIAL_PC_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
 	Wire.begin();
	//Wire.setTimeout(1000);
	Serial.println("test VL53L0X distance sensor");
	Serial.print("inizializzo...");
	bool initDone = false;
	while (!initDone)
	{
		if(LDS.init())
		{
		Serial.println("..init OK");
		initDone = true;
		}
		else
		{
		Serial.println("..init FAIL");
		delay(500);
		}
	}

  
	LDS.setTimeout(500);

	#if defined LONG_RANGE
		Serial.println("Long range");
		// lower the return signal rate limit (default is 0.25 MCPS)
		LDS.setSignalRateLimit(0.1);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
		LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	#endif

	#if defined HIGH_SPEED
		// reduce timing budget to 20 ms (default is about 33 ms)
		distanceSensor.setMeasurementTimingBudget(20000);
	#elif defined HIGH_ACCURACY
		Serial.println("High Accuracy");
		// increase timing budget to 200 ms
		distanceSensor.setMeasurementTimingBudget(200000);
	#endif
}
uint16_t d = 0;


void loop()
{ 
	d = LDS.readRangeSingleMillimeters();
 	//SERIAL_MMI.print("\n1,"); SERIAL_MMI.print(d); SERIAL_MMI.print(";");
	//SERIAL_PC.print("\n1,"); 
	//SERIAL_PC.println(d);// SERIAL_PC.print(";");
	MSG2("LDS :", d);
	if (LDS.timeoutOccurred()) { 
		MSG("  TIMEOUT ");
		Wire.begin();

		LDS.init();

	}

 }
