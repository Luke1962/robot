/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#pragma region Compass
	#include <HMC5883L.h>
 
	HMC5883L compass;
	void setup_Compass() {
		// Initialize Initialize HMC5883L
		Serial.println("Initialize HMC5883L");
		while (!compass.begin())
		{
			Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
			delay(500);
		}

		// Set measurement range
		compass.setRange(HMC5883L_RANGE_1_3GA);

		// Set measurement mode
		compass.setMeasurementMode(HMC5883L_CONTINOUS);

		// Set data rate
		compass.setDataRate(HMC5883L_DATARATE_30HZ);

		// Set number of samples averaged
		compass.setSamples(HMC5883L_SAMPLES_8);

		// Set calibration offset. See HMC5883L_calibration.ino
		compass.setOffset(0, 0);

	}
#pragma endregion


#pragma region MPU
	#include <MPU6050.h>
	MPU6050 mpu;

	void setup_MPU() {
		while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
		{
			Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
			delay(500);
		}
		// If you have GY-86 or GY-87 module.
		// To access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode


		mpu.setI2CMasterModeEnabled(false);
		mpu.setI2CBypassEnabled(true);
		mpu.setSleepEnabled(false);
	}

#pragma endregion

void setup() 
{
	Serial.begin(115200);
	Serial.println("Test di MPU6050 e Compass messi insieme");

	Serial.println("Initialize MPU6050");

	setup_MPU();
	setup_Compass();

}


void loop()
{
	// Read normalized values 
	Vector normAccel = mpu.readNormalizeAccel();
	
	// Calculate Pitch & Roll
	int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
	int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

	// Output
	Serial.print(" Pitch = ");
	Serial.print(pitch);
	Serial.print(" Roll = ");
	Serial.print(roll);
  
	Serial.println();
  
	delay(10);








	Vector norm = compass.readNormalize();

	// Calculate heading
	float heading = atan2(norm.YAxis, norm.XAxis);

	// Set declination angle on your location and fix heading
	// You can find your declination on: http://magnetic-declination.com/
	// (+) Positive or (-) for negative
	// For Bytom / Poland declination angle is 4'26E (positive)
	// Formula: (deg + (min / 60.0)) / (180 / M_PI);
	float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
	heading += declinationAngle;

	// Correct for heading < 0deg and heading > 360deg
	if (heading < 0)
	{
		heading += 2 * PI;
	}

	if (heading > 2 * PI)
	{
		heading -= 2 * PI;
	}

	// Convert to degrees
	float headingDegrees = heading * 180 / M_PI;

	// Output
	Serial.print(" Heading = ");
	Serial.print(heading);
	Serial.print(" Degress = ");
	Serial.print(headingDegrees);
	Serial.println();

	delay(100);




}


