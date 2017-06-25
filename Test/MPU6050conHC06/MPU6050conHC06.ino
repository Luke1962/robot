#include <Arduino.h>

// ================================================================
// ===          HW SETUP   SENSORE INERZIALE + BLUETOOTH        ===
// ================================================================
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LED_PIN1 9 //LED ACCESO=MPU CONNESSO
#define IMU_INT_PIN 2  //PIN al quale collegare il segnale di interupt da MPU

// ================================================================
// ===               BLUETOOTH MODULE HC06		                ===
// ================================================================
#include <HC05.h>
HC05 btSerial = HC05(7, 6);  // cmd, state
#define SER_CH HC05_HW_SERIAL_PORT	//Serial channel of IMU output sream: Serial|btSerial 
#define SERIAL_SPEED 115200

// ================================================================

// ================================================================
// ===               IMU MPU6050 SETUP			                ===
// ================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define IMU_SAMPLING_RATE 10


// OUTPUT FORMAT SETUP--------------------------------------------
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE // Piant hading at each row
#define OUTPUT_TEAPOT// uncomment "OUTPUT_TEAPOT" if you want output that matches the format used for the InvenSense teapot demo
//---------------------------------------------------------------

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

//LC--------------------
const  char s=';';	// separatore dei dati sulla seriale
//--------------------LC

 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 



// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {

	// ================================================================
	// ===                    BLUETOOTH  INITIAL SETUP              ===
	// ================================================================

	//btSerial.setBaud(9600);
	//btSerial.findBaud();


	// ================================================================


    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(SERIAL_SPEED);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    
    // LC ------------------------------------------
    bool imuconnected =mpu.testConnection() ;
    Serial.println(imuconnected ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    digitalWrite(LED_PIN1, imuconnected);


    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-35);  //-35	220
    mpu.setYGyroOffset(-50);  //-50	76
    mpu.setZGyroOffset(75);  //75	-85
    mpu.setZAccelOffset(1688); //1830 1688 factory default for my test chip


    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino Due external interrupt 2)..."));
        attachInterrupt(IMU_INT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();



		// LC --------------------------------------------
		Serial.print (F("Fifo packet size:")); 
		Serial.println (packetSize);

		//const int16_t sRate = 2; //Requested Sample Rate 
 		int sRate = IMU_SAMPLING_RATE;         // incoming serial byte
      // Serial.println(F("Input Rate..."));

#ifndef OUTPUT_TEAPOT
	  // if we get a valid byte, read analog ins:
	   while (Serial.available() == 0) {}
		// get incoming byte:
		sRate = Serial.read();

#endif


		int16_t divider;
		divider = (1000/sRate)-1 ;  // default = 0x19
		mpu.setRate(divider);
		Serial.print (F("Gyro sampling rate has been set to:")); //The sensor register output, FIFO output, DMP sampling and Motion detection are all based on the Sample Rate.
		Serial.println(sRate);




		//--------------------------------------------------LC

    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        digitalWrite(LED_PIN, 1);
    }
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    vel = 0;
    timer = micros();
vlast = 0;
    //Serial.println(mpu.getFullScaleGyroRange());
    //Serial.println(mpu.getFullScaleAccelRange());
    //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    //Serial.println(mpu.getFullScaleGyroRange());
    //Serial.println(mpu.getFullScaleAccelRange());
    //Serial.println(mpu.getDLPFMode());
    //Serial.println(mpu.getDHPFMode());
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);
    //Serial.println(mpu.getDLPFMode());
}
 
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
//	else 
		if (mpuIntStatus & 0x02) {

        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
       
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

		//        #ifdef OUTPUT_READABLE_REALACCEL

            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            thr = sqrt(pow((double)aaWorld.x/16384,2) + pow((double)aaWorld.y/16384,2) + pow((double)aaWorld.z/16384,2));
            //aax = ((double) aaWorld.x * 0.70) + (aax*(0.30));
            //thr  = (double)aaWorld.x;
              

			if(thr < 0.1 && (vlast < 0.1 && vlast > -0.1)){
				vel=0;
				//   kmp = 0;
			}
			//dt = ((micros() - timer));
			if((aaWorld.x > 25 || aaWorld.x < -25 ) && (thr >= 0.009))
			{
				//vel = (vel + (((double)aax / 16384) * ((double)(micros() - timer) / 1000000)));
				vel = (vel + ((((double)aaWorld.x/16384) * 9.81) * ((double)(micros() - timer) / 1000000)));

				//kmp = vel * 3.6;
			}

			// Set velocity to 0 if under theshold ---------------------
			vlast = vel;
			if(thr < 0.3 && (vlast > 0.1 || vlast < -0.1))
			{
				vel = vlast;
			}
			else vel=0;

			timer1 = (double)(micros() - timer1);
			timer = micros();
			blinkState1 = !blinkState1;
			digitalWrite(LED_PIN1, blinkState);

			
			 // Print Result
			 //----------------------------
			
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #else
                #ifdef OUTPUT_READABLE

			SER_CH.println("quternion w,x,y,z , areal.x y z, aw.x,aw.y,aw.z,  g.x,g.y,g.z  ");
		#endif
                        int dec=4;
			SER_CH.print(q.x,6);SER_CH.print(s);
			SER_CH.print(q.y,6);SER_CH.print(s);
			SER_CH.print(q.z,6);SER_CH.print(s);
			SER_CH.print(q.w,6);SER_CH.print(s);

			SER_CH.print(aaReal.x,dec);SER_CH.print(s);
                        SER_CH.print(aaReal.y,dec);SER_CH.print(s);
                        SER_CH.print(aaReal.z,dec);SER_CH.print(s);

			SER_CH.print(aaWorld.x,dec);SER_CH.print(s);
			SER_CH.print(aaWorld.y,dec);SER_CH.print(s);
			SER_CH.print(aaWorld.z,dec);SER_CH.print(s);

			
			SER_CH.print(gravity.x,dec);SER_CH.print(s);
			SER_CH.print(gravity.y,dec);SER_CH.print(s);
			SER_CH.print(gravity.z,dec);SER_CH.print(s);

			SER_CH.println();

        
        #endif



        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }  

}

