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
// ===               SETUP SERIALE				                ===
// ================================================================
#define SERIAL_BAUD_RATE 115200

// ================================================================
// LIBRERIE														==
// ================================================================
//#include <FreeRTOS.h>	//to manage multiple tasks
//#include <CmdMessenger.h>  // CmdMessenger
#include <SchedulerARMAVR.h>

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
	#define IMU_SAMPLING_RATE 20	//frequenza di acquisizione e riempimento FIFO
	#define ACC_OFFSET_X	-5239	
	#define ACC_OFFSET_Y	1794	
	#define ACC_OFFSET_Z	1595	// 1688
	#define GYRO_OFFSET_X	21	// -35
	#define GYRO_OFFSET_Y	-1	// -50
	#define GYRO_OFFSET_Z	-18	// 75


	// ----------------------------------------------------------------
	// ---            VARIABILI GLOBALI								---
	// ----------------------------------------------------------------
const  char s=';';	// separatore dei dati sulla seriale
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



// ========================================================================
// ===																	===
// ===                SEND MESSAGE										===
// ===																	===
// ========================================================================
//invia una string codificata come cmdAck
	void SendMsg(String s){
		//cmdMessenger.sendCmdRTOS(cmdAck,s); 
		Serial.println(s);
	}



void setup() {
	// =================================//
	// SETUP PORTA SERIALE				//
	// =================================//
	Serial.begin(SERIAL_BAUD_RATE);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately

	// =================================//
	// SETUP I/O						//
	// =================================//
	  pinMode(LED_HB, OUTPUT);
	  pinMode(LED_TEST, OUTPUT);
	  pinMode(LED_MPU, OUTPUT);
  
  	// =================================//
	// SETUP BUS SPI (Per IMU)			//
	// =================================//
	Wire.begin();
	//TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  	// =================================//
	// SETUP IMU						//
	// =================================//
			//  pulse(LED_HB, 2);
	#pragma region //setup IMU
//		do{
			SendMsg("Initializing I2C devices..");
			mpu.initialize();

			// verify connection
			SendMsg("Testing device connections...");
    
			// LC ------------------------------------------
			bool imuconnected =mpu.testConnection() ;
			if (imuconnected==true){ 
				SendMsg("MPU6050 connection successful");
			} else {
				SendMsg("MPU6050 connection failed");
			}
			digitalWrite(LED_MPU, imuconnected);


			//// wait for ready
			//Serial.println(F("\nSend any character to begin DMP programming and demo: "));
			//while (Serial.available() && Serial.read()); // empty buffer
			//while (!Serial.available());                 // wait for data
			//while (Serial.available() && Serial.read()); // empty buffer again

			// load and configure the DMP
			SendMsg("Initializing DMP...");
			devStatus = mpu.dmpInitialize();

			// supply your own gyro offsets here, scaled for min sensitivity
			mpu.setXAccelOffset(ACC_OFFSET_X);
			mpu.setYAccelOffset(ACC_OFFSET_Y);
			mpu.setZAccelOffset(ACC_OFFSET_Z);
			mpu.setXGyroOffset(GYRO_OFFSET_X);   
			mpu.setYGyroOffset(GYRO_OFFSET_Y);  
			mpu.setZGyroOffset(GYRO_OFFSET_Z);  

//		} while (!(devStatus==0));


		// make sure it worked (returns 0 if so)
		if (devStatus == 0) {

			// turn on the DMP, now that it's ready
			SendMsg("Enabling DMP...");
			mpu.setDMPEnabled(true);

			// enable Arduino interrupt detection
			//SendMsg("Enabling interrupt detection (Arduino Due external interrupt 2)...");
			attachInterrupt(IMU_INT_PIN, IRS_mpuDataReady, RISING); //spostato su


			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			SendMsg("DMP ready! Waiting for first interrupt...");
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();



			// LC --------------------------------------------
			SendMsg("Fifo packet size:"); 
			SendMsg(String(packetSize));

			//const int16_t sRate = 2; //Requested Sample Rate 
 			int sRate = IMU_SAMPLING_RATE;         // incoming serial byte
		  // Serial.println(F("Input Rate..."));

			int16_t divider;
			divider = (1000/sRate)-1 ;  // default = 0x19
			mpu.setRate(divider);
			SendMsg("Gyro sampling rate has been set to: Hz: "); //The sensor register output, FIFO output, DMP sampling and Motion detection are all based on the Sample Rate.
			SendMsg(String(sRate)) ;

			//--------------------------------------------------LC

		} else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			SendMsg("DMP Initialization failed (code ");
			SendMsg(String(devStatus));
		}

		vel = 0;
		timer = micros();
		vlast = 0;
		mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
		mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
		//Serial.println(mpu.getFullScaleGyroRange());
		//Serial.println(mpu.getFullScaleAccelRange());
		//Serial.println(mpu.getDLPFMode());
		//Serial.println(mpu.getDHPFMode());
		mpu.setDLPFMode(MPU6050_DLPF_BW_98);
		//Serial.println(mpu.getDLPFMode());
		//SendMsg("impostato setFullScaleAccelRange =MPU6050_ACCEL_FS_2");
		//Serial.println(devStatus);
		SendMsg("END of IMU setup------------------------------");

	#pragma endregion // setup the IMU sensor	


	// =================================//
	// SETUP SCHEDULER					//
	// =================================//
	// Add "loop2" and "loop3" to scheduling.
	// "loop" is always started by default.
	Scheduler.startLoop(loopLED);	//blink LED with 0.1 second delay.
	Scheduler.startLoop(loopCMD);	// Task no.3: accept commands from Serial port
	Scheduler.startLoop(loopMSG); //TEST SERIALE CONDIVISA
	Scheduler.startLoop(loop_IMU);
}

// ========================================================================
// ===																	===
// ===       TASK: HEARTHBEAT											===
// ===																	===
// ========================================================================
// Task no.1: blink LED with 1 second delay.
void loop() {
  digitalWrite(LED_HB, HIGH);

  // IMPORTANT:
  // When multiple tasks are running 'delay' passes control to
  // other tasks while waiting and guarantees they get executed.
  delay(1000);

  digitalWrite(LED_HB, LOW);
  delay(1000);
}


// ========================================================================
// ===																	===
// ===       TASK: LED INTERMITTENTE											===
// ===																	===
// ========================================================================
// Task no.2: blink LED with 0.1 second delay.
void loopLED() {
  digitalWrite(LED_TEST, HIGH);
  delay(100);
  digitalWrite(LED_TEST, LOW);
  delay(100);
}

// ========================================================================
// ===																	===
// ===       TASK: GESTIONE COMANDI DA SERIALE							===
// ===																	===
// ========================================================================
// Task no.3: accept commands from Serial port
// '0' turns off LED
// '1' turns on LED
void loopCMD() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '0':
        digitalWrite(LED_MPU, LOW);
        Serial.println("Led turned off!");
        break;
      case '1':
        digitalWrite(LED_MPU, HIGH);
        Serial.println("Led turned on!");
        break;
      default:
        Serial.print("Comando [");
        Serial.print(c);
        Serial.println("] non riconosciuto, inviare 0 o 1:");
    }
  }

  // IMPORTANT:
  // We must call 'yield' at a regular basis to pass
  // control to other tasks.
  yield();
}

// ========================================================================
// ===																	===
// ===       TASK: TEST SERIALE CONDIVISA								===
// ===																	===
// ========================================================================
//-----------------------------------------------------------
void loopMSG(){
  SendMsg(">");
  delay(2000);
  yield();
}

// ========================================================================
// ===																	===
// ===                 I M U   T A S K									===
// ===																	===
// ========================================================================
void loop_IMU() { // Loop IMU  
#pragma region //vTask_IMU

	// if programming failed, don't try to do anything
	if (!dmpReady) {
		SendMsg("MPU not ready");
		yield();

	}
	else
	{    
		

		// wait for MPU interrupt or extra packet(s) available
		while (!mpuInterrupt && fifoCount < packetSize) {
			// other program behavior stuff here
			yield();		
		}



		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();		 
		fifoCount = mpu.getFIFOCount(); // get current FIFO count

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			SendMsg("FIFO overflow!");
			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		//			else 
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


			//----------------------------
			// OUTPUT  Result
			//----------------------------																																		
			int dec=4;
			String sPosition ;

			sPosition = String(q.x,dec) +s +String(q.y,dec) +s +String(q.z,dec) +s +String(q.w,dec)+s; 
			sPosition += String(aaReal.x,dec) +s  +String(aaReal.y,dec) +s +String(aaReal.z,dec) +s;
			sPosition += String(aaWorld.x,dec) +s  +String(aaWorld.y,dec) +s +String(aaWorld.z,dec) +s;
			sPosition += String(gravity.x,dec) +s  +String(gravity.y,dec) +s +String(gravity.z,dec) +s;

			//SER_CH.print(q.x,6);SER_CH.print(s);
			//SER_CH.print(q.y,6);SER_CH.print(s);
			//SER_CH.print(q.z,6);SER_CH.print(s);
			//SER_CH.print(q.w,6);SER_CH.print(s);

			//SER_CH.print(aaReal.x,dec);SER_CH.print(s);
			//SER_CH.print(aaReal.y,dec);SER_CH.print(s);
			//SER_CH.print(aaReal.z,dec);SER_CH.print(s);

			//SER_CH.print(aaWorld.x,dec);SER_CH.print(s);
			//SER_CH.print(aaWorld.y,dec);SER_CH.print(s);
			//SER_CH.print(aaWorld.z,dec);SER_CH.print(s);


			//SER_CH.print(gravity.x,dec);SER_CH.print(s);
			//SER_CH.print(gravity.y,dec);SER_CH.print(s);
			//SER_CH.print(gravity.z,dec);SER_CH.print(s);

			//SER_CH.println();


			SendMsg(sPosition);


		} 

		yield();
	}	//for
#pragma endregion
} // end IMU task --------------------------------------------------------------------

