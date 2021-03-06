

//SUBSCRIBERS
#define LIZI_COMMAND_TOPIC "command"
#define LIZI_PAN_TILT_TOPIC "pan_tilt"
//PUBLISHER
#define LIZI_RAW_TOPIC "lizi_raw"
#define LIZI_GPS_TOPIC "raw_gps"
#define LIZI_STATUS_TOPIC "status"
//SERVICES
#define  RESET_ENCODERS_SRV "reset_encoders"
#define IMU_CALIB_SRV "imu_calib"




//PINS
#define BATTERY_MONITOR_PIN A11

#define PAN_SERVO_PIN 2
#define TILT_SERVO_PIN 3

#define M1DIR_PIN 7
#define M2DIR_PIN 8
#define M1PWM_PIN 9
#define M2PWM_PIN 6
#define M1FB_PIN A0 //not in use
#define M2FB_PIN A1 //not in use
#define nD2_PIN 4
#define nSF_PIN 12
#define ENC1_A_PIN 30
#define ENC1_B_PIN 31
#define ENC2_A_PIN 32
#define ENC2_B_PIN 33

#define LEFT_URF_PIN A8
#define REAR_URF_PIN A9
#define RIGHT_URF_PIN A10
#define URF_TX_PIN 5

/*
LEFT MOTOR RED->M2B
LEFT MOTOR BLACK->M2A
LEFT ENCODER A -> ENC2_A_PIN = 32
LEFT ENCODER B -> ENC2_B_PIN = 33

RIGHT MOTOR RED->M1B
RIGHT MOTOR BLACK->M1A
RIGHT ENCODER A -> ENC1_A_PIN = 30
RIGHT ENCODER B -> ENC1_B_PIN = 31

GPS PIN 1 -> GND
GPS PIN 2 -> VCC
GPS PIN 3 -> RX2 = 17
GPS PIN 4 -> TX2 = 16

*/


#include <Wire.h>
#include "../libraries/I2Cdev/I2Cdev.h"
#include "../libraries/MPU9150Lib/MPU9150Lib.h"
#include "../libraries/CalLib/CalLib.h"
#include	"../DueFlash/DueFlash.h"  
#include	"../MotionDriver/dmpKey.h"		// ".dmpKey.h"
#include	"../MotionDriver/dmpmap.h" <dmpmap.h>
#include <../MotionDriver/inv_mpu.h>
#include <../MotionDriver/inv_mpu_dmp_motion_driver.h>
#include <Arduino.h>


//ROS

#include <ros.h>
#include <lizi/lizi_raw.h>
#include <lizi/lizi_gps.h>
#include <lizi/lizi_command.h>
#include <lizi/lizi_pan_tilt.h>
#include <lizi/lizi_status.h>

#include <lizi/imu_calib.h>
#include <std_srvs/Empty.h>

#define  SERIAL_PORT_SPEED  57600
#define PUB_RAW_INTERVAL 100 //10 hz
unsigned long urf_t = 0, enc_t = 0, status_t = 0, pub_t;

ros::NodeHandle nh;
using std_srvs::Empty;
using lizi::imu_calib;

//PROTOTYPES
void reset_encCb(const Empty::Request & req, Empty::Response & res);
void imu_calibCb(const imu_calib::Request & req, imu_calib::Response & res);
void commandCb( const lizi::lizi_command& msg);
void pantiltCb( const lizi::lizi_pan_tilt& msg);


ros::ServiceServer<imu_calib::Request, imu_calib::Response> imu_calib_server(IMU_CALIB_SRV, &imu_calibCb);
ros::ServiceServer<Empty::Request, Empty::Response> reset_enc_server(RESET_ENCODERS_SRV, &reset_encCb);

ros::Subscriber<lizi::lizi_command> command_sub(LIZI_COMMAND_TOPIC, &commandCb );

ros::Subscriber<lizi::lizi_pan_tilt> pan_tilt_sub(LIZI_PAN_TILT_TOPIC, &pantiltCb );


lizi::lizi_gps gps_msg;
lizi::lizi_raw raw_msg;
lizi::lizi_status status_msg;


ros::Publisher p_raw(LIZI_RAW_TOPIC, &raw_msg);
ros::Publisher p_gps(LIZI_GPS_TOPIC, &gps_msg);
ros::Publisher p_status(LIZI_STATUS_TOPIC, &status_msg);


//PAN TILT
#include <Servo.h>
#define MAX_PAN 35
#define MIN_PAN -35
#define MAX_TILT 30
#define MIN_TILT -30
#define PAN_CENTER 94
#define TILT_CENTER 94

Servo pan_servo;
Servo tilt_servo;
unsigned long pan_tilt_t = 0;
bool pan_tilt_moving = true;
#define PAN_TILT_MOVE_TIME 1000


//GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;
#define  GPS_PORT_SPEED  9600
#define GPS_SERIAL_PORT Serial2

//IMU
float qx = 0, qy = 0, qz = 1, qw = 0;
unsigned long imu_t = 0;
unsigned long CHECK_IMU_INTERVAL;
boolean imu_fault = 0;
//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69
#define  DEVICE_TO_USE    0
MPU9150Lib dueMPU; // the MPU object
//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE  (10)
//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE  (10)
//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 
//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE   40

long lastPollTime; // last time the MPU-9150 was checked
long pollInterval; // gap between polls to avoid thrashing the I2C bus
  char temp_msg[30];

int loopState; // what code to run in the loop

#define LOOPSTATE_NORMAL 0 // normal execution
#define LOOPSTATE_MAGCAL 1 // mag calibration
#define LOOPSTATE_ACCELCAL 2 // accel calibration

static CALLIB_DATA calData; // the calibration data

void magCalStart(void);
void magCalLoop(void);
void accelCalStart(void);
void accelCalLoop(void);




//BATTERY MONITOR
#define STATUS_INTERVAL 1000 //1 hz  
#define VOLTAGE_DIVIDER_RATIO 6.67


//DRIVER
#include <Encoder.h>
#include <PID_v1.h>
#include "../libraries/DualMC33926MotorShield/DualMC33926MotorShield.h"		//was "DualMC33926.h"

DualMC33926MotorShield md(M1DIR_PIN,  M1PWM_PIN,  M1FB_PIN, M2DIR_PIN, M2PWM_PIN,  M2FB_PIN, nD2_PIN,  nSF_PIN);
long left_enc = 0, right_enc = 0;
long pre_left_enc = 0, pre_right_enc = 0;
float left_spd = 0, right_spd = 0;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
#define READ_ENCODERS_INTERVAL 50 //200 hz
int  CONTROL_INTERVAL = 1; //ms
double DT;
#define  WATCHDOG_INTERVAL 1500 //ms
#define  MAX_TICKS_PER_S 100000 //tics/sec
boolean wd_on = false;
float alpha = 0.5;
float kp = 0.001, ki = 0.01, kd = 0;
PID PID1(&Input1, &Output1, &Setpoint1, kp, ki, kd, DIRECT);
PID PID2(&Input2, &Output2, &Setpoint2, kp, ki, kd, DIRECT);

Encoder Enc1(ENC1_A_PIN, ENC1_B_PIN);
Encoder Enc2(ENC2_A_PIN, ENC2_B_PIN);
unsigned long wd_t = 0, control_t = 0;

//URF
#define URF_INTERVAL 100 //10 hz
#define sample_size 5
#include "FastRunningMedian.h"
FastRunningMedian<unsigned int, sample_size, 0> Left_URF_Median;
FastRunningMedian<unsigned int, sample_size, 0> Right_URF_Median;
FastRunningMedian<unsigned int, sample_size, 0> Rear_URF_Median;



void setup()
{

  pinMode(13, OUTPUT);
  Serial.begin(SERIAL_PORT_SPEED);

  nh.initNode();

  nh.advertise(p_raw);
  nh.advertise(p_gps);
  nh.advertise(p_status);

  nh.advertiseService(reset_enc_server);
  nh.advertiseService(imu_calib_server);
  
  nh.subscribe(command_sub);
  nh.subscribe(pan_tilt_sub);

  while (!nh.connected()) {
    nh.spinOnce();
  }
  
  nh.loginfo("Starting up...");

  setup_driver();
  nh.loginfo("Driver ready");

  pan_tilt_setup();
  nh.loginfo("Pan Tilt ready");

  setup_imu();

  setup_urf();
  nh.loginfo("URF sensors ready");

  GPS_SERIAL_PORT.begin(GPS_PORT_SPEED);
  nh.loginfo("GPS ready");



}

void pub_raw() {

  raw_msg.qw = qw;
  raw_msg.qx = qx;
  raw_msg.qy = qy;
  raw_msg.qz = qz;

  raw_msg.left_ticks = -(long)left_enc;
  raw_msg.right_ticks = (long)right_enc;

  raw_msg.left_urf = (float)Left_URF_Median.getMedian() / 799.8124 ; //* 3.3 / 4095 *1.5515;
  raw_msg.rear_urf = (float)Rear_URF_Median.getMedian() / 799.8124 ; //* 3.3 / 4095 *1.5515;
  raw_msg.right_urf = (float)Right_URF_Median.getMedian() / 799.8124 ; //* 3.3 / 4095 *1.5515;

  p_raw.publish(&raw_msg);

}

void loop()
{

  control_loop();


  if (millis() - wd_t >= WATCHDOG_INTERVAL)  {
    if (!wd_on) {
      stop_motors();
      wd_on = true;
    }
  }


  if (millis() - status_t >= STATUS_INTERVAL)  {
    read_status();
    status_t = millis();
    digitalWrite(13, !digitalRead(13));
  }


  if (millis() - enc_t >= READ_ENCODERS_INTERVAL)  {
    read_encoders();
    enc_t = millis();
  }

  if (millis() - pub_t >= PUB_RAW_INTERVAL)  {
    pub_raw();
    pub_t = millis();
  }


  if (millis() - urf_t >= URF_INTERVAL)  {

    read_urf();
    //  nh.spinOnce();
    urf_t = millis();
  }

  read_gps();

  if (!imu_fault) {
    if (millis() - imu_t <= CHECK_IMU_INTERVAL)  {
      read_imu();
    }
    else {
      imu_fault = true;
    }
  }

  pan_tilt_wd();

  nh.spinOnce();
}






