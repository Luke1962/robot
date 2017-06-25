/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include "math.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h> 
#include <sensor_msgs/Range.h>    // ultrasound
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <sensor_msgs/LaserScan.h>  // LDS


ros::NodeHandle  nh;

#define ROS_INFO(s) nh.loginfo(s);

//--------------------------------

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//--------------------------------

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

//--------------------------------
geometry_msgs::Twist msg;


//--------------------------------

sensor_msgs::LaserScan CDLaser_msg;
float f_angle_min;
float f_angle_max;
float f_angle_increment;
float f_time_increment;
float f_scan_time;
float f_range_min;
float f_range_max;
float f_ranges[5]; // max of 30 measurements
float f_intensities[5];

long lds_publisher_timer;

ros:: Publisher pub_Laser("scan", &CDLaser_msg);
//--------------------------------




float move1;
float move2;

const int adc_pin = 0;
double x = 1.0;
double y = 0.0;
double theta = 1.57;



char base_link[] = "/base_link";
char odom[] = "/odom";
char frameid[] = "/ultrasound";

float getRange_Ultrasound(int pin_num){
  int val = 0;
  for(int i=0; i<4; i++) val += analogRead(pin_num);
  float range =  val;
  return range /322.519685;   // (0.0124023437 /4) ; //cvt to meters
}

//-----------------------------------------------------------------
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> subMsg("toggle_led", messageCb );


//-----------------------------------------------------------------
void cmdvelCallBack(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x ;
  move2 = cmd_vel.angular.z ;
  
  x += cos(theta)*cmd_vel.linear.x *0.1;
  y += sin(cmd_vel.angular.z)*cmd_vel.linear.x *0.1;
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel",  cmdvelCallBack);
 
//-----------------------------------------------------------------

double g_req_angular_vel_z = 0;
double g_req_linear_vel_x = 0;
unsigned long g_prev_command_time = 0;

void commandCallback(const geometry_msgs::Twist& cmd_vel)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_vel.linear.x;
  g_req_angular_vel_z = cmd_vel.angular.z;

  g_prev_command_time = millis();

  theta += cmd_vel.angular.z;
  if(theta > 3.14)    theta=-3.14;
  x += cos(theta)*cmd_vel.linear.x *0.1;
  y += sin(theta)*cmd_vel.linear.x *0.1;


  ROS_INFO("cmd_vel");
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

//-----------------------------------------------------------------

std_msgs::String str_msg;
ros::Publisher chatter("/chatter", &str_msg);
ros::Publisher speech("/rp/state_externalization/vocal_message", &str_msg);

char hello[11] = "ciao bello";
char ros_info[30] ;
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  broadcaster.init(nh);
  
  nh.advertise(chatter);
  nh.advertise(speech);
  nh.advertise(pub_range);
  
  nh.subscribe(subMsg);
  nh.subscribe(subCmdVel);
  nh.subscribe(cmd_sub);
  
  // ultrasound setup-------------------
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  
  pinMode(8,OUTPUT);
  digitalWrite(8, LOW);
  
  //-----------------------------
  nh.advertise(pub_Laser);

  f_angle_min = -1.57;
  f_angle_max = 1.57;
  f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
  f_time_increment = 10;
  f_scan_time = 4;
  f_range_min = 0.1;
  f_range_max = 30;

  CDLaser_msg.ranges_length = 5;
  CDLaser_msg.intensities_length = 5;

  // create the test data
  for (int z = 0 ; z<5; z++)
  {
    f_ranges[z] = z;
    f_intensities[z] = z*z;
  }
   //-----------------------------
 ROS_INFO( "ROBOT SETUP COMPLETE");
  nh.spinOnce();
}
long i=0;
long range_time;
long speech_time;

void loop()
{

/*
 // drive in a circle---------
  double dx = 0.02;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
  //---------------------------
 */
    
  // broadcast tf odom->base_link-------------
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();
  //------------------------------------------
 
  if ( millis() >= speech_time ){  
    // pubblicazione su chatter e speech-------
    str_msg.data = hello;
    chatter.publish(&str_msg );
    speech.publish(&str_msg ); 
    ROS_INFO( "speech");

    speech_time =  millis() + 5000;    
    nh.spinOnce();
   }
  
//  str_msg.data = itoa(i,str_msg,5);   
//  speech.publish(&str_msg ); 



  //publish ULTRASOUND
  if ( millis() >= range_time ){
    int r =0;

    range_msg.range = getRange_Ultrasound(5);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
    nh.spinOnce();
  }


if (millis() > lds_publisher_timer)
  {
    CDLaser_msg.header.stamp = nh.now();
    CDLaser_msg.header.frame_id = "laser_frame";
    CDLaser_msg.angle_min = f_angle_min;
    CDLaser_msg.angle_max = f_angle_max;
    CDLaser_msg.angle_increment = f_angle_increment;
    CDLaser_msg.time_increment = f_time_increment;
    CDLaser_msg.scan_time = f_scan_time;
    CDLaser_msg.range_min = f_range_min;
    CDLaser_msg.range_max = f_range_max;

    for (int z = 0 ; z<5; z++)
    {
      CDLaser_msg.ranges[z] = f_ranges[z];
    }

    for (int z = 0 ; z<5; z++)
    {
      CDLaser_msg.intensities[z] = f_intensities[z];
    }

    lds_publisher_timer = millis() + 3000;
    pub_Laser.publish(&CDLaser_msg);
    nh.spinOnce();
  }  


}