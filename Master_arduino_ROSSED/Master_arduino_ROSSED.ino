//Interfacing with another Due
#define USE_USBCON

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>

#include <litemsgs/lite.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <stdlib.h>
//#include "Motor.cpp"

//#include <sstream>
#include <string>

//FOR ROS
ros::NodeHandle nh;

litemsgs::lite odom;
ros::Publisher odom_pub("odom_sub", &odom);
//Defining variables for class motor
#define B 84.3
#define C 251.32  // Circumference of wheel in cm
#define R 20    // Wheel Radius in cm
#define TPRL 21504  //Ticks per Rotation
#define TPRR  21504

#define ROS_ON 
#define MOTOR_HIGH 
#define COMPASS_ON
#define DEBUG_LEVEL 3
double dT;

float rpm[2];
double pose_x = 0, pose_y = 0, dx = 0, dy = 0, dth = 0;
double velX = 0,velY = 0,velTh = 0; //Global frame
float vx = 0; //Input velocity from ROS
double ang_velDesired = 0,velDesired = 0; //Deduced velocities from ROS
double omega= 0,setvelocity = 0; //As given to bot (motors)
double theta = 0, theta_old = 0; 
float initialheading = 0, headin = 0; // For compass
unsigned long now,lastTime = 0;

int ticksl = 0,init_ticksr = 0, ticksr = 0,oticksl = 0,oticksr = 0;
double dtr, dtl;
double DistR, DistL, Dist;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#include <Arduino.h>

void printr(String s, int level) {
#ifdef ROS_ON
  if (level >= DEBUG_LEVEL)
    nh.loginfo(s.c_str());
#else
  if (level >= DEBUG_LEVEL)
    Serial.println(s);
#endif
}

class Motor {
    unsigned int br_pin, dir_pin, pwm_pin, en_pin;
    boolean cclk;
  public:
    int reset;
    Motor();
    Motor(unsigned int pin1, unsigned int pin2, unsigned int pin3, boolean stat1) {
      reset = 0;
      //br_pin = pin0;
      pwm_pin = pin1;
      en_pin = pin2;
      dir_pin = pin3;
      cclk = stat1;
      
      //pinMode(br_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT);
      pinMode(pwm_pin, OUTPUT);
      pinMode(en_pin, OUTPUT);

      //digitalWrite(br_pin, HIGH);
      digitalWrite(en_pin, HIGH);
      digitalWrite(dir_pin, HIGH);
      analogWrite(pwm_pin, 30);
    }

    void enable(boolean en) {
      digitalWrite(en_pin, en);
    }

    void motor_move(int vel) {
      //brake(LOW);
      int val;
      if (vel > 0)
        digitalWrite(dir_pin, cclk);
      else if (vel < 0)
        digitalWrite(dir_pin, !cclk);
      val = constrain(abs(vel), 0, 107.2);
      val = map(val, 0, 107.2, 25, 229);
      analogWrite(pwm_pin, val);
    }
};

//Due QEI
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);

Motor motors[2] = {Motor(9, 23, 25, 0), Motor(8, 22, 24, 0)}; //motors 1 (l), 2 (r)

geometry_msgs::Twist vel_msg;
void velCallback(const geometry_msgs::Twist& vel_msg) {
  vx = vel_msg.linear.x;
  ang_velDesired = vel_msg.angular.z;
  velDesired = vx;
}

void manvelCallback() {
  vx = 0.25;
  ang_velDesired = 0;
  velDesired = vx;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);

void bot_motion(float v, float w) {
  //assuming v in input is cm/sec
  rpm[0] = (v - w * B) * 60 / (2 * PI) / R; //Left motor
  rpm[1] = (v + w * B) * 60 / (2 * PI) / R; //Right motor
  printr("RPM-L : " + String(rpm[0]), 0);
  printr("RPM-R : " + String(rpm[1]), 0);
  
  for (int i = 0; i < 2; i++)
    motors[i].motor_move(rpm[i]);

}

void initMotors(){
  #ifdef MOTOR_HIGH
    for (int i = 0; i < 2; i++)
      motors[i].enable(HIGH);
  #else
    for (int i = 0; i < 2; i++)
      motors[i].enable(LOW);
  #endif
}

float compassData(){
  /********************** Compass *******************/
  #ifdef COMPASS_ON
  if (!mag.begin()) {
    while (1) {
      if (mag.begin())
        break;
    }
  }
  sensors_event_t event;
  mag.getEvent(&event);
  float h = atan2(event.magnetic.x, event.magnetic.y);
  if (h < 0)
    h += 2 * PI;
  if (h > 2 * PI)
    h -= 2 * PI;
  
  return h;
  #endif
  return 0;
  /************************************************/
}

void setup() {

  Serial.begin(115200);
  Serial3.begin(9600);
  
  initMotors();
  
  delay(500);
  /*---------Enabling Registers for QEI-----------------*/
  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B
  REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B
  // activate clock for TC0
  REG_PMC_PCER0 = (1 << 27);
  // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5;
  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (1 << 12); //we are detecting only half resolution
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  /**************************************************/  
  
  initialheading = compassData();
  
  /**********************Intial ticks from slave***************************/
  Serial3.flush();
  String  a = Serial3.readStringUntil('s');
  init_ticksr = atoi(a.c_str());
  /******************************************************/
  

  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(sub);
  nh.spinOnce();
}

void loop(){

  now = millis();
  dT = (double)(now - lastTime) / 1000;

 Serial.println("out");
  if (dT >= 0.015) {
    lastTime = now;  
    oticksl = ticksl;
    oticksr = ticksr;
    ticksl = -(int)REG_TC0_CV0;


  /**********************SLAVE***************************/
  Serial3.flush();
  String  a = Serial3.readStringUntil('s');
  ticksr = atoi(a.c_str());

  /******************************************************/
    printr("ticksl : " + String(ticksl), 3);
    printr("ticksr : " + String(ticksr), 3);

    //manvelCallback();

    dtl = ticksl - oticksl;  
    dtr = ticksr - oticksr;
    DistR = 2 * PI * R * dtr / TPRR; // 10000 ticks per rotation
    DistL = 2 * PI * R * dtl / TPRL;
    Dist = (DistR + DistL) / 2;

    printr("dtl : " + String(dtl), 1);
    printr("dtr : " + String(dtr), 1);
    printr("Distr : " + String(DistL), 1);
    printr("Distl : " + String(DistR), 1);
    printr("velocity : " + String(setvelocity), 0);
    printr("ang-velocity : " + String(omega), 0);


    /**********************COMPASS***************************/
    theta = compassData() - initialheading;
    if(theta<0){
      theta +=2*PI;
    }
    /********************************************************/

    //velTh =  omega; // To debug ROS
    velTh = (theta - theta_old)/dT;
    theta_old = theta;
    
    printr("theta : " + String(theta*180/PI), 3);
    dy = Dist * sin(theta);
    dx = Dist * cos(theta);
    velX = dx / dT;
    velY = dy / dT;
    pose_x += dx;
    pose_y += dy;
    printr("pose_x : " + String(pose_x), 0);
    printr("pose_y : " + String(pose_y), 0);

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionFromYaw(theta);
    odom.header.stamp = nh.now();
    odom.header.frame_id = "odom";
    //Feedback position
    odom.pose.position.x = pose_x * 0.01 ;
    odom.pose.position.y = pose_y * 0.01 ;
    odom.pose.position.z = 0.0;
    odom.pose.orientation = odom_quat;
    //Feedback velocity
    odom.child_frame_id = "base_link";
    odom.twist.linear.x = velX * 0.01;
    odom.twist.linear.y = velY * 0.01;
    odom.twist.angular.x = 0;
    odom.twist.angular.y = 0;
    odom.twist.angular.z = velTh;

    //publish the message - ROS Publisher is here
    odom_pub.publish(&odom);
    
    setvelocity = velDesired * 100; //Velocity in cm/sec
    omega = ang_velDesired; //Angular velocity in radians/sec
    bot_motion(setvelocity, omega);
  } 
  nh.spinOnce();
}
