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

//#define ROS_ON
#define MOTOR_HIGH
#define COMPASS_ON
#define DEBUG_PRIORITY  6 //Refer to Debug->section F
/*
>> 0 - dtl,dtr,DistL,DistR
>> 1 - RPM-L,RPM-R (Set)
>> 2 - setvelocity,omega
>> 3 - pose_x,pose_y,theta
>> 4 - ticksl,ticksr,theta
>> 5 - RPMR,RPML (Feedback)
>> 6 - theta,error
*/
double dT;
float rpm[2];
#define THETA_SIZE 10
float thetaArr[THETA_SIZE];
float thetaAvg;

double pose_x = 0, pose_y = 0, dx = 0, dy = 0, dth = 0;
double velX = 0, velY = 0, velTh = 0; //Global frame
float vx = 0; //Input velocity from ROS
double ang_velDesired = 0, velDesired = 0; //Deduced velocities from ROS
double omega = 0, setvelocity = 0; //As given to bot (motors)
double theta = 0, theta_old = 0;
float initialheading = 0, headin = 0; // For compass
unsigned long now, lastTime = 0;

int ticksl = 0, init_ticksr = 0, ticksr = 0, oticksl = 0, oticksr = 0;
double dtr, dtl;
double DistR, DistL, Dist,  RPMR, RPML;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#include <Arduino.h>

void printr(String s, int level) {
#ifdef ROS_ON
  if (level == DEBUG_PRIORITY)
    nh.loginfo(s.c_str());
#else
  if (level == DEBUG_PRIORITY)
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
      analogWrite(pwm_pin, 25 );
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

Motor motors[2] = {Motor(9, 23, 25, 1), Motor(8, 22, 24, 0)}; //motors 1 (L), 2 (R)


//Due QEI
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);

geometry_msgs::Twist vel_msg;
void velCallback(const geometry_msgs::Twist& vel_msg) {
  vx = vel_msg.linear.x;
  ang_velDesired = vel_msg.angular.z;
  velDesired = vx;
}

void manvelCallback() {
  vx = 0.30;
  ang_velDesired = 0;
  velDesired = vx;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);

void bot_motion(float v, float w) {
  //assuming v in input is cm/sec
  rpm[0] = (v - w * B) * 60 / (2 * PI) / R; //Left motor
  rpm[1] = (v + w * B) * 60 / (2 * PI) / R; //Right motor
  printr("RPM-L : " + String(rpm[0]), 1);
  printr("RPM-R : " + String(rpm[1]), 1);

  for (int i = 0; i < 2; i++)
    motors[i].motor_move(rpm[i]);

}

void initMotors() {
#ifdef MOTOR_HIGH
  for (int i = 0; i < 2; i++)
    motors[i].enable(HIGH);
#else
  for (int i = 0; i < 2; i++)
    motors[i].enable(LOW);
#endif
}

float compassData() {
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

  initMotors();

  //Motor motors[2] = {Motor(9, 23, 25, 1), Motor(8, 22, 24, 0)}; //motors 1 (L), 2 (R)
  digitalWrite(25, HIGH);

  nh.initNode();
  nh.advertise(odom_pub);
  Serial.begin(115200);
  Serial3.begin(9600);

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
  String a;
  while (!Serial3.available());
  a = Serial3.readStringUntil('s');
  init_ticksr = atoi(a.c_str());
  /******************************************************/
  
  for(int i=0;i<THETA_SIZE;i++)
    thetaArr[i] = 0.0;
  thetaAvg = 0.0;

  nh.subscribe(sub);
  nh.spinOnce();
  printr("setup done ", 4);
}

void loop() {

  now = millis();
  dT = (double)(now - lastTime) / 1000;
  if (dT >= 0.015) {
    lastTime = now;
    oticksl = ticksl;
    oticksr = ticksr;
    ticksl = -(int)REG_TC0_CV0 ;


    /**********************SLAVE***************************/
    String  a = Serial3.readStringUntil('s');
    ticksr = atoi(a.c_str()) - init_ticksr;
    /******************************************************/

    printr("ticksl : " + String(ticksl) + " ticksr : " + String(ticksr), 4);

    manvelCallback();

    dtl = ticksl - oticksl;
    dtr = ticksr - oticksr;
    DistR = 2 * PI * R * dtr / TPRR; // 10000 ticks per rotation
    DistL = 2 * PI * R * dtl / TPRL;
    Dist = (DistR + DistL) / 2;
    RPMR = (dtr / (TPRR * dT)) * 60 ; // 10000 ticks per rotation
    RPML = (dtl / (TPRL * dT)) * 60;

    printr("dtl : " + String(dtl) + "dtr : " + String(dtr), 0);
    printr("Distr : " + String(DistL) + "Distl : " + String(DistR), 0);
    printr("velocity : " + String(setvelocity) + " , ang-velocity : " + String(omega), 2);
    printr("RPMR : " + String(RPMR) + " RPML : " + String(RPML), 5 );

    /**********************COMPASS***************************/
    theta = compassData() - initialheading;
    if (theta < 0)
      theta += 2 * PI;
    /********************************************************/

    //velTh =  omega; // To debug ROS
    velTh = (theta - theta_old) / dT;
    theta_old = theta;

    //theta = -theta; //Subject to use of compass1

    printr("theta : " + String(theta * 180 / PI), 4);

    dy = Dist * sin(theta);
    dx = Dist * cos(theta);
    velX = dx / dT;
    velY = dy / dT;
    pose_x += dx;
    pose_y += dy;
    printr("pose_x,pose_y,theta : " + String(pose_x) + "," + String(pose_y) + ","  + String(theta), 3);

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
    
    thetaAvg  -= (thetaArr[0]/THETA_SIZE);
    for(int i=1;i<THETA_SIZE;i++){
      thetaArr[i-1]=thetaArr[i];
    }
    thetaArr[THETA_SIZE-1]=theta;
    thetaAvg += (theta/THETA_SIZE);

    //publish the message - ROS Publisher is here
    odom_pub.publish(&odom);

    setvelocity = velDesired * 100; //Velocity in cm/sec
    float settheta = ang_velDesired * dT; //Theta ref. from ROS's angular velocity in radians/sec

    omega = PID(settheta * 180 / PI, thetaAvg* 180 / PI, dT);
    bot_motion(setvelocity, omega);
    //bot_motion(-25,0);
  }
  nh.spinOnce();
}
