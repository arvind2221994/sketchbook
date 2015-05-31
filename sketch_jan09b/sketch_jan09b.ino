/***********INCLUDES MAN_VEL CALLBACK AND NEW DIMENSIONS*******************/
#include "Arduino.h"
#include "math.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h> //I2C Arduino Library
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <litemsgs/lite.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <stdlib.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
/****************************************************************/
//FOR ROS
ros::NodeHandle nh;

litemsgs::lite odom;
ros::Publisher odo("odom_sub", &odom);
/***************************************************************/
#define rnpin 9
#define rppin 8
#define rspin 10
#define lnpin 12  //Motor Pins Interchanged @Jan 9 - 4.30 pm
#define lppin 13
#define lspin 11
#define encPRA 2  //ENCODER R A PIN
#define encPLA 3  //ENCODER L A PIN
#define encPRB 5  //ENCODER R B PIN
#define encPLB 4  //ENCODER L B PIN
#define B 843    // Breadth in cm
#define D 40    // Dia of Wheel in cm
#define C 251.32  // Circumference of wheel in cm
#define R 20    // Wheel Radius in cm
#define tprR 500  //ticks per rotation on R Encoder
#define tprL 500  //ticks per rotation on L Encoder
double dT;

double ntr,ntl,rtr,rtl,Rpsr,Rpsl,DistR,DistL,Dist, rwr,rwl,spr,spl;
volatile long plticks = 0;                                                      //Encoder increments or decrements this variable
volatile long prticks = 0;                                                      //Encoder increments or decrements this variable
volatile long oprticks = 0;                                                     //used in loop
volatile long oplticks = 0;                                                     //used in loop
volatile double lastOutputr = 0, lastErr_2r = 0, lastErrr = 0, Outputr = 0;      //PID variables
volatile double lastOutputl = 0, lastErr_2l = 0, lastErrl = 0, Outputl = 0;      //PID variables
volatile double lastOutputt = 0, lastErr_2t = 0, lastErrt = 0, Outputt = 0;      //PID variables
unsigned long lastTime = 0;     
double pose_x = 0,pose_y = 0,dx=0,dy=0;
double vel=0,velX=0,velY=0,velAngular=0;
double ang_velDesired = 0;//Time
float velDesired = 0;

float setvelocity=0;
float kpr = 2.75, kir = 0.00, kdr =-0.755 ;
float kpl = 1.47, kil = 0.00, kdl =-0.6494 ;

float Vx=0, Vy=0, vx=0, vy=0;

/*****For Theta Calculations**************/
double omega,theta = 0,theta_old = 0;
float initialheading = 0;

geometry_msgs::Twist vel_msg;
void velCallback(const geometry_msgs::Twist& vel_msg)
{
    vx = vel_msg.linear.x;
    vy = vel_msg.linear.y;
    ang_velDesired=vel_msg.angular.z;
     
    if(ang_velDesired == 0){
      Vx = vx;
      Vy = vy;
    }else{
      Vx = vx - (pose_y * 0.01 * ang_velDesired);
      Vy = vy + (pose_x * 0.01 * ang_velDesired);}    
    
      velDesired = sqrt(Vx*Vx + Vy*Vy); 
      
      if(Vx < 0){
        velDesired = - velDesired;
      }
     //  manvelCallback();
}

void manvelCallback()
{
    vx = 0.2;
    vy = 0.0;
    ang_velDesired=0.0;
     
    if(ang_velDesired == 0){
      Vx = vx;
      Vy = vy;
    }else{
      Vx = vx - (pose_y * 0.01 * ang_velDesired);
      Vy = vy + (pose_x * 0.01 * ang_velDesired);}    
    
    velDesired = sqrt(Vx*Vx + Vy*Vy);
    
     if(Vx < 0){
        velDesired = - velDesired;
     }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);

void setup(){
  //Start serial 
  nh.initNode();
  nh.advertise(odo);
  Serial.begin(57600);
  //Declare pin modes
  pinMode(rnpin,OUTPUT);  //Low RM - Negative/ground for Forward
  pinMode(lnpin,OUTPUT);  //Low LM - Negative/ground for Forward
  pinMode(rppin,OUTPUT);  //High RM - Positive for Forward
  pinMode(lppin,OUTPUT);  //High LM - Positive for Forward
  pinMode(rspin,OUTPUT);  //PWM RM
  pinMode(lspin,OUTPUT);  //PWM LM
  pinMode(encPRA,INPUT);  //Encoder r A pin
  pinMode(encPRB,INPUT);  //Encoder r B pin
  pinMode(encPLA,INPUT);  //Encoder l A pin
  pinMode(encPLB,INPUT);  //Encoder l B pin
 
  attachInterrupt(1,calRticks,RISING);  //ENCODER R A PIN 2
  attachInterrupt(0,calLticks,RISING);  //ENCODER L A PIN 3
  
  if(!mag.begin()){   
    while(1){
      //ros::rospy.logerr("debug");
      if(mag.begin())
      break;
      }
  }
  
  sensors_event_t event; 
  mag.getEvent(&event);
 
  float headin = atan2(event.magnetic.x, event.magnetic.y);
  
  if(headin < 0)
    headin += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(headin > 2*PI)
    headin -= 2*PI;
   
  // Convert radians to degrees for readability.
  initialheading = headin; 
  nh.subscribe(sub);
  nh.spinOnce();
  
}
  
void loop(){
  unsigned long now;
  now = millis();
  dT = (double)(now-lastTime)/1000;
  if(dT>=0.015){
    lastTime = now;

    //manvelCallback();
    
    ntr = prticks-oprticks;           // No.of tick on right during last dT 
    ntl = plticks-oplticks;           // No.of tick on left during last dT
 
    oprticks = prticks;                   //For next Step
    oplticks = plticks;
    
 /**********************ENCODERS***************************/

    setvelocity=velDesired * 100;
    omega = ang_velDesired;

    DistR = 2*PI*R*ntr/tprR;
    DistL = 2*PI*R*ntl/tprL;
    
    Dist = (DistR +DistL)/2;
    rwr = (2*setvelocity + omega*B)/D;                               
    rwl = (2*setvelocity - omega*B)/D;                               
    spr = rwr*dT*tprR/(2*PI);     // Set point (encoder ticks per rotation)
    spl = rwl*dT*tprL/(2*PI);     // Set point (encoder ticks per rotation)
    
    computeR(spr,ntr);  //PID Computation for Right wheel
    computeL(spl,ntl);  //PID Computation for Left wheel
    
<<<<<<< HEAD
    //Serial.println("R");
    //Serial.println(Outputr);
    //Serial.println("L");
    //Serial.println(Outputl);
=======
//    Serial.println("R");
//    Serial.println(Outputr);
//    Serial.println("L");
//    Serial.println(Outputl);
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
    
    motorR(Outputr); //Output Limited to pwm 155 @Jan 9 - 4.34 pm
    motorL(Outputl);
   
  /**********************COMPASS***************************/   
   
    sensors_event_t event; 
    mag.getEvent(&event);
    
    float headin = atan2(event.magnetic.x, event.magnetic.y);
    float declinationAngle = 0;
    headin += declinationAngle;
    if(headin < 0){
      headin += 2*PI;
    }
    // Check for wrap due to addition of declination.
    if(headin > 2*PI){
      headin -= 2*PI;
    }

   
    /********************************ODOMETRY CALCULATION********************************/
    theta = headin - initialheading;     
 
    dy= Dist*sin(theta);
    dx= Dist*cos(theta);
    
    velX=dx/dT;
    velY=dy/dT;
    
    pose_x+=dx;
    pose_y+=dy;

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionFromYaw((theta*PI)/180);
    odom.header.stamp = nh.now();
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.position.x = pose_x*0.01 ;
    odom.pose.position.y = pose_y*0.01 ;
    odom.pose.position.z = 0.0;
    odom.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.linear.x = velX*0.01;
    odom.twist.linear.y = velY*0.01;
    odom.twist.angular.x = 0;
    odom.twist.angular.y = 0;
    odom.twist.angular.z = ang_velDesired;

    //publish the message   //ROS Publisher is here
    odo.publish(&odom);
  }
  nh.spinOnce();
}

void computeR(double Setpoint, double Measured){
  double error = Setpoint - Measured;
  /*Compute PID Output*/
  
  //double correct=(kpr*(error) + kir*error*dT + (kdr*(error - 2*lastErrr + lastErr_2r) / dT));
  double correct=(kpr*(error) + kir*error*dT + (kdr*(error - 2*lastErrr + lastErr_2r) / dT));
 
  Outputr = correct+ lastOutputr;
  /*Max 255, Min -255*/
  if(Outputr>150){
    Outputr = 150;
  }
  else if(Outputr <-150){
    Outputr = -150;
  }
  /*Remember some variables for next time*/
  lastOutputr = Outputr;
  lastErr_2r = lastErrr;
  lastErrr = error;
}
  
void computeL(double Setpoint, double Measured){
  double error = Setpoint - Measured;
  /*Compute PID Output*/
  
  //double correct=(kpl*(error) + kil*error*dT + (kdl*(error - 2*lastErrl + lastErr_2l) / dT));
  double correct=(kpl*(error) + kil*error*dT + (kdl*(error - 2*lastErrl + lastErr_2l) / dT));
  
  Outputl = correct+ lastOutputl;
  /*Max 255, Min -255*/
  if(Outputl>150){
    Outputl = 150;
  }
  else if(Outputl <-150){
    Outputl = -150;
  }
  /*Remember some variables for next time*/
  lastOutputl = Outputl;
  lastErr_2l = lastErrl;
  lastErrl = error;
}

/*** Defining Interrupt functions ***/
  
void calRticks(){
  if(digitalRead(encPRB)==HIGH){
    prticks++;
  } else{
    prticks--;
  }
}

void calLticks(){
  if(digitalRead(encPLB)==HIGH){
    plticks++;
  } else{
    plticks--;
  }
}
  
  
void motorR(int output){
  if(output>0){
    digitalWrite(rppin,HIGH);
    digitalWrite(rnpin,LOW);
  }else {
    digitalWrite(rppin,LOW);
    digitalWrite(rnpin,HIGH);
  }
  analogWrite(rspin,(int) abs(output));
}
void motorL(int output){
  if(output>0){
    digitalWrite(lppin,HIGH);
    digitalWrite(lnpin,LOW);
  }else {
    digitalWrite(lppin,LOW);
    digitalWrite(lnpin,HIGH);
  }
  analogWrite(lspin,(int) abs(output));
}
