#include <PID_v1.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <litemsgs/lite.h>
#include <tf/tf.h>
#include <stdlib.h>
//ROS shit
ros::NodeHandle nh;
litemsgs::lite odom;
ros::Publisher odo("gagan", &odom);

unsigned long int encoderR = 4; // Set the encoder pin numbers here.
unsigned long int encoderL = 5;
volatile long encoderRPos = 0; // Volatile is used since this variable will be used by...
volatile long encoderLPos = 0; // ....Interrupt function 
long newpositionR;
long newpositionL;
long oldpositionR = 0;
long oldpositionL = 0;
unsigned long newtime = 0;
unsigned long oldtime = 0;
int tpsR=0,tpsR_temp=0;
int tpsL=0,tpsL_temp=0;
int sizeMAR=3; //MA=Moving Average
int sizeMAL=3;
int arrayMAR[3]={0};
int arrayMAL[3]={0};
int moving_sumR=0;
int moving_sumL=0;
int i=0;
//Variables used for the PID setup.
float rpmSetpointR=0, rpmlInputR=0, rpmOutputR_temp=0;
unsigned int velOutputR=0;
float rpmSetpointL=0, rpmInputL=0, rpmOutputL_temp=0;
unsigned int rpmOutputL=0;
float thetaSetpointR=0, thetaInputL=0, thetaOutputL_temp=0;
unsigned int thetaOutputL=0;
PID myPID_R(&InputR, &OutputR_temp, &SetpointR,2,0,0, DIRECT);
PID myPID_L(&InputL, &OutputL_temp, &SetpointL,2,0,0, DIRECT);
PID myPID_theta(&InputL, &OutputL_temp, &SetpointL,2,0,0, DIRECT);
float velDesired=0;
float maxVel=0;
float radius=0;
float delta_theta=0,theta=0;
float botWid = 0.39; // Width of the bot in m
float wheelCir = 0.44; // Circumference in m
int tpr = 500; // Ticks per rotation
int maxRpm = 60; // Change it by testing on required terrain
float dist_R = 0,dist_L = 0,dist_baseline = 0.65,dist_centre=0; //in meter. 
float pose_x = 0,pose_y = 0;
float vel=0,velX=0,velY=0,velAngular=0;

geometry_msgs::Twist vel_msg;
void velCallback(const geometry_msgs::Twist& vel_msg)
{
    Vx=vel_msg.linear.x;
    Vy=vel_msg.linear.y;
    if (Vx == 0 && Vy == 0) // Finds theta desired
      thetaDesired = 0;
    if (Vx == 0 && Vy > 0)
      thetaDesired = 1.57;
    if (Vx == 0 && Vy < 0)
      thetaDesired = -1.57;
    if (Vx > 0 && Vy == 0)
      thetaDesired = 0;
    if (Vx < 0 && Vy == 0)
      thetaDesired = 3.1415926535;
    if (Vx>0&&Vy>0)
      {
        thetaDesired = atan(Vy/Vx);
      }
    
    if (Vx<0&&Vy<0)
      {
        thetaDesired = atan(Vy/Vx);
        thetaDesired = thetaDesired - 3.1415926535;
      }
    if (Vx<0&&Vy>0)
      {
        thetaDesired = atan(-Vy/Vx);
        thetaDesired = 3.1415926535-thetaDesired;
      }
    if (Vx>0&&Vy<0)
      {
        thetaDesired = atan(Vy/Vx);
      }
    velDesired = sqrt(Vx*Vx + Vy*Vy); 
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);
void setup() {
  
  nh.initNode();
  nh.advertise(odo);  
  Serial.begin (57600);
  //Serial.println("Initializing....");
  attachInterrupt(0, doEncoderR, RISING);  // encoder on pin2; doEncoderR (ISR for right wheel encoder)
  attachInterrupt(1, doEncoderL, RISING);  // encoder on pin3; doEncoderR (ISR for left wheel encoder)
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT); // PWM pin
  pinMode(11,OUTPUT); // PWM pin
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);  
  pinMode(encoderR,INPUT);
  pinMode(encoderL,INPUT);
  nh.subscribe(sub);
  nh.spinOnce();
  //Initialize the PID 
  myPID_R.SetMode(AUTOMATIC);
  myPID_L.SetMode(AUTOMATIC);
  Serial.println("Initialization Complete.");    // a personal quirk
  // Maximum velocity of the bot-> constrained due to motor rpm
  maxVel = (maxRpm * wheelCir/60);
  // put your setup code here, to run once:
}

void loop() {
  
  newtime = millis();
  newpositionR = encoderRPos;
  newpositionL = encoderLPos;
    //For DEBUGGING...
     //Serial.println(encoderRPos);
     //Serial.println(encoderLPos);
  tpsR_temp = ((newpositionR-oldpositionR) * 1000.0) / (newtime-oldtime);
  tpsL_temp = ((newpositionL-oldpositionL) * 1000.0) / (newtime-oldtime);
  for(i=(sizeMAR-1);i>=1;i--){
  arrayMAR[i]=arrayMAR[i-1];}      
  arrayMAR[0]=tpsR_temp;          
  for(i=(sizeMAL-1);i>=1;i--){
      arrayMAL[i]=arrayMAL[i-1];}      
  arrayMAL[0]=tpsL_temp;
  for(i=0; i<sizeMAR; i++)
    {moving_sumR += arrayMAR[i];}
  tpsR = (moving_sumR/sizeMAR);
  moving_sumR = 0;
  for(i=0; i<sizeMAL; i++)
    {moving_sumL += arrayMAL[i];}
  tpsL = (moving_sumL/sizeMAL);
  moving_sumL = 0;
  dist_R =  (newpositionR-oldpositionR)*wheelCir/500;
  dist_L =  (newpositionL-oldpositionL)*wheelCir/500;
  dist_centre = (dist_R + dist_L)/2;
  vel = (tpsR+tpsL)*wheel_dia*3.1415926535/500;
  delta_theta = (dist_R-dist_L)/botWid;
  theta = theta + delta_theta;
  pose_x = pose_x + dist_centre*cos(theta);
  pose_y = pose_y + dist_centre*sin(theta);
  velX = vel*cos(theta);
  velY = vel*sin(theta);
  velAngular = 1000*(dist_R-dist_L)/(dist_baseline*(newtime-oldtime));
  oldpositionR = newpositionR;
  oldpositionL = newpositionL;
  oldtime = newtime;
  
  

}
