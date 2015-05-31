#include<Adafruit_Sensor.h>
#include<Adafruit_HMC5883_U.h>
#include"Arduino.h"
#include<math.h>
#include<Wire.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
/********DEFINE PINS********/
#define rnpin 9
#define rppin 8
#define rspin 10
#define lnpin 12 //Motor Pins Interchanged @Jan 9 - 4.30 pm
#define lppin 13
#define lspin 11
#define encPRA 3 //ENCODER R A PIN
#define encPLA 2 //ENCODER L A PIN
#define encPRB 5 //ENCODER R B PIN
#define encPLB 4 //ENCODER L B PIN
#define B 843 // Breadth in cm
#define D 40 // Dia of Wheel in cm
#define C 251.32 // Circumference of wheel in cm
#define R 20 // Wheel Radius in cm
#define tprR 500 //ticks per rotation on R Encoder
#define tprL 500 //ticks per rotation on L Encoder
const float pi =3.14;
double dT;
double rtr,rtl,Rpsr,Rpsl,DistR,DistL,omega=0,theta,theta_old = 0, rwr,rwl,spr,spl,ntl_avg=0,ntr_avg=0;
int navg=80,navgp=1, ntr[100],ntl[100];
volatile long plticks = 0;                                                      //Encoder increments or decrements this variable
volatile long prticks = 0;                                                      //Encoder increments or decrements this variable
volatile long oprticks = 0;                                                     //used in loop
volatile long oplticks = 0;                                                     //used in loop
volatile double lastOutputr[100],lastOutputr_avg=0, lastErr_2r = 0, lastErrr = 0, Outputr = 0;      //PID variables
volatile double lastOutputl[100],lastOutputl_avg=0, lastErr_2l = 0, lastErrl = 0, Outputl = 0;      //PID variables
volatile double lastOutputt[100],lastOutputt_avg=0, lastErr_2t = 0, lastErrt = 0, Outputt = 0;      //PID variables
unsigned long lastTime = 0;                                                      //Time
float setvelocity =40 , settheta = 0; //PLEA5555TE THAT SETTHETA IS IN DEGREES //set velocity in cm per second 
float kpr =0.1,kir = 0.34, kdr =0.0;
float kpl =0.1,kil = 0.5, kdl =0.0 ;
float kpt = 2, kit = 0, kdt = 0;  //FINAL VALUES AFTER TUNING 
float initialheading = 0;
float heading =0;


void setup(){
  Serial.begin(115200);
  pinMode(rnpin,OUTPUT);  //Low RM - Negative/ground for Forward
  pinMode(lnpin,OUTPUT);  //Low LM - Negative/ground for Forward
  pinMode(rppin,OUTPUT);  //High RM - Positive for Forward
  pinMode(lppin,OUTPUT);  //High LM - Positive for Forward
  pinMode(rspin,OUTPUT);  //PWM RM
  pinMode(lspin,OUTPUT);  //PWM LM
  pinMode(encPRA,INPUT);  //Encoder r A pin
  pinMode(encPRB,INPUT);  //Encoder r B pin
  pinMode(encPLA,INPUT);  //Encoder l A pin
  pinMode(encPLA,INPUT);  //Encoder l B pin
  // Use the following if pullup resistor is to be used
  
  digitalWrite(encPRA,HIGH);
   digitalWrite(encPRB,HIGH);
   digitalWrite(encPLA,HIGH);
   digitalWrite(encPLB,HIGH);
   
  //Attach interrupts to int0 and int1;

  attachInterrupt(0,calLticks,RISING);  //ENCODER L A PIN 2
  attachInterrupt(1,calRticks,RISING);
  for(int i=0;i<100;i++)
  {
    lastOutputr[i]=0;
  }
}

void loop(){
  unsigned long now;
  now = millis();
  dT = (double)(now-lastTime)/1000;

  if(dT>=0.01){
      lastTime = now;
      ntl[0] = plticks-oplticks;
      ntr[0] = prticks-oprticks;
for(int i=0;i<navg;i++)
{
  ntr_avg+=ntr[i];
  ntl_avg+=ntl[i];
}
ntr_avg=ntr_avg/navg;
ntl_avg=ntl_avg/navg;
    
      rtl = ntl_avg/dT;
      rtr = ntr_avg/dT;
      Rpsl = 2*pi*rtl/tprL;
      Rpsr = 2*pi*rtr/tprR;
//      Serial.print(Outputl);
//      Serial.print(",");
//      Serial.print(Outputr);
//      Serial.print(",");    
      rwr = (2*setvelocity + omega*B)/D;
      rwl = (2*setvelocity - omega*B)/D;
      spr = rwr*dT*tprR/(2*PI); // Set point (encoder ticks per rotation)
      spl = rwl*dT*tprL/(2*PI); // Set point (encoder ticks per rotation)

      Serial.print(Rpsl);
      Serial.print(",");
      Serial.print(Rpsr);
      Serial.print(",");
    Serial.println(rwr);
 
      computeL(rwl,Rpsl);
      computeR(rwr,Rpsr);
//      Serial.print(ntr_avg);
//      Serial.print(",");
//      Serial.println(ntl_avg);
      motorL(Outputl);
      motorR(Outputr);

    
    oplticks = plticks;
    oprticks = prticks;
   for(int i=navg-1;i>0;i--)
   {
     ntr[i]=ntr[i-1];
     ntl[i]=ntl[i-1];
  }
}
}
void computeL(double Setpoint, double Measured){
  double error = Setpoint - Measured;
  double correct=(kpl*(error) + kil*error*dT + (kdl*(error - 2*lastErrl + lastErr_2l) / dT));
  Outputl = (correct + lastOutputl[0]);
  if(Outputl>255){
    Outputl = 255;
  }
  else if(Outputl <-255){
    Outputl = -255;
  }
//  for(int i=navgp-1;i>=0;i--)
//  {
//    lastOutputl[i]=lastOutputl[i-1];
//  }
  lastOutputl[0]=Outputl;
//  for(int i=0;i<navgp;i++)
//  {
//    lastOutputl_avg+=lastOutputl[i];
//  }
//  lastOutputl_avg=lastOutputl_avg/navgp;
  lastErr_2l = lastErrl;
  lastErrl = error;
 // Serial.println(error);
}
void computeR(double Setpoint, double Measured){
  double error = Setpoint - Measured;
//  Serial.print(error);
//    Serial.print(",");
  //Serial.println(lastErrr);
  double correct=(kpr*(error) + kir*error*dT + (kdr*(error - 2*lastErrr + lastErr_2r) / dT));
  Outputr = (correct+lastOutputr[0]);
//  Serial.println(Outputr);
  if(Outputr>255){
    Outputr = 255;
  }
  else if(Outputr <-255){
    Outputr = -255;
  }
//  for(int i=navgp-1;i>0;i--)
//  {
//    lastOutputr[i]=lastOutputr[i-1];
//  }
  lastOutputr[0]=Outputr;
  
  
//  for(int i=0;i<20;i++)
//  {
//    lastOutputr_avg=lastOutputr_avg+lastOutputr[i];
//  }
   
  

  //lastOutputr_avg=lastOutputr_avg/navgp;
 //  Serial.println(lastOutputr_avg);
  lastErr_2r = lastErrr;
  lastErrr = error;
}

void calLticks()
{  
  if(digitalRead(encPLB)==HIGH){
    plticks++;
  } 
  else{
    plticks--;
  }
}
void calRticks()
{ 
  if(digitalRead(encPRB)==HIGH){
    prticks++;
  } 
  else{
    prticks--;
  }
}
void motorL(int output)
{
  if(output>0){
    digitalWrite(lppin,HIGH);
    digitalWrite(lnpin,LOW);
  }
  else
  {
    digitalWrite(lppin,LOW);
    digitalWrite(lnpin,HIGH);
  }    
  analogWrite(lspin,(int) abs(output));
}
void motorR(int output)
{
  if(output>0){
    digitalWrite(rppin,HIGH);
    digitalWrite(rnpin,LOW);
  }
  else
  {
    digitalWrite(rppin,LOW);
    digitalWrite(rnpin,HIGH);
  }    
  analogWrite(rspin,(int) abs(output));
}
