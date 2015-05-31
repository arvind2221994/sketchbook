//Interfacing with another Due
#define USE_USBCON

#include <Wire.h>
//#include <Motor.h>
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


//#include <sstream>
#include <string>

//FOR ROS
ros::NodeHandle nh;

litemsgs::lite odom;
ros::Publisher odo("odom_sub", &odom);
//Defining variables for class motor
#define B 84.3
#define C 251.32  // Circumference of wheel in cm
#define R 20    // Wheel Radius in cm
#define TPR 10000
double dT;

float rpm[2];
double velocity1, velocity2; double pose_x = 0, pose_y = 0, dx = 0, dy = 0, dth = 0;
double vel = 0, velX = 0, velY = 0, velTh = 0;
float setvelocity = 0;

float Vx = 0, Vy = 0, vx = 0, vy = 0;
double ang_velDesired = 0;//Time
float velDesired = 0;
double omega, theta = 0, theta_old = 0;
float initialheading = 0, headin = 0;
unsigned long now;
double lastTime = 0;


int ticks1, ticks2;
int oticks1 = 0, oticks2 = 0;
double ntr, ntl, DistR, DistL, Dist;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);



class Motor
{
    unsigned int br_pin, dir_pin, pwm_pin, en_pin;
    boolean aclk;

  public:
    int reset;
    Motor();
    Motor(unsigned int pin1, unsigned int pin2, unsigned int pin3, boolean stat1)
    {
      reset = 0;
      pwm_pin = pin1;
      en_pin = pin2;
      dir_pin = pin3;
      aclk = stat1;


      // pinMode(br_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT);
      pinMode(pwm_pin, OUTPUT);
      pinMode(en_pin, OUTPUT);

      digitalWrite(en_pin, HIGH);
      //digitalWrite(br_pin, LOW);
      digitalWrite(dir_pin, HIGH);
      analogWrite(pwm_pin, 30);
    }

    void enable(boolean en)
    {
      digitalWrite(en_pin, en);
    }

    void motor_move(int vel)
    {
      //brake(LOW);
      int val;
      if (vel > 0)
        digitalWrite(dir_pin, aclk);
      else if (vel < 0)
        digitalWrite(dir_pin, !aclk);
      val = constrain(abs(vel), 0, 200);
      val = map(val, 0, 107.2, 25, 229);
      analogWrite(pwm_pin, val);
    }

    //  void brake(boolean state, int strength=320)
    //  {
    //    enable(HIGH);
    //    int str = map(abs(strength), 0, 360, 25, 225);
    //    digitalWrite(br_pin, !state);
    //    analogWrite(pwm_pin,str);
    //  }

};

//Due QEI
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);

Motor motors[2] = {Motor(9, 23, 25, 0), Motor(8, 22, 24, 0)}; //motors 1, 2

//motors[4] = { Motor(8, 23, 25, 27, 0), Motor(9, 22, 24, 26, 1)}; //motors 1, 2

/*-----------------------
 motor 1 - 8, 23, 25, 27 --- 8 - pwm, 23 - enable, 25 - dir, 27 - brake
 motor 2 - 9, 22, 26, 27 --- 9 - pwm, 22 - enable, 24 - dir, 26 - brake
  -------------------------*/

geometry_msgs::Twist vel_msg;
void velCallback(const geometry_msgs::Twist& vel_msg)
{
  vx = vel_msg.linear.x;
  ang_velDesired = vel_msg.angular.z;

  Vx = vx;

  if (Vx < 0) {
    velDesired = - Vx;
  }
}

void manvelCallback()
{
  vx = 0.10;
  vy = 0.0;
  ang_velDesired = 0.0;

  Vx = vx;

  if (Vx < 0) {
    velDesired = - Vx;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);
void bot_motion(float v, float w)
{
  //assuming v in input is cm/sec
  rpm[0] = (v + w * B) * 60 / (2 * PI) / R;
  rpm[1] = (v - w * B) * 60 / (2 * PI) / R;
  printr("RPM-L : " + String(rpm[0]));
  printr("RPM-R : " + String(rpm[1]));

  for (int i = 0; i < 2; i++)
    motors[i].motor_move(rpm[i]);

}

void printr(String s){
   nh.loginfo(s.c_str());
}

//void bot_brake(int str)
//{
//  for(int i=0;i<4;i++)
//    motors[i].brake(HIGH, str);
//} 



void setup()
{
  nh.initNode();
  nh.advertise(odo);
  motors[0].enable(HIGH);
  motors[1].enable(HIGH);


  Wire.begin();  // join i2c bus (address optional for master)
  delay(250);
  Serial.begin(115200);   ///////////////57600
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
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (0 << 12); //we are detecting only half resolution
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;


  if (!mag.begin()) {
    while (1) {
      //ros::rospy.logerr("debug");
      if (mag.begin())
        break;
    }
  }

  sensors_event_t event;
  mag.getEvent(&event);

  headin = atan2(event.magnetic.x, event.magnetic.y);

  if (headin < 0)
    headin += 2 * PI;

  // Check for wrap due to addition of declination.
  if (headin > 2 * PI)
    headin -= 2 * PI;

  // Convert radians to degrees for readability.
  initialheading = headin;

  nh.subscribe(sub);
  nh.spinOnce();
}

void loop()
{

  now = millis();
  dT = (double)(now - lastTime) / 1000;
  if (dT >= 0.015) {

    lastTime = now;
    oticks1 = ticks1;
    oticks2 = ticks2;

    ticks1 = -(int)REG_TC0_CV0;

    //manvelCallback();

    //  Serial.print(REG_TC0_CV0,DEC);
    //    Serial.print(ticks1);
    //    Serial.print(",");
    //delay(500);

    Wire.requestFrom(2, 9);    // request 6 bytes from slave device #2
    String a;
    while (Wire.available())   // slave may send less than requested
    {
      //Serial.print("c");
      char c = Wire.read(); // receive a byte as character
      if (c != 's') {
        a += c;
        // print the character
      }
      else {

        break;
      }
    }
    //Serial.println(a);
    //Serial.println(a);
    ticks2 = atoi(a.c_str());
    setvelocity = velDesired * 100;
    omega = ang_velDesired;
    bot_motion(setvelocity,omega);
    
    
    ntl = ticks1 - oticks1;  // l,r doubt - Cleared!
    ntr = ticks2 - oticks2;
    DistR = 2 * PI * R * ntr / TPR; // 10000 ticks per rotation
    DistL = 2 * PI * R * ntl / TPR;
    Dist = (DistR + DistL) / 2;

         printr("velocity : " +String(setvelocity));
         printr("ang-velocity : " +String(omega));

    /**********************COMPASS***************************/

    sensors_event_t event;
    mag.getEvent(&event);

    headin = atan2(event.magnetic.x, event.magnetic.y);
    float declinationAngle = 0;
    headin += declinationAngle;
    if (headin < 0) {
      headin += 2 * PI;

    }
    // Check for wrap due to addition of declination.
    if (headin > 2 * PI) {
      headin -= 2 * PI;
    }

    theta = headin - initialheading;

    float headingDegrees = headin * 180 / M_PI;
    //    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    //    Serial.println(theta);
    //    Serial.println(headin);
    //    Serial.println();

    //velTh = (theta-theta_old)/dT;
    //theta_old = theta;
    velTh =  omega;

    dy = Dist * sin(theta);
    dx = Dist * cos(theta);

    velX = dx / dT;
    velY = dy / dT;

    pose_x += dx;
    pose_y += dy;


    ////////////////////////////////////////////////////////////////////////////////
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionFromYaw((theta * PI) / 180);
    odom.header.stamp = nh.now();
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.position.x = pose_x * 0.01 ;
    odom.pose.position.y = pose_y * 0.01 ;
    odom.pose.position.z = 0.0;
    odom.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.linear.x = velX * 0.01;
    odom.twist.linear.y = velY * 0.01;
    odom.twist.angular.x = 0;
    odom.twist.angular.y = 0;
    odom.twist.angular.z = velTh;

    //publish the message   //ROS Publisher is here
    odo.publish(&odom);

//    Serial.print(" ticks1 ");
//    Serial.print(ticks1);
//
//    Serial.print(",");
//
//    Serial.print(" ticks2 ");
//    Serial.println(ticks2);
//
//    Serial.print(",");
//
//    Serial.print(" pose_x ");
//    Serial.print(pose_x);
//
//    Serial.print(",");
//
//    Serial.print(" pose_y ");
//    Serial.println(pose_y);

//    delay(500);
//
//    bot_motion(20, 0);


    
  } //velocity in cm/sec && angular velocity in radians/sec;
  nh.spinOnce();
}


