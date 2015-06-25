////Interfacing with another Due
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
double theta = 0, theta_old = 0;
float initialheading = 0, headin = 0;
int ticks1= 0,ticks2= 0,oticks1 = 0,oticks2 = 0;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//Due QEI
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);
void setup()
{
  Wire.begin();  // join i2c bus (address optional for master)
  delay(250);
  Serial.begin(9600);
  delay(500);

  /********************** compas thing ************/
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
  /**********************************************/
}

void loop()
{
    /**********************Arduino Slave***************************/
    /*Wire.requestFrom(2, 9);    // request 6 bytes from slave device #2
    String a="";
    while (Wire.available())   // slave may send less than requested
    {
      //Serial.print("c");
      char c = Wire.read(); // receive a byte as character
      if (c != 's') {
        a += c; // print the character
      }
      else {
        break;
      }
    }
    ticks2 = atoi(a.c_str());*/
    /**********************COMPASS***************************/
    sensors_event_t event;
    mag.getEvent(&event);
    headin = atan2(event.magnetic.x, event.magnetic.y);
    float declinationAngle = 0;
    headin += declinationAngle;
    if (headin < 0) {
      headin += 2 * PI;

    }
    //Check for wrap due to addition of declination.
    if (headin > 2 * PI) {
      headin -= 2 * PI;
    }
    theta = headin - initialheading;
    if(theta<0){
      theta +=2*PI;
    }
    //Serial.println(ticks2);
    Serial.println(theta*180/PI);
    /********************************************************/
}


