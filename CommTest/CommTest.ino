//Interfacing with another Due
#include<Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
double theta = 0, theta_old = 0;
float initialheading = 0, headin = 0;
int ticks1 = 0, ticks2 = 0, oticks1 = 0, oticks2 = 0;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//Due QEI
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);
char c;
void setup()
{
  Serial.begin(115200);
  Serial3.begin(9600);
  delay(250);
  if (!mag.begin()) {
    while (1) {
      if (mag.begin())
        break;
    }
  }
  sensors_event_t event;
  mag.getEvent(&event);
  headin = atan2(event.magnetic.x, event.magnetic.y);
  if (headin < 0)
    headin += 2 * PI;
  if (headin > 2 * PI)
    headin -= 2 * PI;
  initialheading = headin;
  Serial.println("hello");
  Serial.println(headin * 180 / PI);
  delay(10000);
}
String a = "";
void loop()
{
  /**********************SLAVE***************************
  Serial3.flush();
  a = Serial3.readStringUntil('s');
  ticks2 = atoi(a.c_str());
  Serial.print(ticks2);
  Serial.print(",");
  /************************Slave*************************/


  /**********************COMPASS***************************/
  sensors_event_t event;
  mag.getEvent(&event);
  headin = atan2(event.magnetic.x, event.magnetic.y);
  float declinationAngle = 0;
  headin += declinationAngle;
  if (headin < 0)
    headin += 2 * PI;
  if (headin > 2 * PI)
    headin -= 2 * PI;
  theta = headin - initialheading;
  if (theta < 0)
    theta += 2 * PI;
  Serial.println(theta * 180 / PI);
  /********************************************************/
}


