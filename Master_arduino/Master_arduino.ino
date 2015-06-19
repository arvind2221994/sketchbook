//Interfacing with another Due
#include <Wire.h>
//#include <Motor.h>
//Defining variables for class motor
#define B 84.3 //cm
#define R 20 //cm
#define TPR 21504
float rpm[2];
double velocity1, velocity2;
int ticks1, ticks2;
int nt1, nt2, oldticks1, oldticks2;
double RPM1, RPM2;
double dT, lasTime, now, r_t;

//Due QEI
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);


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
};

Motor motors[2] = {Motor(9, 23, 25, 0), Motor(8, 22, 24, 0)}; //motors 1, 2
//Motor motors[4] = {Motor(12, 29, 28, 1), Motor(11, 27, 26, 1), Motor(10, 25, 24, 0), Motor(9, 23, 22, 0)}; //motors 1, 2, 3, 4

//motors[4] = { Motor(8, 23, 25, 27, 0), Motor(9, 22, 24, 26, 1)}; //motors 1, 2
/*-----------------------
 motor 1 - 8, 23, 25, 27 --- 8 - pwm, 23 - enable, 25 - dir, 27 - brake
 motor 2 - 9, 22, 26, 27 --- 9 - pwm, 22 - enable, 24 - dir, 26 - brake
 -------------------------*/
void bot_motion(float v, float w)
{
  //assuming v in input is cm/sec
  rpm[0] = (v + w * B) * 60 / (2 * PI) / R;
  rpm[1] = (v - w * B) * 60 / (2 * PI) / R;
  for (int i = 0; i < 2; i++)
    motors[i].motor_move(rpm[i]);
}

void setup()
{
  motors[0].enable(HIGH);
  motors[1].enable(HIGH);
//  motors[0].enable(LOW);
//  motors[1].enable(LOW);
  Wire.begin();
  delay(250);
  // join i2c bus (address optional for master)
  Serial.begin(115200);
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
}
void loop()
{
  now = millis();
  dT = (now - lasTime)/1000;
  if (dT >= 0.015) {
    lasTime = now;
    ticks1 = -(int)REG_TC0_CV0;
    delay(50);
    Wire.requestFrom(2, 9);    // request 6 bytes from slave device #2
    String a;
    while (Wire.available())   // slave may send less than requested
    {
      char c = Wire.read(); // receive a byte as character
      if (c != 's') {
        a += c;
      }
      else {
        break;
      }
    }
    ticks2 = -atoi(a.c_str());
    nt1 = ticks1 - oldticks1;
    nt2 = ticks2 - oldticks2;
    
    oldticks1 = ticks1;
    oldticks2 = ticks2;
    
    RPM1 = ((nt1 / dT) / TPR )* 60;
    RPM2 = ((nt2 / dT) / TPR )* 60;
    Serial.println(RPM1);
    
    r_t = (2*PI/60)*20;
    bot_motion(100 * (r_t), 0); //RPM, radians/sec
    //bot_motion(20, 0);     //cm/sec, radians/sec    
  }
}

