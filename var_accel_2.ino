#include<EEPROM.h>
#include <PS2X_lib.h>

#define PC  Serial
#define MSP_1  Serial1
#define MSP_2  Serial2
#define XB  Serial3

#define ARENA_TYPE_1  ba_spw
#define ARENA_TYPE_2  ra_spw

#define SPEED_FACTOR  1

#define ToDeg(x) (x*180/PI)
#define ToRad(x) (x*PI/180)

#define DELAY 20

#define DEBUG  1

#define SEE_SAW_PISTON  47
#define HOOK_PISTON  46
#define TURN_PISTON  43
#define SUCTION 35
#define SWING_SUCTION 42
#define HUG_PISTON 44

#define VERT_MOTOR_UP  39
#define VERT_MOTOR_DOWN  38
#define HOR_MOTOR_OUT  40
#define HOR_MOTOR_IN  41
#define VER_PWM  2
#define HOR_PWM  3

#define SONAR  A4
#define MSP_RESET SCL

#define HUG_PISTON_ON LOW
#define HOOK_PISTON_ON HIGH
#define TURN_PISTON_ON LOW
#define SEE_SAW_PISTON_ON HIGH
#define SUCTION_ON LOW
#define SWING_SUCTION_ON LOW
 
#define  HOR_TR_REAR A9
#define  HOR_TR_FRONT  A8
#define  VER_TR_TOP  A14

#define TURN_MOTOR_A 36
#define TURN_MOTOR_B 37
#define TURN_MOTOR_PWM 32

#define NUM_SONAR_LEVELS 8 
boolean sonar_flag=0;
int sonar_level=-1;
int sonar_values[NUM_SONAR_LEVELS];

boolean see_saw_state, hook_state, suction_state, turn_state, vaccum_state, swing_suction_state, hug_state,tsop_state;
unsigned long time1,time2,timer_msp;
int ext_enc_fac[4]={1,-1,1,1};
int msp_flag;
float vel[4];
PS2X ps;

double encoder[4], encoder_diff[4];
double e[4], s[4], e_temp[4];
double mx, my, mtheta, x, y, theta,hor_pos=0,hor_prev,hor_cur;
double reset_x, reset_y, reset_theta;

float yaw;

float tt,man_sp,man_rot_sp;
float speeds[4][4]={{9,30,80,100},{0.009,0.018,0.018,0.18},{110,130,180,200},{180,180,255,255}};
int gear=0, gear_cl=0;
char bot_state, bot_state_p;

/*---------class for motor-----------------*/
class Motor
{
  unsigned int br_pin, dir_pin, pwm_pin, en_pin;
  boolean aclk;

public:
  int reset;
  Motor();
  Motor(unsigned int pin1, unsigned int pin2, unsigned int pin3, boolean stat1)
  {
    reset=0;
    pwm_pin=pin1;
    dir_pin=pin2;
    br_pin=pin3;
    aclk=stat1;
    en_pin=8;    

    pinMode(br_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
    pinMode(en_pin, OUTPUT);

    digitalWrite(en_pin, LOW);
    digitalWrite(br_pin, LOW);
    digitalWrite(dir_pin,HIGH);
    analogWrite(pwm_pin,30);
  }
  
  void enable(boolean en)
  {
    digitalWrite(en_pin, en);
  }
  
  void motor_move(int vel)
  {
    brake(LOW);
    int val;
    if(vel > 0)
      digitalWrite(dir_pin, aclk);
    else if(vel < 0)
      digitalWrite(dir_pin, !aclk);
    val = constrain(abs(vel), 0, 360);
    val = map(val, 0, 360, 25, 225);
    analogWrite(pwm_pin, val);
  }

  void brake(boolean state, int strength=320)
  {
    enable(HIGH);
    int str = map(abs(strength), 0, 360, 25, 225);
    digitalWrite(br_pin, !state);
    analogWrite(pwm_pin,str);
  }

}

motors[4] = {Motor(12, 29, 28, 1), Motor(11, 27, 26, 1), Motor(10, 25, 24, 0), Motor(9, 23, 22, 0)}; //motors 1, 2, 3, 4

//motors[4] = {Motor(10, 25, 24, 1),Motor(9, 23, 22, 0),Motor(11, 27, 26, 1),Motor(12, 29, 28, 1)}; //motors 1, 2, 3, 4
/*-----------------------
 motor 1 - 12, 28, 29 --- 12 - pwm, 29 - dir, 28, brake
 motor 2 - 11, 26, 27 --- 11 - pwm, 27 - dir, 26 - brake
 motor 3 - 10, 24, 25 -- 10 - pwm, 25 - dir, 24 - brake
 motor 4 - 9, 22, 23 --- 9 - pwm, 23 - dir, 22 - brake
 -------------------------*/


/*-------------------------function defaults---------------------------------------*/
void bot_brake(int str=300);
void rotate(float ang,float w,float conv=0.02);
void motion_angle(float angle, float vel=150, float w=0);
void clamp_motion(float cx, float cy, boolean speed_override=0);
int goto_coord(float x_final,float y_final,float vel=150,float go_theta=0, float w=0, float time_acc=0, float st_vel=50, float rad_conv=50, float rad_decel=0, float cl_y=-1);

void actuation_init()
{
  pinMode(SEE_SAW_PISTON,OUTPUT);
  pinMode(HOOK_PISTON,OUTPUT);
  pinMode(TURN_PISTON,OUTPUT);
  pinMode(SUCTION,OUTPUT);
  pinMode(SWING_SUCTION,OUTPUT);
  pinMode(HUG_PISTON,OUTPUT);
  
  pinMode(VERT_MOTOR_UP,OUTPUT);
  pinMode(VERT_MOTOR_DOWN,OUTPUT);  
  pinMode(HOR_MOTOR_OUT,OUTPUT);
  pinMode(HOR_MOTOR_IN,OUTPUT);
  pinMode(VER_PWM,OUTPUT);
  pinMode(HOR_PWM,OUTPUT);     
    
  pinMode(HOR_TR_REAR,INPUT);
  pinMode(VER_TR_TOP,INPUT);      
  pinMode(HOR_TR_FRONT,INPUT);
  
  pinMode(SONAR,INPUT);
  pinMode(TURN_MOTOR_A,OUTPUT);
  pinMode(TURN_MOTOR_B,OUTPUT);  
  pinMode(TURN_MOTOR_PWM,OUTPUT);  

  
  digitalWrite(VERT_MOTOR_UP,LOW);
  digitalWrite(VERT_MOTOR_DOWN,LOW);
  digitalWrite(HOR_MOTOR_OUT,LOW);
  digitalWrite(HOR_MOTOR_IN,LOW);
  digitalWrite(VER_PWM,HIGH);
  digitalWrite(HOR_PWM,HIGH);
  digitalWrite(TURN_MOTOR_A,LOW);
  digitalWrite(TURN_MOTOR_B,LOW);
  digitalWrite(TURN_MOTOR_PWM,HIGH);
  
  see_saw_state=!SEE_SAW_PISTON_ON;
  hook_state=HOOK_PISTON_ON;
  turn_state-!TURN_PISTON_ON;
  suction_state=SUCTION_ON;
  swing_suction_state=SWING_SUCTION_ON;
  hug_state=HUG_PISTON_ON;
  
  digitalWrite(SEE_SAW_PISTON,see_saw_state);
  digitalWrite(HOOK_PISTON,hook_state);
  digitalWrite(TURN_PISTON,turn_state);
  digitalWrite(SUCTION,suction_state);
  digitalWrite(SWING_SUCTION,swing_suction_state);
  digitalWrite(HUG_PISTON,hug_state);    
}
  
void sonar_pos_init() 
{
  int add = 0;
  for(int i=0; i<8; ++i) {
    sonar_values[i] = EEPROM.read(add++);
    sonar_values[i] += EEPROM.read(add++)<<8;
  }
}

void setup()
{ 
  sonar_pos_init();
  init_ps2();
  
  PC.begin(115200);
  MSP_1.begin(115200);
  MSP_2.begin(115200);
  XB.begin(57600);
  
  while(PC.available())
    PC.read();
  while(MSP_1.available())
    MSP_1.read();
  while(MSP_2.available())
    MSP_2.read();
  while(XB.available())
    XB.read();
    
  pinMode(MSP_RESET,OUTPUT);
  digitalWrite(MSP_RESET,HIGH);
  delay(10);
  digitalWrite(MSP_RESET,LOW);
 
  PC.println("Setup");
  while(!ps.ButtonPressed(PSB_START))
  {
    ps.read_gamepad();  
  }
  actuation_init();  
   
  time1=millis();
  time2=millis();
  
  bot_state=0x00;
  
}

void loop()
{
  time1=millis();
  //PC.println(trip(),HEX);
  //PC.println(mvs());
  ps_read();   
  //PC.println(millis()-time1);
  while(millis()-time1<20); 
}
