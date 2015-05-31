float vel[4];
double velocity;
double velocity1,velocity2;
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
    val = map(val, 0, 360, 22, 225);
    analogWrite(pwm_pin, 25);
  }

  void brake(boolean state, int strength=320)
  {
    enable(HIGH);
    int str = map(abs(strength), 0, 360, 25, 225);
    digitalWrite(br_pin, !state);
    analogWrite(pwm_pin,str);
  }

}
motors[4] = {Motor(12, 29, 28, 0), Motor(11, 27, 26, 1), Motor(10, 25, 24, 1), Motor(9, 23, 22, 1)}; //motors 1, 2, 3, 4

//motors[4] = {Motor(10, 25, 24, 1),Motor(9, 23, 22, 0),Motor(11, 27, 26, 1),Motor(12, 29, 28, 1)}; //motors 1, 2, 3, 4
/*-----------------------
 motor 1 - 12, 28, 29 --- 12 - pwm, 29 - dir, 28, brake
 motor 2 - 11, 26, 27 --- 11 - pwm, 27 - dir, 26 - brake
 motor 3 - 10, 24, 25 -- 10 - pwm, 25 - dir, 24 - brake
 motor 4 - 9, 22, 23 --- 9 - pwm, 23 - dir, 22 - brake
 -------------------------*/
 void bot_motion(float v, float w)
{ 
  vel[0]=v+w*843;
  vel[1]=v-w*843;

  for(int i=0;i<4;i++) 
    motors[i].motor_move(vel[i]);

}

void bot_brake(int str)
{
  for(int i=0;i<4;i++)
    motors[i].brake(HIGH, str);   
}

 void setup()
 {
  Serial.begin(9600);
     motors[0].enable(HIGH);
   analogReadResolution(12);
 }
 void loop()
 {
//   bot_brake(320);  
  bot_motion(100,0); //velocity in cm/sec && angular velocity in radians/sec;
  velocity1=analogRead(A5);
  velocity2=analogRead(A7);
//  Serial.print(velocity1);
//  Serial.print(",");
//  Serial.println(velocity2);
  velocity1=map(velocity1,0,4096,0,250);
  velocity2=map(velocity2,0,4096,0,250);
  Serial.print(velocity1);
  Serial.print(",");
  Serial.println(velocity2);
  Serial.println();
 }
