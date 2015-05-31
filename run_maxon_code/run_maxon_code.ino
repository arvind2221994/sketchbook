
#define B 843
#define R 20
float rpm[2];
double velocity1,velocity2;




Motor motors[2] = {Motor(9, 23, 25, 1), Motor(8, 22, 24, 0)}; //motors 1, 2

//motors[4] = { Motor(8, 23, 25, 0), Motor(9, 22, 24, 1)}; //motors 1, 2
/*-----------------------
 motor 1 - 8, 23, 25, 27 --- 8 - pwm, 23 - enable, 25 - dir
 motor 2 - 9, 22, 26, 27 --- 9 - pwm, 22 - enable, 24 - dir

 -------------------------*/
void bot_motion(float v, float w)
{ 
  //assuming v in input is cm/sec
  rpm[0]=(v+w*B)*60/(2*PI)/R;
  rpm[1]=(v-w*B)*60/(2*PI)/R;


  for(int i=0;i<2;i++) 
    motors[i].motor_move(rpm[i]);

}

//void bot_brake(int str)
//{
//  for(int i=0;i<4;i++)
  //    motors[i].brake(HIGH, str);   
//}

void setup()
{
  Serial.begin(9600);
analogReadResolution(12);
  motors[0].enable(HIGH);
  motors[1].enable(HIGH);
}
void loop()
{
  //   bot_brake(320);
  bot_motion(100,0); //velocity in cm/sec && angular velocity in radians/sec;
}

