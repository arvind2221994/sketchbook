//Block printing device - 2nd Demo
//Awanish Raj
//25th Jan, 2014
int motor_pinA = 7;  //Input1 of L293D
int motor_pinB = 5;  //Input2 of L293D
int trip_sw = 2;     //Trip switch connection

void setup()
{
  pinMode(11, OUTPUT);    //Pin 11 is connected to EN of L293d
  digitalWrite(11, HIGH); //Permanently enable the L293d
  pinMode(motor_pinA, OUTPUT);
  pinMode(motor_pinB, OUTPUT);
  digitalWrite(motor_pinB, LOW);  //Permanently make one terminal LOW
  digitalWrite(motor_pinA, LOW);  //Initially LOW
}

void motor_on()
{
  digitalWrite(motor_pinA, HIGH);
}

void motor_off()
{
  digitalWrite(motor_pinA, LOW);
}

void loop()
{
  
}
  
