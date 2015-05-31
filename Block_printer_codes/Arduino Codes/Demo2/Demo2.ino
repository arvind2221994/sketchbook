//Block printing device - 2nd Demo
//Awanish Raj
//25th Jan, 2014
int motor_pinA = 7;  //Input1 of L293D
int motor_pinB = 5;  //Input2 of L293D
int trip_sw = 2;     //Trip switch connection
int trip_sw2 = 4;  //Trip Switch 2


void setup()
{
  pinMode(11, OUTPUT);    //Pin 11 is connected to EN of L293d
  digitalWrite(11, HIGH); //Permanently enable the L293d
  pinMode(motor_pinA, OUTPUT);
  pinMode(motor_pinB, OUTPUT);
  digitalWrite(motor_pinB, LOW);  //Permanently make one terminal LOW
  digitalWrite(motor_pinA, LOW);  //Initially LOW
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  pinMode(trip_sw,INPUT);
  pinMode(trip_sw2,INPUT);
  pinMode(3,OUTPUT);    //Voltage for TRIP SWITCH
  digitalWrite(3,HIGH);
  Serial.begin(9600);
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
  if((digitalRead(trip_sw)==HIGH)||(digitalRead(trip_sw2)==HIGH))
  {
    
    Serial.println("HIGH");
  
    motor_on();
    Serial.println("Motor ON");
    delay(6400);
    motor_off();
    Serial.println("Motor OFF");
    delay(2000);
    Serial.println("Motor READY\n");
  }
}
  
