/*#define encPRA 2  //ENCODER R A PIN
#define encPLA 3  //ENCODER L A PIN
#define encPLB 4  //ENCODER R B PIN
#define encPRB 5  //ENCODER L B PIN*/

#define encPRA 2  //ENCODER R A PIN
#define encPLA 3  //ENCODER L A PIN
#define encPRB 5  //ENCODER R B PIN
#define encPLB 4  //ENCODER L B PIN


volatile long plticks = 0;                                                      //Encoder increments or decrements this variable
volatile long prticks = 0;                                                      //Encoder increments or decrements this variable


//volatile long oprticks = 0;                                                     //used in loop
//volatile long oplticks = 0;

void setup()
{
  Serial.begin (57600);
  Serial.println("Initializing....");

  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);  
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  
  pinMode(encPRA,INPUT);  //Encoder r A pin
  pinMode(encPRB,INPUT);  //Encoder r B pin
  pinMode(encPLA,INPUT);  //Encoder l A pin
  pinMode(encPLB,INPUT);  //Encoder l B pin
 
  attachInterrupt(1,calRticks,RISING);  //ENCODER R A PIN 2
  attachInterrupt(0,calLticks,RISING);  //ENCODER L A PIN 3
  /*
  pinMode(encPRB,INPUT);  //Encoder r B pin
  //pinMode(encPLA,INPUT);  //Encoder l A pin
  pinMode(encPLB,INPUT);  //Encoder l B pin
  //Attach interrupts to int0 and int1;
  
  attachInterrupt(0,calRticks,RISING);  //ENCODER R A PIN 2
  attachInterrupt(1,calLticks,RISING);  //ENCODER L A PIN 3
  */
      
}

/*************************************************************************************************************************/

void loop()
  {
    
//   analogWrite(10, 255);
//   analogWrite(11, 255);
//   
//   digitalWrite(8, HIGH);
//   digitalWrite(9, LOW);
//   Serial.println("MotorL Moving");
//     
//   digitalWrite(12, HIGH);
//   digitalWrite(13, LOW);
//   Serial.println("Motor2 Moving");

    Serial.println(prticks);
    Serial.println(plticks);
    Serial.println("   ");
    delay(1000); 
  }
  
void calRticks(){
  if(digitalRead(encPRB)==LOW){
    prticks++;
  } else{
    prticks--;
  }
}

void calLticks(){
  if(digitalRead(encPLB)==LOW){
    plticks++;
  } else{
    plticks--;
  }
}
  

