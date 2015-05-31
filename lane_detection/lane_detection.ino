char incomingByte; // data from serial port
#define MOTORPINL 11
#define MOTORPINR 12
void setup() 
{
  Serial.begin (9600);
  pinMode(MOTORPINL, OUTPUT);
  pinMode(MOTORPINR, OUTPUT);
}
void loop() {
        
        // send data only when you receive data:
        if (Serial.available() > 0) 
        {
                // read the incoming byte:
                incomingByte = Serial.read();

                // say what you got:
               // Serial.print("I received");
                //Serial.println(incomingByte); //check its char or CHAR
                // find out what format in println gives what
                if(incomingByte=='m')
                {
                  forward();	
                  //Serial.println("i got m "); 
                  delay(1000);     
                }
                else if(incomingByte=='r')
                {
                 right();
		 //Serial.println(" i got r"); 
                  delay(1000);                 
                }
                else if(incomingByte=='l')
                {
                  left();
                  //Serial.println(" i got l");
                  delay(1000);                
                }
                else if(incomingByte=='s')
                {
                  wait();
                  //Serial.println("i got s ");
                  delay(1000);
                }
        }
        else
        {
          //Serial.println("No byte received");
        }
}
          void forward(void)
  {
    digitalWrite(MOTORPINL,HIGH);
    digitalWrite(MOTORPINR,LOW);
  }
  void right(void)
  {
    digitalWrite(MOTORPINL,HIGH);
    digitalWrite(MOTORPINR,LOW);
  }
  void left(void)
  {
    digitalWrite(MOTORPINL,LOW);
    digitalWrite(MOTORPINR,HIGH);
  }
  void wait(void)
  {
    digitalWrite(MOTORPINL,LOW);
    digitalWrite(MOTORPINR,HIGH);
  }
