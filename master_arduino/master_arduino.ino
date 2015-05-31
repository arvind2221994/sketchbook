#include <Wire.h>
#include<string.h>

int b;
void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
}

void loop()
{
  Wire.requestFrom(2, 9);    // request 9 bytes from slave device #2
String a;
  while(Wire.available())    // slave may send less than requested
  { 
    char c = Wire.read(); // receive a byte as character
    if(c!='s'){
    a+=c;
 //   Serial.print(c);         // print the character
  }
  else{
    
    break;}
  } 
 // Serial.println(a);
  b=atoi(a.c_str());
   Serial.println(b);
 // delay(500);
}
