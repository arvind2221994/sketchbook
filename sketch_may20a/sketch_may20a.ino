#include 

// GPS Setup
#define rxGPS 3
#define txGPS 5
SoftwareSerial serialGPS = SoftwareSerial(rxGPS, txGPS);
String stringGPS = "";

void setup() {
  //pinMode(, INPUT);
  //pinMode(txGPS, OUTPUT);


  Serial1.begin(19200);
  //Serial.println("Started");
  
  Serial.begin(19200);
  Serial.println("Started");
  

  // GPS Setup
 // serialGPS.begin(4800);
  //digitalWrite(txGPS,HIGH);

  // Cut first gibberish
  while(Serial1.available())
    if (Serial1.read() == '\r')
      break;
}

void loop()
{
  String s = checkGPS();
  if(s && s.substring(0, 6) == "$GPGGA")
  {
    Serial.println(s);
  }
}

// Check GPS and returns string if full line recorded, else false
String checkGPS()
{
  if (Serial1.available())
  {
    char c = Serial1.read();
    if (c != '\n' && c != '\r')
    {
      stringGPS  = c;
    }
    else
    {
      if (stringGPS != "")
      {
        String tmp = stringGPS;
        stringGPS = "";
        return tmp;
      }
    }
  }
  return false;
}
