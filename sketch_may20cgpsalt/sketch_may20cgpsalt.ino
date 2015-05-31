#include <LiquidCrystal.h>
 
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);
 
// give the pin a name:
int led = 9;
// incoming serial byte
int inByte = 0;         
 
void setup()
{
  // set up the lcd's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.print("ENGINEERS GARAGE");
  lcd.setCursor(0, 1);
  lcd.print(" GPS  INTERFACE ");
  
  // initialize the led pin as an output.
  pinMode(led, OUTPUT);  
  // start serial port at 9600 bps
  Serial.begin(4800);
  // wait for a while till the serial port is ready
  delay(100);
 
  // send the initial data once //    
  Serial.print('\n');
  Serial.print("       EG LABS    ");
  Serial.print('\n');
  Serial.print('\r');
  Serial.print(" GEOGRAPHICAL CORDINATES");
  Serial.print('\n');
  Serial.print('\r');
  Serial.print('\n');
  
  digitalWrite(led, HIGH);       
}
 
void loop ()
{ 
//==================== searching for "GG" ===================//
    do
    {
        do
        {
            while ( !Serial.available() );    
        } while ( 'G' != Serial.read() );                    // reading a character from the GPS
      
        while(!Serial.available());
    } while ( 'G' != Serial.read() );
//==================== searching for "GG" ===================//
 
//============== seeking for north cordinate ==============//
    do
    {
        while ( !Serial.available() );                       // reading a character from the GPS    
    } while ( ',' != Serial.read() );
 
    do
    {
        while ( !Serial.available() );                       // reading a character from the GPS
    } while ( ',' != Serial.read() );
//============== seeking for north cordinate ==============//
 
//============== printing the north cordinate ===============//
    Serial.print(" N: ");
    do
    {
        while ( !Serial.available() ); 
        inByte = Serial.read();                              // reading a character from the GPS
        Serial.write ( inByte );                             // printing the Latitude
    } while ( ',' != inByte );
//============== printing the north cordinate ===============//
 
//============== seeking for east cordinate ==============//
    do
    {
        while ( !Serial.available() );                       // reading a character from the GPS
    } while ( ',' != Serial.read() );
//============== seeking for east cordinate ==============//
 
//============== printing the east cordinate ===============//
    Serial.print(" E: ");
    do
    {
        while ( !Serial.available() ); 
        inByte = Serial.read();                              // reading a character from the GPS
        Serial.write ( inByte );                             // printing the Longitude
    } while ( ',' != inByte );
//============== printing the east cordinate ===============//
    Serial.println();
    
    delay ( 100);
}
