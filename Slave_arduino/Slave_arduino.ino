#include<Wire.h>
const int quad_A = 2;
const int quad_B = 13;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);  
int ticks;
String enc_ticks;
void setup()
{
  Wire.begin(2);                // join i2c bus with address #2
    Serial.begin(115200);
    delay(1000);
    
    // activate peripheral functions for quad pins
    REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
    REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B    
    REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
    REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B 
    
    // activate clock for TC0
    REG_PMC_PCER0 = (1<<27);
    // select XC0 as clock source and set capture mode
    REG_TC0_CMR0 = 5; 
    // activate quadrature encoder and position measure mode, no filters
    REG_TC0_BMR = (1<<9)|(1<<8)|(0<<12);
    // enable the clock (CLKEN=1) and reset the counter (SWTRG=1) 
    // SWTRG = 1 necessary to start the clock!!
    REG_TC0_CCR0 = 5; 
  Wire.onRequest(requestEvent); // register event
  // function that executes whenever data is requested by master
// this function is registered as an event, see setup()

}

void loop()
{
 
 ticks = (int)(REG_TC0_CV0);
 enc_ticks=String(ticks);
 // delay(500);
 enc_ticks+='s';
 delay(100);
 Serial.println(enc_ticks);
}


void requestEvent()
{
  Wire.write(enc_ticks.c_str()); // respond with message of 6 byte                      // as expected by master
 Serial.println(enc_ticks);
}




