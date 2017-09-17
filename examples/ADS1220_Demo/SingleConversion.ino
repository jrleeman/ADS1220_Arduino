/*
  
  This example shows how to get single conversions out of the ADS1220.
  Tested with a ADS1120 but should work with the ADS1220 too.
  Felipe Navarro 17/09/2017
  
*/

#include "ADS1220.h"

ADS1220 adc;

void setup() {
  Serial.begin(115200);

  adc.begin(8,7);
  
  adc.setGain(1); //Set gain is a little buggie with ADS1120, doesn't have a ADS1220 to test out
  adc.setDataRate(0x00);  
  adc.setOpMode(0x00);
  adc.setConversionMode(0x01); //This enables Single Conversion modes
  adc.setMultiplexer(0x00);

}

void loop() {
    long test = adc.readADC_Single();
    Serial.println(test); 
}
