#include "ADS1220.h"

ADS1220 adc;

long adc_reading;

void setup() {
  Serial.begin(115200);

  adc.begin();
  adc.setConversionMode(0x01);
  adc.setDataRate(0x00);
}

void loop() {
  if (adc.isDataReady()){
    byte * point;
    point = adc.readADC_Array();
    
    long adcVal = * point;
    adcVal = (adcVal << 8) | *(point + 1);
    adcVal = (adcVal << 8) | *(point + 2);
  
    adcVal = ( adcVal << 8 );
    adcVal = ( adcVal >> 8 );
    
    Serial.println(adcVal);
  }
}
