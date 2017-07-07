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
    adc_reading = adc.readADC();
    Serial.println(adc_reading);
  }
}
