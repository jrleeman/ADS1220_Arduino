#include "ADS1220.h"
#define FSR (((long int)1<<23)-1)
ADS1220 adc;
long adc_reading;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting ADC");
  pinMode(ADS1220_DRDY_PIN, INPUT);
  adc.begin();
  Serial.println("DONE!");
  pinMode(6, INPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
}

void loop() {
  if (adc.isDataReady()){
    adc_reading = adc.readADC();
    Serial.println(adc_reading);
  }
}
