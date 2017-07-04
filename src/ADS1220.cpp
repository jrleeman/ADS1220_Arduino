#include "ADS1220.h"
#include "Arduino.h"
#include "SPI.h"

ADS1220::ADS1220() {
  int x = 1;
}

void ADS1220::writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(ADS1220_CS_PIN, LOW);
  delay(5);
  SPI.transfer(CMD_WREG|(address<<2)); // What not setting num bytes?
  SPI.transfer(value);
  delay(5);
  digitalWrite(ADS1220_CS_PIN, HIGH);
}

uint8_t ADS1220::readRegister(uint8_t address) {
  digitalWrite(ADS1220_CS_PIN,LOW);
  delay(5);
  SPI.transfer(CMD_RREG|(address<<2)); // What not setting num bytes?
  uint8_t data = SPI.transfer(SPI_MASTER_DUMMY);
  delay(5);
  digitalWrite(ADS1220_CS_PIN,HIGH);
  return data;
}

void ADS1220::begin() {
  // Configure the SPI interface (CPOL=0, CPHA=1)
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);

  // Configure chip select as an output
  pinMode(ADS1220_CS_PIN, OUTPUT);

  // Configure DRDY as as input (mfg wants us to use interrupts)
  pinMode(ADS1220_DRDY_PIN, INPUT);

  digitalWrite(ADS1220_CS_PIN, LOW); // Set CS Low
  delayMicroseconds(1); // Wait a minimum of td(CSSC)
  reset(); // Send reset command
  delayMicroseconds(1);; // Delay a minimum of 50 us + 32 * tclk

  // Write registers (42h, 08h, 04h, 10h, 00h)
  writeRegister(CONFIG_REG0_ADDRESS, 0x05); //0x06 for gain
  writeRegister(CONFIG_REG1_ADDRESS, 0x04); //0x44 for 90 SPS 0x04 for 20
  writeRegister(CONFIG_REG2_ADDRESS, 0x10); //0x00 for above 20 sps
  writeRegister(CONFIG_REG3_ADDRESS, 0x00);

  // Sanity check read back (optional)


  startSync(); // Send start/sync for continuous conversion mode
  delayMicroseconds(1); // Delay a minimum of td(SCCS)
  digitalWrite(ADS1220_CS_PIN, HIGH); // Clear CS to high
}

bool ADS1220::isDataReady() {
  if (digitalRead(ADS1220_DRDY_PIN) == HIGH) {
    return false;
  }
  return true;
}

long ADS1220::readADC() {
  digitalWrite(ADS1220_CS_PIN, LOW); // Take CS low
  delayMicroseconds(1); // Minimum of td(CSSC)
  long adcVal = SPI.transfer(SPI_MASTER_DUMMY);
  adcVal = (adcVal << 8) | SPI.transfer(SPI_MASTER_DUMMY);
  adcVal = (adcVal << 8) | SPI.transfer(SPI_MASTER_DUMMY);

  adcVal = ( adcVal << 8 );
  adcVal = ( adcVal >> 8 );
  delayMicroseconds(1); // Minimum of td(CSSC)
  digitalWrite(ADS1220_CS_PIN, HIGH);
  return adcVal;
}

void ADS1220::sendCommand(uint8_t command) {
  // Following Protocentral's code, not sure exactly what's going on here.
  digitalWrite(ADS1220_CS_PIN, LOW);
  delay(2);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  delay(2);
  digitalWrite(ADS1220_CS_PIN, LOW);
  delay(2);
  SPI.transfer(command);
  delay(2);
  digitalWrite(ADS1220_CS_PIN, HIGH);
}

void ADS1220::reset() {
  sendCommand(CMD_RESET);
}

void ADS1220::startSync() {
  sendCommand(CMD_START_SYNC);
}

void ADS1220::powerDown() {
  sendCommand(CMD_PWRDWN);
}

void ADS1220::rdata() {
  sendCommand(CMD_RDATA);
}
