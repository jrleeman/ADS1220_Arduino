#include "ADS1220.h"
#include "Arduino.h"
#include "SPI.h"

ADS1220::ADS1220() {
}

void ADS1220::writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(ADS1220_CS_PIN, LOW);
  delay(5);
  SPI.transfer(CMD_WREG|(address<<2)); // What not setting num bytes?
  SPI.transfer(value);
  delay(5);
  //startSync(); // Send start/sync for continuous conversion mode
  //delayMicroseconds(1); // Delay a minimum of td(SCCS)
  //digitalWrite(ADS1220_CS_PIN, HIGH);
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

void ADS1220::begin(uint8_t cs_pin, uint8_t drdy_pin) {
  // Set pins up
  ADS1220_CS_PIN = cs_pin;
  ADS1220_DRDY_PIN = drdy_pin;

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

byte * ADS1220::readADC_Array() {
  digitalWrite(ADS1220_CS_PIN, LOW); // Take CS low
  delayMicroseconds(1); // Minimum of td(CSSC)
  static byte dataarray[3];
  for (int x = 0; x < 3 ; x++)
  {
    dataarray[x] = SPI.transfer(SPI_MASTER_DUMMY);
  }
  delayMicroseconds(1); // Minimum of td(CSSC)
  digitalWrite(ADS1220_CS_PIN, HIGH);
  return dataarray;
}

//Single Conversion read modes
long ADS1220::readADC_Single() {
  digitalWrite(ADS1220_CS_PIN, LOW); // Take CS low
  delayMicroseconds(1); // Minimum of td(CSSC)

  SPI.transfer(0x08);
  while(digitalRead(ADS1220_DRDY_PIN) == HIGH)
  {
    //Wait to DRDY goes down
    //Not a good thing
    //Code could be stuck here
    //Need a improve later
  }
  
  long adcVal = SPI.transfer(SPI_MASTER_DUMMY);
  adcVal = (adcVal << 8) | SPI.transfer(SPI_MASTER_DUMMY);
  adcVal = (adcVal << 8) | SPI.transfer(SPI_MASTER_DUMMY);

  adcVal = ( adcVal << 8 );
  adcVal = ( adcVal >> 8 );
  delayMicroseconds(1); // Minimum of td(CSSC)
  digitalWrite(ADS1220_CS_PIN, HIGH);
  return adcVal;
}

byte * ADS1220::readADC_SingleArray() {
  digitalWrite(ADS1220_CS_PIN, LOW); // Take CS low
  delayMicroseconds(1); // Minimum of td(CSSC)

  SPI.transfer(0x08);
  while(digitalRead(ADS1220_DRDY_PIN) == HIGH)
  {
    //Wait to DRDY goes down
    //Not a good thing
    //Code could be stuck here
    //Need a improve later
  }

  static byte dataarray[3];
  for (int x = 0; x < 3 ; x++)
  {
    dataarray[x] = SPI.transfer(SPI_MASTER_DUMMY);
  }
  delayMicroseconds(1); // Minimum of td(CSSC)
  digitalWrite(ADS1220_CS_PIN, HIGH);
  return dataarray;
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

void ADS1220::writeRegisterMasked(uint8_t value, uint8_t mask, uint8_t address) {
  // Write the value to a register using the mask to leave the rest of the
  // register untouched. This does not shift the value, so it shoudl be provided
  // shifted to the appropriate positions.

  // Read what's in the register now
  uint8_t register_contents = readRegister(address);

  // Flip the mask so that it's zeros where we'll put data and zero out that
  // part of the register's contents
  register_contents = register_contents & ~mask;

  // OR in the value to be written
  register_contents = register_contents | value;

  // Write it back out
  writeRegister(address, register_contents);
}

void ADS1220::setMultiplexer(uint8_t value) {
  /* Set multiplexer

  | Value | AINp | AINn |
  | ----- | ---- | ---- |
  | 0x00  | AIN0 | AIN1 |
  | 0X01  | AIN0 | AIN2 |
  | 0X02  | AIN0 | AIN3 |
  | 0X03  | AIN1 | AIN2 |
  | 0X04  | AIN1 | AIN3 |
  | 0X05  | AIN2 | AIN3 |
  | 0X06  | AIN1 | AIN0 |
  | 0X07  | AIN3 | AIN2 |
  | 0X08  | AIN0 | AVSS |
  | 0X09  | AIN1 | AVSS |
  | 0X0A  | AIN2 | AVSS |
  | 0X0B  | AIN3 | AVSS |
  | 0X0C  |  REF/4 MON  |
  | 0X0D  | APWR/4 MON  |
  | 0X0E  |   SHORTED   |
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x0E) {
    value = 0x00;
  }
  value = value << 4; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_MUX, CONFIG_REG0_ADDRESS);
}

void ADS1220::setGain(uint8_t gain) {
  /* Sets ADC gain. Possible values are 1, 2, 4, 8, 16, 32, 64, 128. */
  uint8_t value = 0x00;
  switch(gain) {
    case 1:
      value = 0x00;
      break;
    case 2:
      value = 0x01;
      break;
    case 4:
      value = 0x02;
      break;
    case 8:
      value = 0x03;
      break;
    case 16:
      value = 0x04;
      break;
    case 32:
      value = 0x05;
      break;
    case 64:
      value = 0x06;
      break;
    case 128:
      value = 0x07;
      break;
    default:
      value = 0x00;
      break;
  }
  writeRegisterMasked(value, REG_MASK_GAIN, CONFIG_REG0_ADDRESS);
}

void ADS1220::setPGAbypass(bool value) {
  /* Bypasses the PGA if true.
     PGA can only be disabled for gains 1, 2, 4.
  */
  writeRegisterMasked(value, REG_MASK_PGA_BYPASS, CONFIG_REG0_ADDRESS);
}

void ADS1220::setDataRate(uint8_t value) {
  /* Sets the data rate for the ADC. See table 18 in datasheet for datarates
     in various operating modes. */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x07) {
    value = 0x00;
  }
  value = value << 5; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_DATARATE, CONFIG_REG1_ADDRESS);
}

void ADS1220::setOpMode(uint8_t value) {
  /* Sets the ADC operating mode:
     0 - Normal mode
     1 - Duty-cycle mode
     2 - Turbo mode
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x02) {
    value = 0x00;
  }
  value = value << 3; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_OP_MODE, CONFIG_REG1_ADDRESS);
}

void ADS1220::setConversionMode(uint8_t value) {
  /* Sets the ADC conversion mode.
     0 - Single shot mode
     1 - continuous conversion mode
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x01) {
    value = 0x00;
  }
  value = value << 2; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_CONV_MODE, CONFIG_REG1_ADDRESS);
}

void ADS1220::setTemperatureMode(uint8_t value) {
  /* Controls the state of the internal temperature sensor.
     0 - Disables temperature sensor
     1 - Enables temperature sensor
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x01) {
    value = 0x00;
  }
  value = value << 1; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_TEMP_MODE, CONFIG_REG1_ADDRESS);
}

void ADS1220::setBurnoutCurrentSources(bool value) {
  /* Turns the 10uA burn-out current sources on or off. */
  writeRegisterMasked(value, REG_MASK_BURNOUT_SOURCES, CONFIG_REG1_ADDRESS);
}

void ADS1220::setVoltageRef(uint8_t value) {
  /* Sets the voltage reference used by the ADC.
     0 - Internal 2.048 V
     1 - External on REFP0 and REFN0 inputs
     2 - External on AIN0/REFP1 and AIN3/REFN1 inputs
     3 - Use analog supply as reference
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x03) {
    value = 0x00;
  }
  value = value << 6; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_VOLTAGE_REF, CONFIG_REG2_ADDRESS);
}

void ADS1220::setFIR(uint8_t value) {
  /* Controls the FIR filter on the ADC.
     0 - No 50 or 60 Hz rejection
     1 - Both 50 and 60 Hz rejection
     2 - 50 Hz rejection
     3 - 60 Hz rejection
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x03) {
    value = 0x00;
  }
  value = value << 4; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_FIR_CONF, CONFIG_REG2_ADDRESS);
}

void ADS1220::setPowerSwitch(uint8_t value) {
  /* Configures behavior of low-side switch between AIN3/REFN1 and AVSS.
     0 - Always open
     1 - Automatically closes when START/SYNC command is sent and opens when
         POWERDOWN command is issues.
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x01) {
    value = 0x00;
  }
  value = value << 3; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_PWR_SWITCH, CONFIG_REG2_ADDRESS);
}

void ADS1220::setIDACcurrent(uint8_t value) {
  /* Set current for both IDAC1 and IDAC2 excitation sources.
     0 - Off
     1 - 10 uA
     2 - 50 uA
     3 - 100 uA
     4 - 250 uA
     5 - 500 uA
     6 - 1000 uA
     7 - 1500 uA
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x07) {
    value = 0x00;
  }
  writeRegisterMasked(value, REG_MASK_IDAC_CURRENT, CONFIG_REG2_ADDRESS);
}

void ADS1220::setIDAC1routing(uint8_t value) {
  /* Selects where IDAC1 is routed to.
     0 - Disabled
     1 - AIN0/REFP1
     2 - AIN1
     3 - AIN2
     4 - AIN3/REFN1
     5 - REFP0
     6 - REFN0
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x06) {
    value = 0x00;
  }
  writeRegisterMasked(value, REG_MASK_IDAC1_ROUTING, CONFIG_REG3_ADDRESS);
}

void ADS1220::setIDAC2routing(uint8_t value) {
  /* Selects where IDAC2 is routed to.
     0 - Disabled
     1 - AIN0/REFP1
     2 - AIN1
     3 - AIN2
     4 - AIN3/REFN1
     5 - REFP0
     6 - REFN0
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x06) {
    value = 0x00;
  }
  writeRegisterMasked(value, REG_MASK_IDAC2_ROUTING, CONFIG_REG3_ADDRESS);
}

void ADS1220::setDRDYmode(uint8_t value) {
  /* Controls the behavior of the DOUT/DRDY pin when new data are ready.
     0 - Only the dedicated DRDY pin is used
     1 - Data ready indicated on DOUT/DRDY and DRDY
 */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x01) {
    value = 0x00;
  }
  writeRegisterMasked(value, REG_MASK_DRDY_MODE, CONFIG_REG3_ADDRESS);
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
