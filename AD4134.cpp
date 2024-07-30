#include "AD4134.h"

#define AD4134_DEVICE_CONFIG_REG 0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK 0x01
#define AD4134_POWER_MODE_HIGH_PERF 0x01

#define AD4134_DATA_PACKET_CONFIG_REG 0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK 0x30
#define AD4134_DATA_FRAME_24BIT_CRC 0x30

#define AD4134_DIG_IF_CFG_REG 0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK 0x03
#define AD4134_DATA_FORMAT_QUAD_CH_PARALLEL 0x02


AD4134::AD4134(int csPin)
  : _csPin(csPin) {}

void AD4134::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  SPI.begin();
  Serial.println("SPI initialized.");
}

void AD4134::configure() {
  // updateRegister(AD4134_DEVICE_CONFIG_REG, AD4134_DEVICE_CONFIG_POWER_MODE_MASK, AD4134_POWER_MODE_HIGH_PERF);
  // Serial.println("Configured power mode");

  updateRegister(AD4134_DATA_PACKET_CONFIG_REG, AD4134_DATA_PACKET_CONFIG_FRAME_MASK, AD4134_DATA_FRAME_24BIT_CRC);
  Serial.println("Configured data packet frame.");

  updateRegister(AD4134_DIG_IF_CFG_REG, AD4134_DIF_IF_CFG_FORMAT_MASK, AD4134_DATA_FORMAT_QUAD_CH_PARALLEL);
  Serial.println("Configured data interface format.");
}

uint32_t AD4134::readADC(uint8_t channel) {
  uint32_t adcValue = 0;

  digitalWrite(_csPin, LOW);
  SPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x58 | (channel & 0x03));  
  adcValue |= SPI.transfer(0x00) << 16;   // Read MSB
  adcValue |= SPI.transfer(0x00) << 8;    
  adcValue |= SPI.transfer(0x00);         // Read LSB
  SPI.endTransaction();
  digitalWrite(_csPin, HIGH);

  Serial.print("Read ADC Value: ");
  Serial.println(adcValue, HEX);

  return adcValue;
}

void AD4134::writeRegister(uint8_t reg, uint8_t value) {
  spiWrite(reg, value);
}

uint8_t AD4134::readRegister(uint8_t reg) {
  return spiRead(reg);
}

void AD4134::updateRegister(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current = readRegister(reg);
  uint8_t newValue = (current & ~mask) | (value & mask);
  writeRegister(reg, newValue);

  Serial.print("Updated register 0x");
  Serial.print(reg, HEX);
  Serial.print(" from 0x");
  Serial.print(current, HEX);
  Serial.print(" to 0x");
  Serial.println(newValue, HEX);
}

uint32_t AD4134::spiRead(uint8_t reg_addr) {
  uint32_t result = 0;
  uint16_t packet = 0;

  packet = reg_addr << 8;

  SPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  delayMicroseconds(1);
  result = SPI.transfer16(packet);
  delayMicroseconds(1);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  Serial.print("Read 0x");
  Serial.print(result, HEX);
  Serial.print(" from register 0x");
  Serial.println(reg_addr, HEX);

  return result;
}

void AD4134::spiWrite(uint8_t reg_addr, uint8_t value) {
  uint16_t packet = (reg_addr << 8) | value;

  SPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  delayMicroseconds(1);
  SPI.transfer16(packet);
  delayMicroseconds(1);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  Serial.print("Wrote 0x");
  Serial.print(value, HEX);
  Serial.print(" to register 0x");
  Serial.println(reg_addr, HEX);
}
