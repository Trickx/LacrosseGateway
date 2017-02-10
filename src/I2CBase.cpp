#include "I2CBase.h"

void I2CBase::Write8(byte reg, byte value) {
  Wire.beginTransmission(m_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte I2CBase::Read8(byte reg) {
  uint8_t value;

  Wire.beginTransmission(m_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(m_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

uint16_t I2CBase::Read16(byte reg) {
  uint16_t value;

  Wire.beginTransmission(m_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(m_address, (byte)2);
  value = (Wire.read() << 8) | Wire.read();
  Wire.endTransmission();

  return value;
}

uint16_t I2CBase::Read16_LE(byte reg) {
  uint16_t temp = Read16(reg);
  return (temp >> 8) | (temp << 8);
}

int16_t I2CBase::ReadS16_LE(byte reg){
  return (int16_t)Read16_LE(reg);
}

