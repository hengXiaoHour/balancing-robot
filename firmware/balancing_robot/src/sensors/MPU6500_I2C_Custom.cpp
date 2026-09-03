#include <Wire.h>
#include "MPU6500_I2C_Custom.h"

MPU6500_I2C_Custom::MPU6500_I2C_Custom() {}

bool MPU6500_I2C_Custom::initialize() {
  if (!detectAddress()) {
    return false;
  }

  delay(100);

  // Wake up sensor
  writeRegister(MPU6500_REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Set accelerometer range to +/-2g
  writeRegister(MPU6500_REG_ACCEL_CONFIG, 0x00);

  // Set gyro range to +/-2000 deg/s
  writeRegister(MPU6500_REG_GYRO_CONFIG, 0x18);

  // Set DLPF (260Hz)
  writeRegister(MPU6500_REG_CONFIG, 0x00);

  delay(100);
  return true;
}

void MPU6500_I2C_Custom::readAccel() {
  uint8_t data[6];
  readRegisters(MPU6500_REG_ACCEL_XOUT_H, 6, data);

  accelX = ((int16_t)data[0] << 8) | data[1];
  accelY = ((int16_t)data[2] << 8) | data[3];
  accelZ = ((int16_t)data[4] << 8) | data[5];

  accelX -= accelXOffset;
  accelY -= accelYOffset;
  accelZ -= accelZOffset;
}

void MPU6500_I2C_Custom::readGyro() {
  uint8_t data[6];
  readRegisters(MPU6500_REG_GYRO_XOUT_H, 6, data);

  gyroX = ((int16_t)data[0] << 8) | data[1];
  gyroY = ((int16_t)data[2] << 8) | data[3];
  gyroZ = ((int16_t)data[4] << 8) | data[5];

  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;
}

void MPU6500_I2C_Custom::readTemp() {
  uint8_t data[2];
  readRegisters(0x41, 2, data);
  temp = ((int16_t)data[0] << 8) | data[1];
}

void MPU6500_I2C_Custom::readAll() {
  readAccel();
  readGyro();
  readTemp();
}

bool MPU6500_I2C_Custom::detectAddress() {
  deviceAddress = MPU6500_I2C_ADDR;
  uint8_t whoAmI = readRegister(MPU6500_REG_WHO_AM_I);
  if (whoAmI == 0x70) {
    return true;
  }

  deviceAddress = MPU6500_I2C_ADDR_ALT;
  whoAmI = readRegister(MPU6500_REG_WHO_AM_I);
  if (whoAmI == 0x70) {
    return true;
  }

  deviceAddress = MPU6500_I2C_ADDR;
  return false;
}

void MPU6500_I2C_Custom::writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t MPU6500_I2C_Custom::readRegister(uint8_t reg) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void MPU6500_I2C_Custom::readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, count);

  for (int i = 0; i < count; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      data[i] = 0;
    }
  }
}
