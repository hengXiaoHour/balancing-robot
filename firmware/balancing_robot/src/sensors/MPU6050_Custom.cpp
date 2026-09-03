#include <Wire.h>
#include "MPU6050_Custom.h"

MPU6050_Custom::MPU6050_Custom() {}

// Initialize MPU6050
bool MPU6050_Custom::initialize() {
  // Check if MPU6050 is present at 0x68 or 0x69
  if (!detectAddress()) {
    return false;
  }

  delay(100);

  // Wake up MPU6050 (clear sleep bit)
  writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Set accelerometer range to ±2g
  writeRegister(MPU6050_REG_ACCEL_CONFIG, 0x00);  // ±2g

  // Set gyro range to ±2000°/s
  writeRegister(MPU6050_REG_GYRO_CONFIG, 0x18);   // ±2000°/s

  // Enable DLPF (Digital Low-Pass Filter)
  // Register 0x1A: bits [2:0] = DLPF configuration
  // 0x06 = 5Hz bandwidth (best for balancing robot stability)
  // Options: 0x00(260Hz) 0x01(184Hz) 0x02(94Hz) 0x03(44Hz) 0x04(21Hz) 0x05(10Hz) 0x06(5Hz)
  writeRegister(MPU6050_REG_CONFIG, 0x00);  // DLPF = 260Hz

  delay(100);
  return true;
}

// Read accelerometer data
void MPU6050_Custom::readAccel() {
  uint8_t data[6];
  readRegisters(MPU6050_REG_ACCEL_XOUT_H, 6, data);

  accelX = ((int16_t)data[0] << 8) | data[1];
  accelY = ((int16_t)data[2] << 8) | data[3];
  accelZ = ((int16_t)data[4] << 8) | data[5];

  // Apply calibration offsets
  accelX -= accelXOffset;
  accelY -= accelYOffset;
  accelZ -= accelZOffset;
}

// Read gyroscope data
void MPU6050_Custom::readGyro() {
  uint8_t data[6];
  readRegisters(MPU6050_REG_GYRO_XOUT_H, 6, data);

  gyroX = ((int16_t)data[0] << 8) | data[1];
  gyroY = ((int16_t)data[2] << 8) | data[3];
  gyroZ = ((int16_t)data[4] << 8) | data[5];

  // Apply calibration offsets
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;
}

// Read temperature
void MPU6050_Custom::readTemp() {
  uint8_t data[2];
  readRegisters(0x41, 2, data);
  temp = ((int16_t)data[0] << 8) | data[1];
}

// Read all sensor data
void MPU6050_Custom::readAll() {
  uint8_t data[14];
  readRegisters(MPU6050_REG_ACCEL_XOUT_H, 14, data);

  accelX = ((int16_t)data[0] << 8) | data[1];
  accelY = ((int16_t)data[2] << 8) | data[3];
  accelZ = ((int16_t)data[4] << 8) | data[5];
  temp = ((int16_t)data[6] << 8) | data[7];
  gyroX = ((int16_t)data[8] << 8) | data[9];
  gyroY = ((int16_t)data[10] << 8) | data[11];
  gyroZ = ((int16_t)data[12] << 8) | data[13];

  // Apply calibration offsets
  accelX -= accelXOffset;
  accelY -= accelYOffset;
  accelZ -= accelZOffset;
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;
}

bool MPU6050_Custom::detectAddress() {
  deviceAddress = MPU6050_ADDR;
  uint8_t whoAmI = readRegister(MPU6050_REG_WHO_AM_I);
  if (whoAmI == 0x68 || whoAmI == 0x69) {
    return true;
  }

  deviceAddress = MPU6050_ADDR_ALT;
  whoAmI = readRegister(MPU6050_REG_WHO_AM_I);
  if (whoAmI == 0x68 || whoAmI == 0x69) {
    return true;
  }

  deviceAddress = MPU6050_ADDR;
  return false;
}

// Write to a register
void MPU6050_Custom::writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Read from a register
uint8_t MPU6050_Custom::readRegister(uint8_t reg) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

// Read multiple registers
void MPU6050_Custom::readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
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
