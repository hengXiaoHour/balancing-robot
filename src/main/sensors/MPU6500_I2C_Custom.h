#ifndef MPU6500_I2C_CUSTOM_H
#define MPU6500_I2C_CUSTOM_H

#include <Wire.h>

#define MPU6500_I2C_ADDR 0x68
#define MPU6500_I2C_ADDR_ALT 0x69

// Register addresses
#define MPU6500_REG_PWR_MGMT_1 0x6B
#define MPU6500_REG_ACCEL_XOUT_H 0x3B
#define MPU6500_REG_GYRO_XOUT_H 0x43
#define MPU6500_REG_WHO_AM_I 0x75
#define MPU6500_REG_ACCEL_CONFIG 0x1C
#define MPU6500_REG_GYRO_CONFIG 0x1B
#define MPU6500_REG_CONFIG 0x1A

class MPU6500_I2C_Custom {
public:
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  int16_t temp;

  // Calibration offsets
  int16_t accelXOffset = 0;
  int16_t accelYOffset = 0;
  int16_t accelZOffset = 0;
  int16_t gyroXOffset = 0;
  int16_t gyroYOffset = 0;
  int16_t gyroZOffset = 0;
  uint8_t deviceAddress = MPU6500_I2C_ADDR;

  MPU6500_I2C_Custom() {}

  bool initialize() {
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

  void readAccel() {
    uint8_t data[6];
    readRegisters(MPU6500_REG_ACCEL_XOUT_H, 6, data);

    accelX = ((int16_t)data[0] << 8) | data[1];
    accelY = ((int16_t)data[2] << 8) | data[3];
    accelZ = ((int16_t)data[4] << 8) | data[5];

    accelX -= accelXOffset;
    accelY -= accelYOffset;
    accelZ -= accelZOffset;
  }

  void readGyro() {
    uint8_t data[6];
    readRegisters(MPU6500_REG_GYRO_XOUT_H, 6, data);

    gyroX = ((int16_t)data[0] << 8) | data[1];
    gyroY = ((int16_t)data[2] << 8) | data[3];
    gyroZ = ((int16_t)data[4] << 8) | data[5];

    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;
  }

  void readTemp() {
    uint8_t data[2];
    readRegisters(0x41, 2, data);
    temp = ((int16_t)data[0] << 8) | data[1];
  }

  void readAll() {
    readAccel();
    readGyro();
    readTemp();
  }

private:
  bool detectAddress() {
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

  void writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
  }

  uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddress, (uint8_t)1);
    if (Wire.available()) {
      return Wire.read();
    }
    return 0xFF;
  }

  void readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
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
};

#endif
