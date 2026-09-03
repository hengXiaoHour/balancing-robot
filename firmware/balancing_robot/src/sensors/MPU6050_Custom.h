#ifndef MPU6050_CUSTOM_H
#define MPU6050_CUSTOM_H

#include <Arduino.h>

#define MPU6050_ADDR 0x68
#define MPU6050_ADDR_ALT 0x69

// Register addresses
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_CONFIG 0x1A  // DLPF configuration

class MPU6050_Custom {
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
  uint8_t deviceAddress = MPU6050_ADDR;

  MPU6050_Custom();

  // Initialize MPU6050
  bool initialize();

  // Read accelerometer data
  void readAccel();

  // Read gyroscope data
  void readGyro();

  // Read temperature
  void readTemp();

  // Read all sensor data
  void readAll();

private:
  bool detectAddress();

  // Write to a register
  void writeRegister(uint8_t reg, uint8_t data);

  // Read from a register
  uint8_t readRegister(uint8_t reg);

  // Read multiple registers
  void readRegisters(uint8_t reg, uint8_t count, uint8_t *data);
};

#endif
