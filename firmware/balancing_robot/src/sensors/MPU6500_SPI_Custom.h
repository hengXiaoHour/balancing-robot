#ifndef MPU6500_SPI_CUSTOM_H
#define MPU6500_SPI_CUSTOM_H

#include <Arduino.h>
#include <SPI.h>

// MPU6500 register addresses
#define MPU6500_REG_PWR_MGMT_1    0x6B
#define MPU6500_REG_ACCEL_XOUT_H  0x3B
#define MPU6500_REG_GYRO_XOUT_H   0x43
#define MPU6500_REG_WHO_AM_I      0x75
#define MPU6500_REG_ACCEL_CONFIG  0x1C
#define MPU6500_REG_GYRO_CONFIG   0x1B
#define MPU6500_REG_CONFIG        0x1A
#define MPU6500_REG_USER_CTRL     0x6A

class MPU6500_SPI_Custom {
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

  MPU6500_SPI_Custom();

  bool initialize();
  void readAccel();
  void readGyro();
  void readTemp();
  void readAll();

private:
  SPISettings spiSettings;

  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t reg, uint8_t count, uint8_t* data);
};

#endif
