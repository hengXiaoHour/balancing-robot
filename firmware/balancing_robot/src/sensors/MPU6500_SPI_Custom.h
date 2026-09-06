#ifndef MPU6500_SPI_CUSTOM_H
#define MPU6500_SPI_CUSTOM_H

#include <Arduino.h>
#include <SPI.h>
#include "imu_driver.h"

// MPU6500 register addresses
#define MPU6500_REG_PWR_MGMT_1    0x6B
#define MPU6500_REG_ACCEL_XOUT_H  0x3B
#define MPU6500_REG_GYRO_XOUT_H   0x43
#define MPU6500_REG_WHO_AM_I      0x75
#define MPU6500_REG_ACCEL_CONFIG  0x1C
#define MPU6500_REG_GYRO_CONFIG   0x1B
#define MPU6500_REG_CONFIG        0x1A
#define MPU6500_REG_USER_CTRL     0x6A

class MPU6500_SPI_Custom : public IMU_Driver {
public:
  MPU6500_SPI_Custom();

  bool initialize() override;
  const char* driverName() const override { return "MPU6500 SPI"; }
  void readAccel() override;
  void readGyro() override;
  void readTemp() override;
  void readAll() override;

private:
  SPISettings spiSettings;

  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t reg, uint8_t count, uint8_t* data);
};

#endif
