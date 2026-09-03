#ifndef MPU6500_SPI_CUSTOM_H
#define MPU6500_SPI_CUSTOM_H

#include <SPI.h>
#include "../config/board.h"

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

  MPU6500_SPI_Custom()
      : spiSettings(IMU_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3) {}

  bool initialize() {
    pinMode(IMU_SPI_CS_PIN, OUTPUT);
    digitalWrite(IMU_SPI_CS_PIN, HIGH);

    SPI.begin(IMU_SPI_SCK_PIN, IMU_SPI_MISO_PIN, IMU_SPI_MOSI_PIN, IMU_SPI_CS_PIN);
    delay(50);

    uint8_t whoAmI = readRegister(MPU6500_REG_WHO_AM_I);
    if (whoAmI != 0x70) {
      return false;
    }

    delay(50);

    // Wake up sensor
    writeRegister(MPU6500_REG_PWR_MGMT_1, 0x00);
    delay(50);

    // Disable I2C interface (SPI mode)
    writeRegister(MPU6500_REG_USER_CTRL, 0x10);
    delay(10);

    // Set accelerometer range to +/-2g
    writeRegister(MPU6500_REG_ACCEL_CONFIG, 0x00);

    // Set gyro range to +/-2000 deg/s
    writeRegister(MPU6500_REG_GYRO_CONFIG, 0x18);

    // Set DLPF
    writeRegister(MPU6500_REG_CONFIG, 0x00);

    delay(50);
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
  SPISettings spiSettings;

  void writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(IMU_SPI_CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);
    digitalWrite(IMU_SPI_CS_PIN, HIGH);
    SPI.endTransaction();
  }

  uint8_t readRegister(uint8_t reg) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(IMU_SPI_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(IMU_SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
  }

  void readRegisters(uint8_t reg, uint8_t count, uint8_t* data) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(IMU_SPI_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < count; i++) {
      data[i] = SPI.transfer(0x00);
    }
    digitalWrite(IMU_SPI_CS_PIN, HIGH);
    SPI.endTransaction();
  }
};

#endif
