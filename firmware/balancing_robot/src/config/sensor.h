#ifndef SENSOR_H
#define SENSOR_H

#include "board.h"

// Shared IMU SPI speed
#define IMU_SPI_CLOCK_HZ 1000000
// Shared I2C bus speed for IMU
#define I2C_SPEED 1000000

// ===== SENSOR SOFTWARE LOW-PASS FILTER =====
// These are applied to raw MPU readings before the attitude filter uses them.
#define ACCEL_LPF_ALPHA 0.7f
#define GYRO_LPF_ALPHA 0.7f

// ===== IMU SELECTION =====
// Change ONLY this line:
#define IMU_SENSOR_PROFILE 1   // 1=MPU6050 (I2C), 2=MPU6500 (SPI), 3=MPU6500 (I2C)

#define IMU_SENSOR_MPU6050 1
#define IMU_SENSOR_MPU6500_SPI 2
#define IMU_SENSOR_MPU6500_I2C 3

#if IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6050
  #define IMU_USE_MPU6500_SPI 0
  #include "../sensors/MPU6050_Custom.h"
  using IMU_Custom = MPU6050_Custom;
#elif IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6500_SPI
  #define IMU_USE_MPU6500_SPI 1
  #include "../sensors/MPU6500_SPI_Custom.h"
  using IMU_Custom = MPU6500_SPI_Custom;
#elif IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6500_I2C
  #define IMU_USE_MPU6500_SPI 0
  #include "../sensors/MPU6500_I2C_Custom.h"
  using IMU_Custom = MPU6500_I2C_Custom;
#else
  #error "Invalid IMU_SENSOR_PROFILE. Use 1, 2, or 3."
#endif

#endif
