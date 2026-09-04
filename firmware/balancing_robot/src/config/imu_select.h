#ifndef IMU_SELECT_H
#define IMU_SELECT_H

// ===== IMU DRIVER STORE =====
// No user-editable lines here. The active IMU is chosen in settings.h
// via ACTIVE_IMU. This file only stores the per-sensor profiles.

#include "board_pins.h"

// Shared IMU SPI speed
#define IMU_SPI_CLOCK_HZ 1000000
// Shared I2C bus speed for IMU
#define I2C_SPEED 1000000

// IMU IDs (match ACTIVE_IMU values in settings.h)
#define IMU_SENSOR_MPU6050 1
#define IMU_SENSOR_MPU6500_SPI 2
#define IMU_SENSOR_MPU6500_I2C 3

#if ACTIVE_IMU == IMU_SENSOR_MPU6050
  #define IMU_USE_MPU6500_SPI 0
  #include "../sensors/MPU6050_Custom.h"
  using IMU_Custom = MPU6050_Custom;
#elif ACTIVE_IMU == IMU_SENSOR_MPU6500_SPI
  #define IMU_USE_MPU6500_SPI 1
  #include "../sensors/MPU6500_SPI_Custom.h"
  using IMU_Custom = MPU6500_SPI_Custom;
#elif ACTIVE_IMU == IMU_SENSOR_MPU6500_I2C
  #define IMU_USE_MPU6500_SPI 0
  #include "../sensors/MPU6500_I2C_Custom.h"
  using IMU_Custom = MPU6500_I2C_Custom;
#else
  #error "Invalid ACTIVE_IMU in settings.h. Use 1, 2, or 3."
#endif

#endif
