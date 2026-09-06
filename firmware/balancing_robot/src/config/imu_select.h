#ifndef IMU_SELECT_H
#define IMU_SELECT_H

// ===== IMU DRIVER STORE =====
// No user-editable lines here. ACTIVE_IMU in settings.h picks the mode:
// 0 = auto-detect (probe SPI then I2C by WHO_AM_I), 1/2/3 = force one driver.
// All drivers are always compiled; initIMU() in system/imu.cpp chooses.

#include "board_pins.h"

// Shared IMU SPI speed
#define IMU_SPI_CLOCK_HZ 1000000
// Shared I2C bus speed for IMU
#define I2C_SPEED 1000000

// IMU IDs (match ACTIVE_IMU values in settings.h)
#define IMU_SENSOR_AUTO 0
#define IMU_SENSOR_MPU6050 1
#define IMU_SENSOR_MPU6500_SPI 2
#define IMU_SENSOR_MPU6500_I2C 3

#if ACTIVE_IMU < 0 || ACTIVE_IMU > 3
  #error "Invalid ACTIVE_IMU in settings.h. Use 0 (auto), 1, 2, or 3."
#endif

#include "../sensors/imu_driver.h"
#include "../sensors/MPU6050_Custom.h"
#include "../sensors/MPU6500_SPI_Custom.h"
#include "../sensors/MPU6500_I2C_Custom.h"

#endif
