#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "../config/config.h"   // I2C_SDA/SCL/SPEED
#include "../config/sensor.h"   // IMU_Custom alias for the selected profile

// Global IMU object + init flag (definitions in imu.cpp)
extern IMU_Custom mpu;
extern bool mpuInitialized;

// Bring up I2C bus + IMU, set mpuInitialized (see imu.cpp)
void initIMU();

#endif
