#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "../config/settings.h"   // I2C_SDA/SCL/SPEED + IMU_Custom alias

// Global IMU driver pointer + init flag (definitions in imu.cpp)
// mpu points at the auto-detected (or ACTIVE_IMU-forced) driver object.
extern IMU_Driver* mpu;
extern bool mpuInitialized;

// Human-readable active driver name, or "none" (see imu.cpp)
const char* activeImuName();

// CLI 'imu show' listing (see imu.cpp)
void printImuStatus();

// Owned by the .ino control loop, touched by retry (definition in balancing_robot.ino)
extern bool filterInitialized;

// Bring up I2C bus + IMU, set mpuInitialized (see imu.cpp)
void initIMU();

// Re-attempt IMU init if boot failed (see imu.cpp)
void retryMPUInitialization();

// Non-blocking 30s periodic retry poll for loop() (see imu.cpp)
void pollMpuRetry();

#endif
