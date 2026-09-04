#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "../config/config.h"   // I2C_SDA/SCL/SPEED
#include "../config/sensor.h"   // IMU_Custom alias for the selected profile

// Global IMU object + init flag (definitions in imu.cpp)
extern IMU_Custom mpu;
extern bool mpuInitialized;

// Owned by the .ino control loop, touched by retry (definition in balancing_robot.ino)
extern bool filterInitialized;

// Bring up I2C bus + IMU, set mpuInitialized (see imu.cpp)
void initIMU();

// Re-attempt IMU init if boot failed (see imu.cpp)
void retryMPUInitialization();

// Non-blocking 30s periodic retry poll for loop() (see imu.cpp)
void pollMpuRetry();

#endif
