#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <Arduino.h>
#include "../config/config.h"
#include "imu.h"  // mpu, mpuInitialized
#include "../control/motor_control.h"       // stopMotors, applyVehicleInputLimits, updateVehicleMotorControl
#include "../control/pid_controller.h"      // updateDualPID, applyBraking
#if ENABLE_CASCADE_PID
#include "../control/cascade_pid_controller.h"  // updateCascadePID, resetCascadePID
#endif
#include "../comms/esp_now_handler.h"       // updateESPNOWDataOnly
#include "../comms/websocket_handler.h"     // broadcastTelemetryImmediate
#include "../utils/timing.h"                // loopTimeUs
#include "../filters/filter_selector.h"     // which filter is active
#ifdef USE_KALMAN_FILTER
#include "../filters/kalman_filter.h"
#endif
#ifdef USE_MAHONY_FILTER
#include "../filters/mahony_filter.h"
#endif
#ifdef USE_MADGWICK_FILTER
#include "../filters/madgwick_filter.h"
#endif
#ifdef USE_COMPLEMENTARY_FILTER
#include "../filters/complementary_filter.h"
#endif
#ifdef USE_COMPLEMENTARY_QUATERNION_FILTER
#include "../filters/complementary_quaternion_filter.h"
#endif
#ifdef USE_EKF_FILTER
#include "../filters/ekf_filter.h"
#endif

// Shared globals (definitions in balancing_robot.ino)
extern float dt;
extern int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
extern int16_t mpu_gyroX, mpu_gyroY, mpu_gyroZ;
extern float filtered_accelX, filtered_accelY, filtered_accelZ;
extern float filtered_gyroX, filtered_gyroY, filtered_gyroZ;
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY, gyroZ;
extern float axBias, axScale, ayBias, ayScale, azBias, azScale;
extern bool filterInitialized;
extern bool debugMonitoring;
extern bool statusMonitoring;
extern volatile unsigned long filterExecutionTime;
extern float avgFilterTime;
extern float pitch_final, roll_final;
extern float trim_pitch, trim_roll;
extern float filtered_pitch, filtered_roll;
extern bool safeToArm;
extern uint16_t safeAngleCounter;
extern bool motorsArmed, motorsActive;
extern bool testMotorActive;
extern float throttle;
extern float pidIntegral_Pitch, pidIntegral_Roll, pidIntegral_Yaw;
extern volatile unsigned long loopTimeMs;
extern volatile uint32_t freeHeapMemory, minFreeHeap;
extern SemaphoreHandle_t dataLock;  // shared-data mutex (defined in control_task.cpp)

// 1kHz control loop pinned to Core 0 (see control_task.cpp)
void controlLoopTask(void *pvParameters);

#endif
