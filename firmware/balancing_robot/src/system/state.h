#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

// ===== Shared live robot state (see state.cpp) =====
// Cross-cutting globals with no single owning module: attitude, setpoints,
// raw sensor readings, filter I/O, filter-ready flag, control timestep.
// (Other headers may also declare these extern — duplicate externs are legal;
// the single DEFINITION lives in state.cpp.)

// Attitude (deg)
extern float pitch, roll, yaw;
extern float pitch_final, roll_final;       // WITH trim applied
extern float pitch_bias, roll_bias, yaw_bias;
extern float trim_pitch, trim_roll;         // sensor trim offsets

// Angle setpoints (deg)
extern float pitch_setpoint;
extern float roll_setpoint;
extern float yaw_setpoint;

// Raw sensor readings (LSB)
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY, gyroZ;
extern int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
extern int16_t mpu_gyroX, mpu_gyroY, mpu_gyroZ;

// Pre-filtered sensor data (deg/s, g)
extern float filtered_accelX, filtered_accelY, filtered_accelZ;
extern float filtered_gyroX, filtered_gyroY, filtered_gyroZ;

// Failsafe-filtered attitude
extern float filtered_pitch;
extern float filtered_roll;

// Filter ready flag (set by initIMU once sensors are warm)
extern bool filterInitialized;

// Control-loop timestep (s)
extern float dt;

#endif  // ROBOT_STATE_H
