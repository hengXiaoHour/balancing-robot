#include "state.h"

// ===== Shared live robot state definitions (were in balancing_robot.ino) =====
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float pitch_final = 0.0f, roll_final = 0.0f;
float pitch_bias = 0.0f, roll_bias = 0.0f, yaw_bias = 0.0f;
float trim_pitch = 0.0f, trim_roll = 0.0f;

float pitch_setpoint = 0.0f;
float roll_setpoint = 0.0f;
float yaw_setpoint = 0.0f;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
int16_t mpu_gyroX, mpu_gyroY, mpu_gyroZ;

float filtered_accelX = 0.0f, filtered_accelY = 0.0f, filtered_accelZ = 0.0f;
float filtered_gyroX = 0.0f, filtered_gyroY = 0.0f, filtered_gyroZ = 0.0f;

float filtered_pitch = 0.0f;
float filtered_roll = 0.0f;

bool filterInitialized = false;

float dt = 0.02f;
