#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "../config/settings.h"  // For BOARD_ESP32C3 check

// ===== Single PID Controller State Variables =====
// Shared reference
extern float pitch, roll, yaw;  // 3D angles (raw)
extern float pitch_final, roll_final;  // Final angles WITH trim applied
extern float pitch_setpoint;    // Target pitch angle
extern float roll_setpoint;     // Target roll angle
extern float yaw_setpoint;      // Target yaw angle
extern float dt;
extern int16_t gyroY;  // Angular velocity for pitch (D term)
extern int16_t gyroX;  // Angular velocity for roll (D term)
extern int16_t gyroZ;  // Angular velocity for yaw (rate feedback)
extern float gyroBiasY;
extern float gyroBiasX;
extern float gyroBiasZ;
extern float filtered_accelX, filtered_accelY;  // Pre-filtered accel (velocity estimation)

// SINGLE PID (calculates one output for pitch)
extern float pidError;
extern float pidIntegral;
extern float pidOutput;
extern float KP, KI, KD;  // Single PID tuning

// DUAL PID - PITCH AXIS
extern float pidError_Pitch;
extern float pidIntegral_Pitch;
extern float pidOutput_Pitch;
extern float KP_Pitch, KI_Pitch, KD_Pitch;  // Pitch PID gains

// DUAL PID - ROLL AXIS
extern float pidError_Roll;
extern float pidIntegral_Roll;
extern float pidOutput_Roll;
extern float KP_Roll, KI_Roll, KD_Roll;  // Roll PID gains

// YAW AXIS (Rate-Only Control)
extern float pidIntegral_Yaw;
extern float pidOutput_Yaw;
extern float KP_Yaw, KI_Yaw, KD_Yaw;  // Yaw PID gains
extern float yaw_rate_target;  // Target yaw rotation rate
extern float throttle;  // For integrator reset logic
extern float LOW_THROTTLE_THRESHOLD;

// MOTOR INDIVIDUAL SCALING (to compensate for gear bias)
extern float motorScale_Left;   // Scale factor for left motor (1.0 = normal)
extern float motorScale_Right;  // Scale factor for right motor (1.0 = normal)

extern float pidOutput_Left;   // PID output * left scale
extern float pidOutput_Right;  // PID output * right scale

// ===== CASCADED CONTROL: Velocity Estimation & Speed Control =====
// Velocity estimates (m/s) - integrated from accelerometer
extern float vel_x, vel_y;

// Speed setpoints (m/s) - from joystick input
extern float speed_x_setpoint, speed_y_setpoint;

// Speed PID for cascaded control
extern float speedKp, speedKi, speedKd;
extern float speedIntegral_x, speedIntegral_y;
extern float speedDeadband, maxAngleFromSpeed;

// Velocity estimation parameters
extern float accel_x_filtered, accel_y_filtered;
extern float accelAlpha, velDecay;

// Control state
extern bool motorsArmed;

// ===== CASCADED CONTROL: Rate PID Gains =====
extern float KP_Rate_X, KI_Rate_X, KD_Rate_X;  // Forward/backward rate PID
extern float KP_Rate_Y, KI_Rate_Y, KD_Rate_Y;  // Left/right rate PID

// ===== PID API (see pid_controller.cpp) =====
void updateSinglePID();
float getAdaptivePitchP(float baseKp);
float getAdaptivePitchI(float baseKi);
float getAdaptivePitchD(float baseKd);
float getAdaptiveRollP(float baseKp);
float getAdaptiveRollI(float baseKi);
float getAdaptiveRollD(float baseKd);
void updateDualPID();
void updateYawPID();  // Yaw PID calculation (used by both single and cascade modes)
void updateVelocityEstimation(float accel_x, float accel_y);
void updateCascadedControl();
void applyBraking();

#endif
