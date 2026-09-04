#include <Arduino.h>
#include "pid_controller.h"

// Definitions live here (were in balancing_robot.ino); externs in pid_controller.h
float KP = DEFAULT_KP, KI = DEFAULT_KI, KD = DEFAULT_KD;
float pidError = 0.0f, pidIntegral = 0.0f, pidOutput = 0.0f;
float pidError_Pitch = 0.0f, pidIntegral_Pitch = 0.0f, pidOutput_Pitch = 0.0f;
float KP_Pitch = DEFAULT_KP, KI_Pitch = DEFAULT_KI, KD_Pitch = DEFAULT_KD;
float pidError_Roll = 0.0f, pidIntegral_Roll = 0.0f, pidOutput_Roll = 0.0f;
float KP_Roll = DEFAULT_KP, KI_Roll = DEFAULT_KI, KD_Roll = DEFAULT_KD;
float pidIntegral_Yaw = 0.0f, pidOutput_Yaw = 0.0f;
float KP_Yaw = DEFAULT_KP, KI_Yaw = DEFAULT_KI, KD_Yaw = DEFAULT_KD;
float yaw_rate_target = 0.0f;
float LOW_THROTTLE_THRESHOLD = 15.0f;
// Velocity estimation state (used by updateVelocityEstimation)
float vel_x = 0.0f, vel_y = 0.0f;
float speed_x_setpoint = 0.0f, speed_y_setpoint = 0.0f;
float speedKp = 0.1f, speedKi = 0.01f, speedKd = 0.0f;
float speedIntegral_x = 0.0f, speedIntegral_y = 0.0f;
float speedDeadband = 0.05f, maxAngleFromSpeed = 5.0f;
float accel_x_filtered = 0.0f, accel_y_filtered = 0.0f;
float accelAlpha = 0.1f, velDecay = 0.9f;

// ===== Update Single PID Controller (PITCH ONLY) =====
void updateSinglePID() {
  // Calculate error using pitch angle (forward/backward tilt)
  pidError = pitch_setpoint - PITCH_ANGLE_RAW_USED;

  // Proportional term
  float pTerm = KP * pidError;

  // Integral term (anti-windup)
  pidIntegral += pidError * dt;
  pidIntegral = constrain(pidIntegral, -500, 500);
  float iTerm = KI * pidIntegral;

  // Derivative term using gyro rate (angular velocity in deg/s)
  // Convert gyro to deg/s (sensitivity: 16.4 for ±2000°/s)
  // Gyro Y is inverted, so negate it to match pitch convention
  float gyroRate = (-GYRO_PITCH_RATE_USED / 16.4f) - GYRO_PITCH_BIAS_USED;
  // D term should oppose the velocity (negative feedback for damping)
  float dTerm = -KD * gyroRate;  // Negative sign for proper damping

  // Compute single PID output
  pidOutput = pTerm + iTerm + dTerm;

  // Apply motor scaling to compensate for gear bias
  pidOutput_Left = pidOutput * motorScale_Left;
  pidOutput_Right = pidOutput * motorScale_Right;
}

// ===== Adaptive PID Gains =====
// For balancing robot mode - no throttle adaptation needed
float getAdaptivePitchP(float baseKp) {
  return baseKp;
}

float getAdaptivePitchI(float baseKi) {
  return baseKi;
}

float getAdaptivePitchD(float baseKd) {
  return baseKd;
}

float getAdaptiveRollP(float baseKp) {
  return baseKp;
}

float getAdaptiveRollI(float baseKi) {
  return baseKi;
}

float getAdaptiveRollD(float baseKd) {
  return baseKd;
}

// ===== Update Dual PID Controller (PITCH + ROLL) =====
void updateDualPID() {
  // ===== PITCH AXIS CONTROL (Forward/Backward Balance) =====
  // Use FINAL angles (with trim applied) for PID error calculation
  pidError_Pitch = pitch_setpoint - PITCH_ANGLE_FINAL_USED;

  // Add deadband to prevent oscillation when stopping
  if (abs(pidError_Pitch) < 0.0f) {
    pidError_Pitch = 0.0f;
  }

  // Pitch Proportional term - ADAPTIVE based on throttle
  float adaptive_KP_Pitch = getAdaptivePitchP(KP_Pitch);
  float pTerm_Pitch = adaptive_KP_Pitch * pidError_Pitch;

  // Pitch Integral term (anti-windup, only accumulate outside deadband)
  if (abs(pidError_Pitch) > 0.0f) {
    pidIntegral_Pitch += pidError_Pitch * dt;
  }
  pidIntegral_Pitch = constrain(pidIntegral_Pitch, -500, 500);
  float adaptive_KI_Pitch = getAdaptivePitchI(KI_Pitch);
  float iTerm_Pitch = adaptive_KI_Pitch * pidIntegral_Pitch;

  // Pitch Derivative term using gyro Y rate
  float gyroRate_Pitch = (-GYRO_PITCH_RATE_USED / 16.4f) - GYRO_PITCH_BIAS_USED;
  float adaptive_KD_Pitch = getAdaptivePitchD(KD_Pitch);
  float dTerm_Pitch = -adaptive_KD_Pitch * gyroRate_Pitch;

  // Pitch PID output (negated because pitch error sign is inverted compared to roll)
  pidOutput_Pitch = -(pTerm_Pitch + iTerm_Pitch + dTerm_Pitch);

  // ===== ROLL AXIS CONTROL (Left/Right Balance) =====
  pidError_Roll = ROLL_ANGLE_FINAL_USED - roll_setpoint;  // NOTE: Inverted sign compared to pitch!

  // Add deadband to prevent oscillation when stopping
  if (abs(pidError_Roll) < 0.0f) {
    pidError_Roll = 0.0f;
  }

  // Roll Proportional term - ADAPTIVE based on throttle
  float adaptive_KP_Roll = getAdaptiveRollP(KP_Roll);
  float pTerm_Roll = adaptive_KP_Roll * pidError_Roll;

  // Roll Integral term (anti-windup, only accumulate outside deadband)
  if (abs(pidError_Roll) > 0.0f) {
    pidIntegral_Roll += pidError_Roll * dt;
  }
  pidIntegral_Roll = constrain(pidIntegral_Roll, -500, 500);
  float adaptive_KI_Roll = getAdaptiveRollI(KI_Roll);
  float iTerm_Roll = adaptive_KI_Roll * pidIntegral_Roll;

  // Roll Derivative term using gyro X rate
  float gyroRate_Roll = -(GYRO_ROLL_RATE_USED / 16.4f) - GYRO_ROLL_BIAS_USED;  // X axis inverted (negate gyroX)
  float adaptive_KD_Roll = getAdaptiveRollD(KD_Roll);
  float dTerm_Roll = -adaptive_KD_Roll * gyroRate_Roll;

  // Roll PID output
  pidOutput_Roll = pTerm_Roll + iTerm_Roll + dTerm_Roll;

  // Yaw is calculated separately (used by both single and cascade PID modes)
  updateYawPID();

  // Store pitch output for compatibility
  pidOutput = pidOutput_Pitch;
}

// ===== YAW PID CONTROLLER (Separate function for use in both single and cascade modes) =====
void updateYawPID() {
  // ===== YAW AXIS CONTROL (Rate-Only, uses gyroZ feedback) =====
  // Rate-only: measure actual yaw rate from gyro
  float gyroRate_Yaw = (GYRO_YAW_RATE_USED / 16.4f) - GYRO_YAW_BIAS_USED;  // Convert to deg/s
  float yaw_rate_error = yaw_rate_target - gyroRate_Yaw;

  // Proportional term (respond to rotation rate error)
  float pTerm_Yaw = KP_Yaw * yaw_rate_error;

  // Integral term (with anti-windup & reset at low throttle)
  if (throttle > LOW_THROTTLE_THRESHOLD) {
    // Only accumulate integral when throttle is above minimum
    pidIntegral_Yaw += yaw_rate_error * dt;
    pidIntegral_Yaw = constrain(pidIntegral_Yaw, -500, 500);
  } else {
    // Reset integrator at low throttle (prevents wind-up during idle)
    pidIntegral_Yaw = 0.0f;
  }
  float iTerm_Yaw = KI_Yaw * pidIntegral_Yaw;

  // Derivative term using gyro rate (D-on-measurement for smoothness)
  // NOTE: Inverted sign - add gyro feedback for damping (slows down rotation)
  float dTerm_Yaw = -KD_Yaw * gyroRate_Yaw;

  // Yaw PID output (inverted sign for correct control direction)
  pidOutput_Yaw = -(pTerm_Yaw + iTerm_Yaw + dTerm_Yaw);
}

void updateVelocityEstimation(float accel_x, float accel_y) {
  // Low-pass filter accelerometer data
  accel_x_filtered += accelAlpha * (accel_x - accel_x_filtered);
  accel_y_filtered += accelAlpha * (accel_y - accel_y_filtered);

  // Integrate acceleration to get velocity (simple Euler integration)
  // For pitch: accel_x is forward/backward acceleration
  // For roll: accel_y is left/right acceleration
  vel_x += accel_x_filtered * dt;
  vel_y += accel_y_filtered * dt;

  // Apply velocity decay to simulate air resistance/drag - more aggressive
  vel_x *= velDecay;
  vel_y *= velDecay;

  // Reset velocity when not armed (prevents drift)
  if (!motorsArmed) {
    vel_x = 0.0f;
    vel_y = 0.0f;
    speedIntegral_x = 0.0f;
    speedIntegral_y = 0.0f;
  }
}

// ===== CASCADED CONTROL: Velocity PID (Outer Loop) =====
void updateCascadedControl() {
  // Update velocity estimation from accelerometer data
  updateVelocityEstimation(filtered_accelX, filtered_accelY);

  // ===== VELOCITY PID: X-axis (Forward/Backward) =====
  // Error = desired speed - actual speed
  float speed_error_x = speed_x_setpoint - vel_x;

  // Proportional term
  float speed_p_x = KP_Rate_X * speed_error_x;

  // Integral term (with anti-windup)
  if (fabs(speed_error_x) > speedDeadband) {
    speedIntegral_x += speed_error_x * dt;
    speedIntegral_x = constrain(speedIntegral_x, -50.0f, 50.0f);
  }
  float speed_i_x = KI_Rate_X * speedIntegral_x;

  // Derivative term (on measurement for stability)
  static float prev_vel_x = 0.0f;
  float speed_d_x = KD_Rate_X * (vel_x - prev_vel_x) / dt;
  prev_vel_x = vel_x;

  // Velocity PID output = angle correction (degrees)
  float angle_correction_x = speed_p_x + speed_i_x + speed_d_x;
  angle_correction_x = constrain(angle_correction_x, -maxAngleFromSpeed, maxAngleFromSpeed);

  // ===== VELOCITY PID: Y-axis (Left/Right) =====
  // Error = desired speed - actual speed
  float speed_error_y = speed_y_setpoint - vel_y;

  // Proportional term
  float speed_p_y = KP_Rate_Y * speed_error_y;

  // Integral term (with anti-windup)
  if (fabs(speed_error_y) > speedDeadband) {
    speedIntegral_y += speed_error_y * dt;
    speedIntegral_y = constrain(speedIntegral_y, -50.0f, 50.0f);
  }
  float speed_i_y = KI_Rate_Y * speedIntegral_y;

  // Derivative term (on measurement for stability)
  static float prev_vel_y = 0.0f;
  float speed_d_y = KD_Rate_Y * (vel_y - prev_vel_y) / dt;
  prev_vel_y = vel_y;

  // Velocity PID output = angle correction (degrees)
  float angle_correction_y = speed_p_y + speed_i_y + speed_d_y;
  angle_correction_y = constrain(angle_correction_y, -maxAngleFromSpeed, maxAngleFromSpeed);

  // ===== APPLY ANGLE CORRECTIONS =====
  // Add velocity PID corrections to RC angle setpoints
  // Note: RC input sets speed setpoints, velocity PID outputs angle corrections
  pitch_setpoint = angle_correction_x;  // Forward/backward speed → pitch angle
  roll_setpoint = -angle_correction_y;  // Left/right speed → roll angle (inverted)

  // Apply deadband to final setpoints
  if (fabs(pitch_setpoint) < 0.0f) pitch_setpoint = 0.0f;
  if (fabs(roll_setpoint) < 0.0f) roll_setpoint = 0.0f;
}


void applyBraking() {
  // Simple braking: when joystick is released, apply small opposite angle
  // This helps stop the drone faster by creating drag through angle

  const float BRAKE_STRENGTH = 3.0f;  // Degrees of braking angle
  const float BRAKE_DEADZONE = 0.0f;  // Joystick deadzone
  const float BRAKE_THRESHOLD = 0.3f;  // Minimum previous input to apply braking

  // Store previous setpoints for braking logic
  static float prev_pitch_setpoint = 0.0f;
  static float prev_roll_setpoint = 0.0f;

  // Pitch braking (forward/backward)
  if (fabs(pitch_setpoint) < BRAKE_DEADZONE && fabs(prev_pitch_setpoint) > BRAKE_THRESHOLD) {
    // Joystick just released from significant input - apply opposite braking
    float brake_angle = -prev_pitch_setpoint * BRAKE_STRENGTH;
    brake_angle = constrain(brake_angle, -5.0f, 5.0f);  // Limit brake angle
    pitch_setpoint = brake_angle;
  }

  // Roll braking (left/right)
  if (fabs(roll_setpoint) < BRAKE_DEADZONE && fabs(prev_roll_setpoint) > BRAKE_THRESHOLD) {
    // Joystick just released from significant input - apply opposite braking
    float brake_angle = -prev_roll_setpoint * BRAKE_STRENGTH;
    brake_angle = constrain(brake_angle, -5.0f, 5.0f);  // Limit brake angle
    roll_setpoint = brake_angle;
  }

  // Update previous values
  prev_pitch_setpoint = pitch_setpoint;
  prev_roll_setpoint = roll_setpoint;
}
