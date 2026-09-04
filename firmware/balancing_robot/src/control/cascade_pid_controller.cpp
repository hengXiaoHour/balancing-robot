#include <Arduino.h>
#include "cascade_pid_controller.h"
#include "pid_controller.h"  // updateYawPID()

// ===== CASCADE PID STATE VARIABLES (PITCH AXIS) =====
// Rate targets defined here (were in balancing_robot.ino); externs in cascade_pid_controller.h
float pitch_rate_target = 0.0f;
float roll_rate_target = 0.0f;
// Outer loop: angle → rate setpoint
float cascade_angle_error_pitch = 0.0f;
float cascade_angle_integral_pitch = 0.0f;
float cascade_rate_setpoint_pitch = 0.0f;  // Output of outer loop

// Inner loop: rate error → motor output
float cascade_rate_error_pitch = 0.0f;
float cascade_rate_integral_pitch = 0.0f;
float cascade_pitch_output = 0.0f;  // Final motor command for pitch

// ===== CASCADE PID STATE VARIABLES (ROLL AXIS) =====
// Outer loop: angle → rate setpoint
float cascade_angle_error_roll = 0.0f;
float cascade_angle_integral_roll = 0.0f;
float cascade_rate_setpoint_roll = 0.0f;  // Output of outer loop

// Inner loop: rate error → motor output
float cascade_rate_error_roll = 0.0f;
float cascade_rate_integral_roll = 0.0f;
float cascade_roll_output = 0.0f;  // Final motor command for roll

// ===== CASCADE PID GAINS (TUNABLE) - PITCH AXIS =====
// Angle (Outer) Loop
float cascade_pitch_angle_kp = CASCADE_PITCH_ANGLE_KP;
float cascade_pitch_angle_ki = CASCADE_PITCH_ANGLE_KI;
float cascade_pitch_angle_kd = CASCADE_PITCH_ANGLE_KD;

// Rate (Inner) Loop
float cascade_pitch_rate_kp = CASCADE_PITCH_RATE_KP;
float cascade_pitch_rate_ki = CASCADE_PITCH_RATE_KI;
float cascade_pitch_rate_kd = CASCADE_PITCH_RATE_KD;

// ===== CASCADE PID GAINS (TUNABLE) - ROLL AXIS =====
// Angle (Outer) Loop
float cascade_roll_angle_kp = CASCADE_ROLL_ANGLE_KP;
float cascade_roll_angle_ki = CASCADE_ROLL_ANGLE_KI;
float cascade_roll_angle_kd = CASCADE_ROLL_ANGLE_KD;

// Rate (Inner) Loop
float cascade_roll_rate_kp = CASCADE_ROLL_RATE_KP;
float cascade_roll_rate_ki = CASCADE_ROLL_RATE_KI;
float cascade_roll_rate_kd = CASCADE_ROLL_RATE_KD;

// ===== DEBUG: Print macro expansion at compile time =====
// Uncomment to verify macros are correctly defined for your vehicle mode
//#pragma message("CASCADE_PITCH_ANGLE_KP = " STRINGIFY(CASCADE_PITCH_ANGLE_KP))

// ===== HELPER: Get gyro rate in deg/s =====
float getGyroPitchRate_DPS() {
  // GYRO_Y is inverted, so negate it to match pitch convention.
  float raw_rate = (-GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED;
  return raw_rate;
}

float getGyroRollRate_DPS() {
  // Match single-PID roll convention: invert roll gyro feedback sign.
  float raw_rate = -(GYRO_ROLL_RATE_USED / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED;
  return raw_rate;
}

// ===== DUAL LOOP CASCADE PID (PITCH) =====
void updateCascadePID_Pitch() {
  #if CASCADE_MODE_ANGLE_CONTROL
  // ===== ANGLE MODE: Outer loop (angle) + Inner loop (rate) =====
  // ===== OUTER LOOP: Angle PID =====
  // Input: angle error | Output: rate setpoint
  // Use FINAL angle (with trim applied) so trim offset works correctly
  // Negate error to match motor mixing direction (pitch error sign is inverted)
  cascade_angle_error_pitch = -(pitch_setpoint - PITCH_ANGLE_FINAL_USED);

  // Proportional
  float angle_p_pitch = cascade_pitch_angle_kp * cascade_angle_error_pitch;

  // Integral (anti-windup)
  cascade_angle_integral_pitch += cascade_angle_error_pitch * dt;
  cascade_angle_integral_pitch = constrain(cascade_angle_integral_pitch,
                                            -CASCADE_ANGLE_INTEGRAL_LIMIT,
                                            CASCADE_ANGLE_INTEGRAL_LIMIT);
  float angle_i_pitch = cascade_pitch_angle_ki * cascade_angle_integral_pitch;

  // Derivative (using angle rate from gyro)
  // Gyro is already inverted in getGyroPitchRate_DPS(), so don't negate again
  float angle_d_pitch = cascade_pitch_angle_kd * getGyroPitchRate_DPS();

  // Combine outer loop: produces rate setpoint
  cascade_rate_setpoint_pitch = angle_p_pitch + angle_i_pitch + angle_d_pitch;

  // Limit rate setpoint to prevent unrealistic demands
  cascade_rate_setpoint_pitch = constrain(cascade_rate_setpoint_pitch,
                                           -CASCADE_MAX_RATE_SETPOINT,
                                           CASCADE_MAX_RATE_SETPOINT);
  #else
  // ===== RATE MODE: Direct rate control (skip outer angle loop) =====
  cascade_rate_setpoint_pitch = pitch_rate_target;  // Use direct rate command
  cascade_angle_integral_pitch = 0.0f;  // Reset outer loop integrator
  #endif

  // ===== INNER LOOP: Rate PID (used by both angle and rate modes) =====
  // Input: rate error (gyro feedback) | Output: motor command
  float gyro_pitch_rate = getGyroPitchRate_DPS();
  // Negate gyro feedback to match pitch direction (gyro already inverted, so add instead)
  cascade_rate_error_pitch = cascade_rate_setpoint_pitch + gyro_pitch_rate;

  // Proportional
  float rate_p_pitch = cascade_pitch_rate_kp * cascade_rate_error_pitch;

  // Integral
  cascade_rate_integral_pitch += cascade_rate_error_pitch * dt;
  cascade_rate_integral_pitch = constrain(cascade_rate_integral_pitch,
                                           -CASCADE_RATE_INTEGRAL_LIMIT,
                                           CASCADE_RATE_INTEGRAL_LIMIT);
  float rate_i_pitch = cascade_pitch_rate_ki * cascade_rate_integral_pitch;

  // Derivative (rate gyro is already a derivative, use gyro acceleration if available)
  // For simplicity, set to 0 or implement acceleration filtering if needed
  float rate_d_pitch = cascade_pitch_rate_kd * 0.0f;  // May be 0 depending on gyro data

  // Combine inner loop: produces motor output
  cascade_pitch_output = rate_p_pitch + rate_i_pitch + rate_d_pitch;

  // Limit output
  cascade_pitch_output = constrain(cascade_pitch_output, -PID_MAX, PID_MAX);
}

// ===== DUAL LOOP CASCADE PID (ROLL) =====
void updateCascadePID_Roll() {
  // Balancing robot doesn't use roll axis (no sideways tilt)
  // Zero out roll outputs to prevent stale values
  cascade_angle_error_roll = 0.0f;
  cascade_angle_integral_roll = 0.0f;
  cascade_rate_setpoint_roll = 0.0f;
  cascade_rate_error_roll = 0.0f;
  cascade_rate_integral_roll = 0.0f;
  cascade_roll_output = 0.0f;
  return;

  // BALANCING ROBOT MODE: Pitch axis control (no roll needed)
  #if CASCADE_MODE_ANGLE_CONTROL
  // ===== ANGLE MODE: Outer loop (angle) + Inner loop (rate) =====
  // Match single-PID roll convention (inverted vs pitch by design).
  cascade_angle_error_roll = ROLL_ANGLE_FINAL_USED - roll_setpoint;

  // Proportional
  float angle_p_roll = cascade_roll_angle_kp * cascade_angle_error_roll;

  // Integral
  cascade_angle_integral_roll += cascade_angle_error_roll * dt;
  cascade_angle_integral_roll = constrain(cascade_angle_integral_roll,
                                           -CASCADE_ANGLE_INTEGRAL_LIMIT,
                                           CASCADE_ANGLE_INTEGRAL_LIMIT);
  float angle_i_roll = cascade_roll_angle_ki * cascade_angle_integral_roll;

  // Derivative
  float angle_d_roll = -cascade_roll_angle_kd * getGyroRollRate_DPS();

  // Rate setpoint from outer loop
  cascade_rate_setpoint_roll = angle_p_roll + angle_i_roll + angle_d_roll;
  cascade_rate_setpoint_roll = constrain(cascade_rate_setpoint_roll,
                                          -CASCADE_MAX_RATE_SETPOINT,
                                          CASCADE_MAX_RATE_SETPOINT);
  #else
  // ===== RATE MODE: Direct rate control (skip outer angle loop) =====
  cascade_rate_setpoint_roll = roll_rate_target;  // Use direct rate command
  cascade_angle_integral_roll = 0.0f;  // Reset outer loop integrator
  #endif

  // ===== INNER LOOP: Rate PID (used by both angle and rate modes) =====
  float gyro_roll_rate = getGyroRollRate_DPS();
  cascade_rate_error_roll = cascade_rate_setpoint_roll - gyro_roll_rate;

  // Proportional
  float rate_p_roll = cascade_roll_rate_kp * cascade_rate_error_roll;

  // Integral
  cascade_rate_integral_roll += cascade_rate_error_roll * dt;
  cascade_rate_integral_roll = constrain(cascade_rate_integral_roll,
                                          -CASCADE_RATE_INTEGRAL_LIMIT,
                                          CASCADE_RATE_INTEGRAL_LIMIT);
  float rate_i_roll = cascade_roll_rate_ki * cascade_rate_integral_roll;

  // Derivative
  float rate_d_roll = cascade_roll_rate_kd * 0.0f;

  // Motor output
  cascade_roll_output = rate_p_roll + rate_i_roll + rate_d_roll;
  cascade_roll_output = constrain(cascade_roll_output, -PID_MAX, PID_MAX);
}

// ===== COMBINED CASCADE UPDATE (PITCH + ROLL + YAW) =====
void updateCascadePID() {
  updateCascadePID_Pitch();
  updateCascadePID_Roll();

  // Map cascade outputs directly to motor control variables
  // Map cascade outputs to motor control (setLeftMotorSpeed and setRightMotorSpeed)
  pidOutput_Pitch = cascade_pitch_output;
  pidOutput_Roll = cascade_roll_output;

  // Yaw always uses single PID controller (not cascade)
  // Same for both single PID and cascade PID modes
  updateYawPID();
}

// ===== RESET CASCADE PID INTEGRATORS =====
void resetCascadePID() {
  cascade_angle_integral_pitch = 0.0f;
  cascade_rate_integral_pitch = 0.0f;
  cascade_angle_integral_roll = 0.0f;
  cascade_rate_integral_roll = 0.0f;
  cascade_rate_setpoint_pitch = 0.0f;
  cascade_rate_setpoint_roll = 0.0f;
  cascade_pitch_output = 0.0f;
  cascade_roll_output = 0.0f;
}

// ===== RESET CASCADE PID TO DEFAULTS =====
void resetCascadePIDToDefaults() {
  // Pitch axis
  cascade_pitch_angle_kp = CASCADE_PITCH_ANGLE_KP;
  cascade_pitch_angle_ki = CASCADE_PITCH_ANGLE_KI;
  cascade_pitch_angle_kd = CASCADE_PITCH_ANGLE_KD;
  cascade_pitch_rate_kp = CASCADE_PITCH_RATE_KP;
  cascade_pitch_rate_ki = CASCADE_PITCH_RATE_KI;
  cascade_pitch_rate_kd = CASCADE_PITCH_RATE_KD;

  // Roll axis
  cascade_roll_angle_kp = CASCADE_ROLL_ANGLE_KP;
  cascade_roll_angle_ki = CASCADE_ROLL_ANGLE_KI;
  cascade_roll_angle_kd = CASCADE_ROLL_ANGLE_KD;
  cascade_roll_rate_kp = CASCADE_ROLL_RATE_KP;
  cascade_roll_rate_ki = CASCADE_ROLL_RATE_KI;
  cascade_roll_rate_kd = CASCADE_ROLL_RATE_KD;

  Serial.println("[OK] Cascade PID reset to defaults");
  resetCascadePID();
}
