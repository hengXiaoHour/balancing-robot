#include <Arduino.h>
#include "motor_control.h"
#include "../config/config.h"  // ENA/IN pins, PWM_*, PID_MAX, ROBOT_* limits

// Definitions live here (were in balancing_robot.ino); externs in *_handler.h / control_task.h
bool motorsArmed = false;
bool motorsActive = false;
float motorScale_Left = 1.0f;
float motorScale_Right = 1.0f;
float throttle = 25.0f;
float throttle_increment = 5.0f;
float throttle_input_normalized = 0.0f;
float pidOutput_Left = 0.0f;
float pidOutput_Right = 0.0f;
bool safeToArm = false;
uint16_t safeAngleCounter = 0;
bool testMotorActive = false;  // stubs — test mode disabled in BALANCING_ROBOT mode

// ===== Stop All Motors =====
void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ===== Set Left Motor Speed (±4095 PWM, positive=forward, negative=backward) =====
void setLeftMotorSpeed(float speed) {
  speed = constrain(speed, -PID_MAX, PID_MAX);
  int pwmValue = (int)fabs(speed);

  if (speed >= 0.0f) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  ledcWrite(ENA, pwmValue);
}

// ===== Set Right Motor Speed (±4095 PWM, positive=forward, negative=backward) =====
void setRightMotorSpeed(float speed) {
  speed = constrain(speed, -PID_MAX, PID_MAX);
  int pwmValue = (int)fabs(speed);

  if (speed >= 0.0f) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  ledcWrite(ENB, pwmValue);
}

// ===== Balancing-robot (vehicle) mode functions =====
void toggleMotorTest() {
  Serial.println("[MODE] Motor test command is disabled in BALANCING_ROBOT mode");
}

void updateMotorTest() {
}

void applyVehicleInputLimits() {
  #if ENABLE_CASCADE_PID && !CASCADE_MODE_ANGLE_CONTROL
  // In rate mode, don't modify pitch_setpoint - let pitch_rate_target handle it
  roll_setpoint = 0.0f;  // Roll always disabled for balancing robot
  return;
  #endif

  // ===== ANGLE MODE: Apply pitch setpoint limits and slew-rate limiting =====
  float measured_pitch = PITCH_ANGLE_FINAL_USED;
  float raw_setpoint = constrain(pitch_setpoint, -ROBOT_MAX_PITCH_SETPOINT_DEG, ROBOT_MAX_PITCH_SETPOINT_DEG);

  roll_setpoint = 0.0f;

  // Soft guard: only block outward commands near/over tilt boundary.
  // Do NOT inject opposite recovery commands (can cause oscillation).
  float guarded_setpoint = raw_setpoint;
  float guard_start_pos = ROBOT_MAX_ACTUAL_TILT_DEG - ROBOT_TILT_GUARD_BAND_DEG;
  float guard_start_neg = -ROBOT_MAX_ACTUAL_TILT_DEG + ROBOT_TILT_GUARD_BAND_DEG;

  if (measured_pitch >= ROBOT_MAX_ACTUAL_TILT_DEG && guarded_setpoint > 0.0f) {
    guarded_setpoint = 0.0f;
  } else if (measured_pitch <= -ROBOT_MAX_ACTUAL_TILT_DEG && guarded_setpoint < 0.0f) {
    guarded_setpoint = 0.0f;
  } else {
    // Linearly attenuate outward command inside guard band.
    if (measured_pitch > guard_start_pos && guarded_setpoint > 0.0f) {
      float t = (ROBOT_MAX_ACTUAL_TILT_DEG - measured_pitch) / max(ROBOT_TILT_GUARD_BAND_DEG, 0.001f);
      t = constrain(t, 0.0f, 1.0f);
      guarded_setpoint *= t;
    } else if (measured_pitch < guard_start_neg && guarded_setpoint < 0.0f) {
      float t = (measured_pitch + ROBOT_MAX_ACTUAL_TILT_DEG) / max(ROBOT_TILT_GUARD_BAND_DEG, 0.001f);
      t = constrain(-t, 0.0f, 1.0f);
      guarded_setpoint *= t;
    }
  }

  // Slew-rate limit setpoint changes to avoid jerky PID excitation.
  static float limited_setpoint = 0.0f;
  float max_step = ROBOT_SETPOINT_SLEW_DEG_PER_S * dt;
  float delta = guarded_setpoint - limited_setpoint;
  delta = constrain(delta, -max_step, max_step);
  limited_setpoint += delta;

  pitch_setpoint = limited_setpoint;
}

void initVehicleMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  setLeftMotorSpeed(0.0f);
  setRightMotorSpeed(0.0f);
}

void updateVehicleMotorControl() {
  // Balancing Robot Motor Mixing:
  // - pidOutput_Pitch: Balance command (both motors equal)
  // - pidOutput_Yaw: Steering command (differential speed)
  // - pidOutputRoll is NOT used in balancing mode (no sideways tilt)
  // NOTE: Yaw uses ONLY the single PID controller (unified across all modes)
  //       Whether in CASCADE_PID or single PID mode, yaw control is identical

  #if ENABLE_CASCADE_PID
  // Cascade PID: pitch from cascade loop, yaw from single PID controller
  float balance_cmd = pidOutput_Pitch;  // From cascade pitch loop
  float turn_cmd = pidOutput_Yaw;       // From single PID yaw controller (KP_Yaw, KI_Yaw, KD_Yaw from NVS)
  #else
  // Single PID: both pitch and yaw from single PID controller
  float balance_cmd = pidOutput_Pitch;  // From single PID pitch controller
  float turn_cmd = pidOutput_Yaw;       // From single PID yaw controller (KP_Yaw, KI_Yaw, KD_Yaw from NVS)
  #endif

  // Motor mixing for 2-wheel differentials:
  // LEFT = Balance + Turn
  // RIGHT = Balance - Turn
  float left_cmd = balance_cmd + turn_cmd;
  float right_cmd = balance_cmd - turn_cmd;

  left_cmd = constrain(left_cmd, -PID_MAX, PID_MAX);
  right_cmd = constrain(right_cmd, -PID_MAX, PID_MAX);

  setLeftMotorSpeed(left_cmd);
  setRightMotorSpeed(right_cmd);
}
