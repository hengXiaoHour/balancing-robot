#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ===== Balancing Robot Motor Control (2 Motors) =====
// Motor Layout:
//   Left Motor (ENA pin with IN1/IN2 direction control)
//   Right Motor (ENB pin with IN3/IN4 direction control)
// PWM with direction control for brushed DC motors

// ===== Stop All Motors (see motor_control.cpp) =====
void stopMotors();

// ===== Set Left Motor Speed (see motor_control.cpp) =====
// ±4095 PWM, positive=forward, negative=backward
void setLeftMotorSpeed(float speed);

// ===== Set Right Motor Speed (see motor_control.cpp) =====
// ±4095 PWM, positive=forward, negative=backward
void setRightMotorSpeed(float speed);

// ===== Vehicle (balancing-robot mode) functions (see motor_control.cpp) =====
// Shared globals (definitions in balancing_robot.ino)
extern float pitch_setpoint;
extern float roll_setpoint;
extern float dt;
extern float pidOutput_Pitch;
extern float pidOutput_Yaw;
extern float pitch_final;  // for PITCH_ANGLE_FINAL_USED in applyVehicleInputLimits

void toggleMotorTest();          // switch between normal mode and 2-motor test
void updateMotorTest();          // empty stub — called from controlLoopTask
extern bool testMotorActive;     // true while the wheel test runs (failsafe clears it)
void applyVehicleInputLimits();  // clamp/slew-limit radio stick setpoints
void initVehicleMotors();        // pin setup for 2-motor drive
void updateVehicleMotorControl();// cascade/single-PID → left+right motor mix

#endif
