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

#endif
