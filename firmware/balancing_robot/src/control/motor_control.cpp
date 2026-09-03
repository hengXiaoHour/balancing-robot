#include <Arduino.h>
#include "motor_control.h"
#include "../config/config.h"  // ENA/IN1..IN4/ENB pins, PID_MAX

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
