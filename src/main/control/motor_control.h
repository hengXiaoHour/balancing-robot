#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ===== Balancing Robot Motor Control (2 Motors) =====
// Motor Layout:
//   Left Motor (ENA pin with IN1/IN2 direction control)
//   Right Motor (ENB pin with IN3/IN4 direction control)
// PWM with direction control for brushed DC motors

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

#endif
