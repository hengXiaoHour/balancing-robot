#include <Arduino.h>
#include "complementary_quaternion_filter.h"
#include "../config/settings.h"  // ACCEL_SENSITIVITY, GYRO_* macros

// ===== Complementary Quaternion Filter Parameters =====
static float comp_q_q0 = 1.0f, comp_q_q1 = 0.0f, comp_q_q2 = 0.0f, comp_q_q3 = 0.0f;  // Quaternion
static float comp_q_Kp = 0.2f;  // Proportional gain for accel correction (tune as needed) LOWER = TRUST ACCEL MORE ( FROM 0.1 TO 2.0 )
static bool comp_q_initialized = false;
// ===== Impact Detection State =====
unsigned long lastImpactTime_compq = 0;
const unsigned long IMPACT_BOOST_DURATION_CQ = 800;  // milliseconds to boost accel trust post-impact
const float ACCEL_IMPACT_THRESHOLD_CQ = 0.8f;  // g's (0.8g above/below 1g = impact)
const float GYRO_IMPACT_THRESHOLD_CQ = 300.0f;  // deg/s (300°/s = large rotation/impact)
/*
Higher Kp (2.0–3.0) → accel pulls quaternion back faster → snappier response, more noise/jitter from accel
Lower Kp (0.5–1.0) → accel correction slower → smoother, but more gyro drift over time
*/

// ===== Initialize Complementary Quaternion from Current Accel =====
void initComplementaryQuaternionFilter() {
  if (comp_q_initialized) return;

  float accel_X_corrected = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float accel_Y_corrected = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float accel_Z_corrected = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  float accelNorm = sqrt(accel_X_corrected * accel_X_corrected +
                         accel_Y_corrected * accel_Y_corrected +
                         accel_Z_corrected * accel_Z_corrected);
  if (accelNorm == 0.0f) return;

  accel_X_corrected /= accelNorm;
  accel_Y_corrected /= accelNorm;
  accel_Z_corrected /= accelNorm;

  // Initialize pitch and roll from accelerometer
  float init_pitch = atan2(-accel_X_corrected, sqrt(accel_Y_corrected * accel_Y_corrected + accel_Z_corrected * accel_Z_corrected)) * 180.0f / PI;
  float init_roll = -atan2(accel_Y_corrected, accel_Z_corrected) * 180.0f / PI;

  // Convert to quaternion
  float half_pitch = (init_pitch * PI / 180.0f) * 0.5f;
  float half_roll = (init_roll * PI / 180.0f) * 0.5f;
  float half_yaw = 0.0f;

  float cp = cos(half_pitch);
  float sp = sin(half_pitch);
  float cr = cos(half_roll);
  float sr = sin(half_roll);
  float cy = cos(half_yaw);
  float sy = sin(half_yaw);

  comp_q_q0 = cy * cr * cp + sy * sr * sp;
  comp_q_q1 = cy * sr * cp - sy * cr * sp;
  comp_q_q2 = cy * cr * sp + sy * sr * cp;
  comp_q_q3 = sy * cr * cp - cy * sr * sp;

  float qNorm = sqrt(comp_q_q0 * comp_q_q0 + comp_q_q1 * comp_q_q1 +
                     comp_q_q2 * comp_q_q2 + comp_q_q3 * comp_q_q3);
  if (qNorm > 0.0f) {
    comp_q_q0 /= qNorm;
    comp_q_q1 /= qNorm;
    comp_q_q2 /= qNorm;
    comp_q_q3 /= qNorm;
  }

  comp_q_initialized = true;
}

// ===== Reset Complementary Quaternion Filter =====
void resetComplementaryQuaternionFilter() {
  comp_q_q0 = 1.0f;
  comp_q_q1 = 0.0f;
  comp_q_q2 = 0.0f;
  comp_q_q3 = 0.0f;
  comp_q_initialized = false;
}

// ===== Complementary Quaternion Filter Update =====
void updateComplementaryQuaternionFilter() {
  // Normalize accelerometer
  float accel_X_corrected = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float accel_Y_corrected = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float accel_Z_corrected = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  float accelNorm = sqrt(accel_X_corrected * accel_X_corrected +
                         accel_Y_corrected * accel_Y_corrected +
                         accel_Z_corrected * accel_Z_corrected);

  if (accelNorm == 0.0f) return;

  accel_X_corrected /= accelNorm;
  accel_Y_corrected /= accelNorm;
  accel_Z_corrected /= accelNorm;

  // ===== IMPACT DETECTION =====
  float accel_magnitude = accelNorm;  // Already computed above
  float accel_deviation = fabs(accel_magnitude - 1.0f);
  float gyro_magnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ) / GYRO_SENSITIVITY;  // Convert to deg/s

  if (accel_deviation > ACCEL_IMPACT_THRESHOLD_CQ || gyro_magnitude > GYRO_IMPACT_THRESHOLD_CQ) {
    lastImpactTime_compq = millis();
  }

  bool inImpactBoost = (millis() - lastImpactTime_compq) < IMPACT_BOOST_DURATION_CQ;

  // Boost Kp during impact for faster recovery
  float Kp_boosted = comp_q_Kp;
  if (inImpactBoost) {
    Kp_boosted *= 5.0f;  // 5x accel correction during impact recovery
  }

  // Convert gyro to rad/s using global axis mapping and bias correction
  // (same convention used by PID/cascade controllers and other filters)
  float gyro_roll_deg = (GYRO_ROLL_RATE_USED / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED;
  float gyro_pitch_deg = (GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED;
  float gyro_yaw_deg = (GYRO_YAW_RATE_USED / GYRO_SENSITIVITY) - GYRO_YAW_BIAS_USED;

  float gx = gyro_roll_deg / 57.2958f;
  float gy = gyro_pitch_deg / 57.2958f;
  float gz = gyro_yaw_deg / 57.2958f;

  // ===== Estimated gravity from current quaternion =====
  float q0 = comp_q_q0, q1 = comp_q_q1, q2 = comp_q_q2, q3 = comp_q_q3;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // ===== Error as cross product =====
  float ex = (accel_Y_corrected * vz - accel_Z_corrected * vy);
  float ey = (accel_Z_corrected * vx - accel_X_corrected * vz);
  float ez = (accel_X_corrected * vy - accel_Y_corrected * vx);

  // ===== Apply proportional correction to gyro (use boosted Kp during impact) =====
  gx += Kp_boosted * ex;
  gy += Kp_boosted * ey;
  gz += Kp_boosted * ez;

  // ===== Quaternion derivative from corrected gyro =====
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // ===== Integrate =====
  comp_q_q0 += qDot0 * dt;
  comp_q_q1 += qDot1 * dt;
  comp_q_q2 += qDot2 * dt;
  comp_q_q3 += qDot3 * dt;

  // ===== Normalize quaternion =====
  float qNorm = sqrt(comp_q_q0 * comp_q_q0 + comp_q_q1 * comp_q_q1 +
                     comp_q_q2 * comp_q_q2 + comp_q_q3 * comp_q_q3);
  comp_q_q0 /= qNorm;
  comp_q_q1 /= qNorm;
  comp_q_q2 /= qNorm;
  comp_q_q3 /= qNorm;

  // ===== Convert quaternion to Euler angles =====
  // Roll (phi)
  roll = atan2(2.0f * (comp_q_q0 * comp_q_q1 + comp_q_q2 * comp_q_q3),
               1.0f - 2.0f * (comp_q_q1 * comp_q_q1 + comp_q_q2 * comp_q_q2)) * 180.0f / PI;

  // Pitch (theta)
  float pitch_arg = 2.0f * (comp_q_q0 * comp_q_q2 - comp_q_q3 * comp_q_q1);
  pitch_arg = constrain(pitch_arg, -1.0f, 1.0f);
  pitch = -asin(pitch_arg) * 180.0f / PI;

  // Yaw (psi)
  yaw = -atan2(2.0f * (comp_q_q0 * comp_q_q3 + comp_q_q1 * comp_q_q2),
              1.0f - 2.0f * (comp_q_q2 * comp_q_q2 + comp_q_q3 * comp_q_q3)) * 180.0f / PI;
}
