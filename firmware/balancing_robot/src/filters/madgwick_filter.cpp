#include <Arduino.h>
#include "madgwick_filter.h"
#include "../config/settings.h"  // ACCEL_SENSITIVITY, GYRO_* macros

// Madgwick Filter Parameters
static float madgwick_q0 = 1.0f, madgwick_q1 = 0.0f, madgwick_q2 = 0.0f, madgwick_q3 = 0.0f;  // Quaternion
static float madgwick_beta = 0.008f;  // Algorithm gain (tune for your robot) higher = trust accel more
static bool madgwick_initialized = false;

// ===== Impact Detection State =====
unsigned long lastImpactTime_madgwick = 0;
const unsigned long IMPACT_BOOST_DURATION_MD = 800;  // milliseconds to boost accel trust post-impact
const float ACCEL_IMPACT_THRESHOLD_MD = 0.8f;  // g's (0.8g above/below 1g = impact)
const float GYRO_IMPACT_THRESHOLD_MD = 300.0f;  // deg/s (300°/s = large rotation/impact)

// ===== Initialize Madgwick Quaternion from Current Accel =====
void initMadgwickFilter() {
  if (madgwick_initialized) return;

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

  float init_pitch = atan2(-accel_X_corrected, sqrt(accel_Y_corrected * accel_Y_corrected + accel_Z_corrected * accel_Z_corrected)) * 180.0f / PI;
  float init_roll = atan2(accel_Y_corrected, accel_Z_corrected) * 180.0f / PI;

  float half_pitch = (init_pitch * PI / 180.0f) * 0.5f;
  float half_roll = (init_roll * PI / 180.0f) * 0.5f;
  float half_yaw = 0.0f;

  float cp = cos(half_pitch);
  float sp = sin(half_pitch);
  float cr = cos(half_roll);
  float sr = sin(half_roll);
  float cy = cos(half_yaw);
  float sy = sin(half_yaw);

  madgwick_q0 = cy * cr * cp + sy * sr * sp;
  madgwick_q1 = cy * sr * cp - sy * cr * sp;
  madgwick_q2 = cy * cr * sp + sy * sr * cp;
  madgwick_q3 = sy * cr * cp - cy * sr * sp;

  float qNorm = sqrt(madgwick_q0 * madgwick_q0 + madgwick_q1 * madgwick_q1 +
                     madgwick_q2 * madgwick_q2 + madgwick_q3 * madgwick_q3);
  if (qNorm > 0.0f) {
    madgwick_q0 /= qNorm;
    madgwick_q1 /= qNorm;
    madgwick_q2 /= qNorm;
    madgwick_q3 /= qNorm;
  }

  madgwick_initialized = true;
}

// ===== Madgwick Filter Update =====
void updateMadgwickFilter() {
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

  if (accel_deviation > ACCEL_IMPACT_THRESHOLD_MD || gyro_magnitude > GYRO_IMPACT_THRESHOLD_MD) {
    lastImpactTime_madgwick = millis();
  }

  bool inImpactBoost = (millis() - lastImpactTime_madgwick) < IMPACT_BOOST_DURATION_MD;

  // Boost beta during impact for faster recovery
  float beta_boosted = madgwick_beta;
  if (inImpactBoost) {
    beta_boosted *= 5.0f;  // 5x accel correction during impact recovery
  }

  // Convert gyro to deg/s using global axis mapping, then apply calibrated gyro bias
  // (same convention used by PID/cascade controllers and Mahony filter)
  float gyro_roll_deg = (GYRO_ROLL_RATE_USED / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED;
  float gyro_pitch_deg = (GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED;
  float gyro_yaw_deg = (GYRO_YAW_RATE_USED / GYRO_SENSITIVITY) - GYRO_YAW_BIAS_USED;

  // Quaternion update expects x=roll-rate, y=pitch-rate, z=yaw-rate (rad/s)
  float gyro_X_rad = gyro_roll_deg / 57.2958f;
  float gyro_Y_rad = gyro_pitch_deg / 57.2958f;
  float gyro_Z_rad = gyro_yaw_deg / 57.2958f;

  float q0 = madgwick_q0, q1 = madgwick_q1, q2 = madgwick_q2, q3 = madgwick_q3;

  // Auxiliary variables
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1;
  float _8q2 = 8.0f * q2;
  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  // Gradient (for descent algorithm)
  float s0 = _4q0 * q2q2 + _2q2 * accel_X_corrected + _4q0 * q1q1 - _2q1 * accel_Y_corrected;
  float s1 = _4q1 * q3q3 - _2q3 * accel_X_corrected + 4.0f * q0q0 * q1 - _2q0 * accel_Y_corrected - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accel_Z_corrected;
  float s2 = 4.0f * q0q0 * q2 + _2q0 * accel_X_corrected + _4q2 * q3q3 - _2q3 * accel_Y_corrected - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accel_Z_corrected;
  float s3 = 4.0f * q1q1 * q3 - _2q1 * accel_X_corrected + 4.0f * q2q2 * q3 - _2q2 * accel_Y_corrected;

  // Normalize gradient
  float sNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (sNorm == 0.0f) sNorm = 1.0f;
  s0 /= sNorm;
  s1 /= sNorm;
  s2 /= sNorm;
  s3 /= sNorm;

  // Quaternion derivative from gyro
  float qDot0 = 0.5f * (-q1 * gyro_X_rad - q2 * gyro_Y_rad - q3 * gyro_Z_rad);
  float qDot1 = 0.5f * (q0 * gyro_X_rad + q2 * gyro_Z_rad - q3 * gyro_Y_rad);
  float qDot2 = 0.5f * (q0 * gyro_Y_rad - q1 * gyro_Z_rad + q3 * gyro_X_rad);
  float qDot3 = 0.5f * (q0 * gyro_Z_rad + q1 * gyro_Y_rad - q2 * gyro_X_rad);

  // Integrate (use boosted beta during impact)
  madgwick_q0 += (qDot0 - beta_boosted * s0) * dt;
  madgwick_q1 += (qDot1 - beta_boosted * s1) * dt;
  madgwick_q2 += (qDot2 - beta_boosted * s2) * dt;
  madgwick_q3 += (qDot3 - beta_boosted * s3) * dt;

  // Normalize quaternion
  float qNorm = sqrt(madgwick_q0 * madgwick_q0 + madgwick_q1 * madgwick_q1 +
                     madgwick_q2 * madgwick_q2 + madgwick_q3 * madgwick_q3);
  madgwick_q0 /= qNorm;
  madgwick_q1 /= qNorm;
  madgwick_q2 /= qNorm;
  madgwick_q3 /= qNorm;

  // Convert quaternion to Euler angles
  // Roll (phi)
  roll = atan2(2.0f * (madgwick_q0 * madgwick_q1 + madgwick_q2 * madgwick_q3),
               1.0f - 2.0f * (madgwick_q1 * madgwick_q1 + madgwick_q2 * madgwick_q2)) * 180.0f / PI;

  // Pitch (theta)
  float pitch_arg = 2.0f * (madgwick_q0 * madgwick_q2 - madgwick_q3 * madgwick_q1);
  pitch_arg = constrain(pitch_arg, -1.0f, 1.0f);
  pitch = -asin(pitch_arg) * 180.0f / PI;

  // Yaw (psi)
  yaw = -atan2(2.0f * (madgwick_q0 * madgwick_q3 + madgwick_q1 * madgwick_q2),
              1.0f - 2.0f * (madgwick_q2 * madgwick_q2 + madgwick_q3 * madgwick_q3)) * 180.0f / PI;
}
