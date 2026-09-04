#include <Arduino.h>
#include "kalman_filter.h"
#include "../config/config.h"  // ACCEL_SENSITIVITY, GYRO_* macros

// ===== Kalman Filter Initialization State =====
static bool kalman_initialized = false;

// 6x6 covariance matrix defined here (was in balancing_robot.ino); extern in kalman_filter.h
float P[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1}
};

// ===== Impact Detection State =====
unsigned long lastImpactTime = 0;
const unsigned long IMPACT_BOOST_DURATION = 800;  // milliseconds to boost accel trust post-impact
const float ACCEL_IMPACT_THRESHOLD = 0.8f;  // g's (0.8g above/below 1g = impact)
const float GYRO_IMPACT_THRESHOLD = 300.0f;  // deg/s (300°/s = large rotation/impact)

// ===== Initialize Kalman Filter from Current Accel =====
void initKalmanFilter() {
  if (kalman_initialized) return;

  extern float filtered_accelX, filtered_accelY, filtered_accelZ;
  extern float axBias, ayBias, azBias;

  float accel_X_corrected = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float accel_Y_corrected = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float accel_Z_corrected = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  // Initialize pitch and roll from accelerometer
  pitch = atan2(accel_X_corrected, accel_Z_corrected) * 180.0f / PI;
  roll = atan2(accel_Y_corrected, accel_Z_corrected) * 180.0f / PI;
  yaw = 0.0f;  // Yaw cannot be determined from accel alone

  // Initialize biases to zero
  pitch_bias = 0.0f;
  roll_bias = 0.0f;
  yaw_bias = 0.0f;

  // Initialize covariance matrix to identity (high uncertainty)
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }

  kalman_initialized = true;
}

// ===== 3D Kalman Filter Update =====
void updateKalmanFilter() {
  // Only update if initialized
  if (!kalman_initialized) return;

  // Get acceleration angles (bias corrected using filtered accel values)
  extern float filtered_accelX, filtered_accelY, filtered_accelZ;
  extern float axBias, ayBias, azBias;

  float accel_X_corrected = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float accel_Y_corrected = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float accel_Z_corrected = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  // Calculate pitch and roll from accelerometer
  float accel_pitch = atan2(accel_X_corrected, accel_Z_corrected) * 180.0 / PI;
  float accel_roll = atan2(accel_Y_corrected, accel_Z_corrected) * 180.0 / PI;

  // Convert gyro to deg/s with configured axis mapping and calibrated deg/s bias.
  float gyro_pitch_rate = (GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED;
  float gyro_roll_rate = (GYRO_ROLL_RATE_USED / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED;
  float gyro_yaw_rate = (GYRO_YAW_RATE_USED / GYRO_SENSITIVITY) - GYRO_YAW_BIAS_USED;

  // ===== IMPACT DETECTION =====
  // Detect high acceleration or gyro rate (crash, hit obstacle, large maneuver)
  float accel_magnitude = sqrt(accel_X_corrected*accel_X_corrected +
                               accel_Y_corrected*accel_Y_corrected +
                               accel_Z_corrected*accel_Z_corrected);
  float accel_deviation = fabs(accel_magnitude - 1.0f);  // Deviation from 1g
  float gyro_magnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ) / GYRO_SENSITIVITY;  // Convert to deg/s

  if (accel_deviation > ACCEL_IMPACT_THRESHOLD || gyro_magnitude > GYRO_IMPACT_THRESHOLD) {
    lastImpactTime = millis();  // Record impact time
  }

  // Check if we're still in post-impact boost window
  bool inImpactBoost = (millis() - lastImpactTime) < IMPACT_BOOST_DURATION;

  // Apply bias correction
  gyro_pitch_rate -= pitch_bias;
  gyro_roll_rate -= roll_bias;
  gyro_yaw_rate -= yaw_bias;

  // Predict step - integrate gyro
  pitch = pitch + dt * gyro_pitch_rate;
  roll = roll + dt * gyro_roll_rate;
  yaw = yaw + dt * gyro_yaw_rate;

  // Simplified Kalman update for 3D (using diagonal approximation for speed)
  // This is a reduced complexity version suitable for embedded systems

  // Measurement residuals
  float residual_pitch = accel_pitch - pitch;
  float residual_roll = accel_roll - roll;

  // ===== DYNAMIC KALMAN GAINS =====
  // Base gains (scale Q_BIAS from config to control bias adaptation speed)
  // KALMAN_Q_BIAS ranges from 1e-8 to 1e-2; map to K_bias range
  float K_bias = KALMAN_Q_BIAS * 0.2f;  // Scale factor to make Q_BIAS meaningful
  K_bias = constrain(K_bias, 0.001f, 0.05f);  // Clamp to reasonable range

  // Accel trust gains (higher in post-impact to correct faster)
  float K_pitch = KALMAN_Q_ANGLE * 2.0f;  // Base gain from Q_ANGLE config
  float K_roll = KALMAN_Q_ANGLE * 2.0f;

  // If in post-impact boost window, increase accel trust significantly
  if (inImpactBoost) {
    K_pitch *= 5.0f;  // Boost accel trust 5x for 800ms post-impact
    K_roll *= 5.0f;
    K_bias *= 2.0f;   // Also boost bias adaptation post-impact
  }

  // Clamp gains to reasonable ranges
  K_pitch = constrain(K_pitch, 0.0001f, 0.1f);
  K_roll = constrain(K_roll, 0.0001f, 0.1f);

  // Update state with measurement
  pitch += K_pitch * residual_pitch;
  roll += K_roll * residual_roll;

  // Update bias estimates (drift correction - scales with Q_BIAS from config)
  pitch_bias += K_bias * residual_pitch;
  roll_bias += K_bias * residual_roll;

  // Constrain bias values (loosen during impact boost to allow fast adaptation)
  float bias_limit = inImpactBoost ? 20.0f : 5.0f;
  pitch_bias = constrain(pitch_bias, -bias_limit, bias_limit);
  roll_bias = constrain(roll_bias, -bias_limit, bias_limit);
  yaw_bias = constrain(yaw_bias, -bias_limit, bias_limit);

  // Wrap angles to ±180°
  if (pitch > 180) pitch -= 360;
  if (pitch < -180) pitch += 360;
  if (roll > 180) roll -= 360;
  if (roll < -180) roll += 360;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}
