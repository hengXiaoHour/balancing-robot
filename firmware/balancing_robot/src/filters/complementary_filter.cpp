#include <Arduino.h>
#include "complementary_filter.h"
#include "../config/config.h"  // ACCEL_SENSITIVITY, GYRO_* macros

// Complementary Filter Parameters
static float comp_alpha = 0.999f;  // Weight of gyro (0.98 = 98% gyro, 2% accel)
                                   // Tune this: higher = trust gyro more, lower = trust accel more

// ===== Complementary Filter Update =====
void updateComplementaryFilter() {
  // Calculate pitch and roll from accelerometer
  float accel_X_corrected = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float accel_Y_corrected = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float accel_Z_corrected = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  float accel_pitch = atan2(accel_X_corrected, accel_Z_corrected) * 180.0f / PI;
  float accel_roll = atan2(accel_Y_corrected, accel_Z_corrected) * 180.0f / PI;

  // Gyro in deg/s with configured axis mapping + saved gyro bias.
  float gyro_pitch_rate = (GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED - pitch_bias;
  float gyro_roll_rate = (GYRO_ROLL_RATE_USED / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED - roll_bias;
  float gyro_yaw_rate = (GYRO_YAW_RATE_USED / GYRO_SENSITIVITY) - GYRO_YAW_BIAS_USED - yaw_bias;

  // Integrate gyro to get angle change
  pitch = comp_alpha * (pitch + gyro_pitch_rate * dt) + (1.0f - comp_alpha) * accel_pitch;
  roll = comp_alpha * (roll + gyro_roll_rate * dt) + (1.0f - comp_alpha) * accel_roll;

  // Yaw from gyro only (no accel reference for yaw)
  yaw = yaw + gyro_yaw_rate * dt;
}
