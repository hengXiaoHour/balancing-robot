#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

// ===== Mahony Filter State Variables =====
extern float pitch, roll, yaw;
extern float pitch_bias, roll_bias, yaw_bias;
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float dt;
extern int16_t gyroX, gyroY, gyroZ;
extern float filtered_accelX, filtered_accelY, filtered_accelZ;
extern float axBias, ayBias, azBias;
extern float axScale, ayScale, azScale;

// Mahony Filter Parameters
static float mahony_q0 = 1.0f, mahony_q1 = 0.0f, mahony_q2 = 0.0f, mahony_q3 = 0.0f;
static float mahony_Kp = 0.15f;
static float mahony_Ki = 0.005f;
static float mahony_integralFBx = 0.0f, mahony_integralFBy = 0.0f, mahony_integralFBz = 0.0f;
static bool mahony_initialized = false;

// ===== Impact Detection State =====
unsigned long lastImpactTime_mahony = 0;
const unsigned long IMPACT_BOOST_DURATION_M = 800;  // milliseconds to boost accel trust post-impact
const float ACCEL_IMPACT_THRESHOLD_M = 0.8f;  // g's (0.8g above/below 1g = impact)
const float GYRO_IMPACT_THRESHOLD_M = 300.0f;  // deg/s (300°/s = large rotation/impact)

// ===== Initialize Mahony Quaternion from Current Accel =====
void initMahonyFilter() {
  if (mahony_initialized) return;
  
  float accel_X_corrected = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float accel_Y_corrected = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float accel_Z_corrected = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;
  
  // Calculate pitch and roll from accelerometer (robust form)
  float init_pitch = atan2(-accel_X_corrected, sqrt(accel_Y_corrected * accel_Y_corrected + accel_Z_corrected * accel_Z_corrected)) * 180.0f / PI;
  float init_roll = atan2(accel_Y_corrected, accel_Z_corrected) * 180.0f / PI;
  
  // Convert Euler angles to quaternion
  float half_pitch = (init_pitch * PI / 180.0f) * 0.5f;
  float half_roll = (init_roll * PI / 180.0f) * 0.5f;
  float half_yaw = 0.0f;
  
  float cp = cos(half_pitch);
  float sp = sin(half_pitch);
  float cr = cos(half_roll);
  float sr = sin(half_roll);
  float cy = cos(half_yaw);
  float sy = sin(half_yaw);
  
  mahony_q0 = cy * cr * cp + sy * sr * sp;
  mahony_q1 = cy * sr * cp - sy * cr * sp;
  mahony_q2 = cy * cr * sp + sy * sr * cp;
  mahony_q3 = sy * cr * cp - cy * sr * sp;
  
  // Normalize quaternion
  float qnorm = sqrt(mahony_q0 * mahony_q0 + mahony_q1 * mahony_q1 + 
                     mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3);
  if (qnorm > 0.0f) {
    mahony_q0 /= qnorm;
    mahony_q1 /= qnorm;
    mahony_q2 /= qnorm;
    mahony_q3 /= qnorm;
  }
  
  // Reset integral terms to zero - will be found during warmup
  mahony_integralFBx = 0.0f;
  mahony_integralFBy = 0.0f;
  mahony_integralFBz = 0.0f;
  
  mahony_initialized = true;
}

// ===== Reset Mahony Filter =====
void resetMahonyFilter() {
  mahony_q0 = 1.0f;
  mahony_q1 = 0.0f;
  mahony_q2 = 0.0f;
  mahony_q3 = 0.0f;
  mahony_integralFBx = 0.0f;
  mahony_integralFBy = 0.0f;
  mahony_integralFBz = 0.0f;
  mahony_initialized = false;
}

// ===== Mahony Filter Update =====
void updateMahonyFilter() {
  // Normalize accelerometer readings
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
  
  if (accel_deviation > ACCEL_IMPACT_THRESHOLD_M || gyro_magnitude > GYRO_IMPACT_THRESHOLD_M) {
    lastImpactTime_mahony = millis();
  }
  
  bool inImpactBoost = (millis() - lastImpactTime_mahony) < IMPACT_BOOST_DURATION_M;
  
  // Boost Kp during impact for faster recovery
  float Kp_boosted = mahony_Kp;
  if (inImpactBoost) {
    Kp_boosted *= 5.0f;  // 5x accel correction during impact recovery
  }
  
  // Convert gyro to deg/s using global axis mapping, then apply calibrated gyro bias
  // (same convention used by PID/cascade controllers)
  float gyro_roll_deg = (GYRO_ROLL_RATE_USED / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED;
  float gyro_pitch_deg = (GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED;
  float gyro_yaw_deg = (GYRO_YAW_RATE_USED / GYRO_SENSITIVITY) - GYRO_YAW_BIAS_USED;

  // Quaternion update expects x=roll-rate, y=pitch-rate, z=yaw-rate (rad/s)
  float gyro_X_rad = gyro_roll_deg / 57.2958f;
  float gyro_Y_rad = gyro_pitch_deg / 57.2958f;
  float gyro_Z_rad = gyro_yaw_deg / 57.2958f;
  
  // Calculate predicted gravity vector from quaternion
  float halfvx = 2.0f * (mahony_q1 * mahony_q3 - mahony_q0 * mahony_q2);
  float halfvy = 2.0f * (mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3);
  float halfvz = mahony_q0 * mahony_q0 - mahony_q1 * mahony_q1 - mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3;
  
  // Compute error as cross product
  float halfex = (accel_Y_corrected * halfvz - accel_Z_corrected * halfvy);
  float halfey = (accel_Z_corrected * halfvx - accel_X_corrected * halfvz);
  float halfez = (accel_X_corrected * halfvy - accel_Y_corrected * halfvx);
  
  // ===== Accel Confidence Weighting =====
  float accel_mag = sqrt(accel_X_corrected * accel_X_corrected + 
                         accel_Y_corrected * accel_Y_corrected + 
                         accel_Z_corrected * accel_Z_corrected);
  
  float accel_error = fabs(accel_mag - 1.0f);
  float accel_confidence = 1.0f - (accel_error * accel_error * 0.5f);
  accel_confidence = constrain(accel_confidence, 0.3f, 1.0f);
  
  halfex *= accel_confidence;
  halfey *= accel_confidence;
  halfez *= accel_confidence;
  
  // Accumulate integral feedback (kept disabled by default via Ki=0 to prevent slow drift flips)
  mahony_integralFBx += mahony_Ki * halfex * dt;
  mahony_integralFBy += mahony_Ki * halfey * dt;
  mahony_integralFBz += mahony_Ki * halfez * dt;

  float integral_limit = 0.2f;
  if (mahony_integralFBx > integral_limit) mahony_integralFBx = integral_limit;
  if (mahony_integralFBx < -integral_limit) mahony_integralFBx = -integral_limit;
  if (mahony_integralFBy > integral_limit) mahony_integralFBy = integral_limit;
  if (mahony_integralFBy < -integral_limit) mahony_integralFBy = -integral_limit;
  if (mahony_integralFBz > integral_limit) mahony_integralFBz = integral_limit;
  if (mahony_integralFBz < -integral_limit) mahony_integralFBz = -integral_limit;
  
  // Apply proportional and integral feedback to gyro (use boosted Kp during impact)
  float gx = gyro_X_rad + Kp_boosted * halfex + mahony_integralFBx;
  float gy = gyro_Y_rad + Kp_boosted * halfey + mahony_integralFBy;
  float gz = gyro_Z_rad + Kp_boosted * halfez + mahony_integralFBz;
  
  // Quaternion rate equations
  float qa = mahony_q0;
  float qb = mahony_q1;
  float qc = mahony_q2;
  float qd = mahony_q3;
  
  mahony_q0 += (-qb * gx - qc * gy - qd * gz) * 0.5f * dt;
  mahony_q1 += (qa * gx + qc * gz - qd * gy) * 0.5f * dt;
  mahony_q2 += (qa * gy - qb * gz + qd * gx) * 0.5f * dt;
  mahony_q3 += (qa * gz + qb * gy - qc * gx) * 0.5f * dt;
  
  // Normalize quaternion
  float qnorm = sqrt(mahony_q0 * mahony_q0 + mahony_q1 * mahony_q1 + 
                     mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3);
  if (qnorm > 0.0f) {
    mahony_q0 /= qnorm;
    mahony_q1 /= qnorm;
    mahony_q2 /= qnorm;
    mahony_q3 /= qnorm;
  }
  
  // Convert quaternion to Euler angles
  roll = atan2(2.0f * (mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3),
               1.0f - 2.0f * (mahony_q1 * mahony_q1 + mahony_q2 * mahony_q2)) * 180.0f / PI;
  
  float pitch_arg = 2.0f * (mahony_q0 * mahony_q2 - mahony_q3 * mahony_q1);
  pitch_arg = constrain(pitch_arg, -1.0f, 1.0f);
  pitch = -asin(pitch_arg) * 180.0f / PI;
  
  // Yaw (ψ)
  yaw = -atan2(2.0f * (mahony_q0 * mahony_q3 + mahony_q1 * mahony_q2),
              1.0f - 2.0f * (mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3)) * 180.0f / PI;
}

#endif
