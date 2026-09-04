#include <Arduino.h>
#include <string.h>  // memset
#include "ekf_filter.h"
#include "../config/settings.h"  // ACCEL_SENSITIVITY, GYRO_* macros

// ===== EKF Tuning Parameters =====
// Process noise (Q matrix diagonal)
/*static float ekf_q_gyro = 0.001f;   // Quaternion process noise (typ 0.0001–0.01)
                                      // Higher = trust gyro less, more responsive to accel
                                      // Lower  = trust gyro more, smoother but slower correction
static float ekf_q_bias = 0.01f;  // Gyro bias random walk (typ 1e-6–1e-3)
                                      // Higher = bias adapts faster (good for temp drift)
                                      // Lower  = bias assumed very stable
// Measurement noise (R matrix diagonal)
static float ekf_r_accel = 1.5f;    // Accelerometer measurement noise (typ 0.01–2.0)
                                      // Higher = trust accel less, smoother
                                      // Lower  = trust accel more, noisier but faster correction
*/

static float ekf_q_gyro = 0.0005f;
static float ekf_q_bias = 0.00005f;
static float ekf_r_accel = 2.0f;

// ===== EKF State =====
static float ekf_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Quaternion [w, x, y, z]
static float ekf_bias[3] = {0.0f, 0.0f, 0.0f};      // Gyro bias [bx, by, bz] in rad/s
static float ekf_P[7][7];  // 7x7 covariance matrix
static bool ekf_initialized = false;

// ===== Impact Detection State =====
unsigned long lastImpactTime_ekf = 0;
const unsigned long IMPACT_BOOST_DURATION_EKF = 800;
const float ACCEL_IMPACT_THRESHOLD_EKF = 0.8f;
const float GYRO_IMPACT_THRESHOLD_EKF = 300.0f;

// ===== Helper: Normalize quaternion =====
static void ekf_normalize_quat() {
  float norm = sqrt(ekf_q[0]*ekf_q[0] + ekf_q[1]*ekf_q[1] +
                    ekf_q[2]*ekf_q[2] + ekf_q[3]*ekf_q[3]);
  if (norm > 0.0f) {
    ekf_q[0] /= norm;
    ekf_q[1] /= norm;
    ekf_q[2] /= norm;
    ekf_q[3] /= norm;
  }
}

// ===== Initialize EKF from Current Accel =====
void initEKFFilter() {
  if (ekf_initialized) return;

  float ax = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float ay = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float az = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  float accelNorm = sqrt(ax*ax + ay*ay + az*az);
  if (accelNorm == 0.0f) return;
  ax /= accelNorm;
  ay /= accelNorm;
  az /= accelNorm;

  // Compute initial pitch and roll from accel
  float init_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;
  float init_roll  = -atan2(ay, az) * 180.0f / PI;

  // Convert to quaternion
  float hp = (init_pitch * PI / 180.0f) * 0.5f;
  float hr = (init_roll  * PI / 180.0f) * 0.5f;
  float hy = 0.0f;

  float cp = cos(hp), sp = sin(hp);
  float cr = cos(hr), sr = sin(hr);
  float cy = cos(hy), sy = sin(hy);

  ekf_q[0] = cy*cr*cp + sy*sr*sp;
  ekf_q[1] = cy*sr*cp - sy*cr*sp;
  ekf_q[2] = cy*cr*sp + sy*sr*cp;
  ekf_q[3] = sy*cr*cp - cy*sr*sp;
  ekf_normalize_quat();

  // Zero bias
  ekf_bias[0] = 0.0f;
  ekf_bias[1] = 0.0f;
  ekf_bias[2] = 0.0f;

  // Initialize P to identity (moderate uncertainty)
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++)
      ekf_P[i][j] = (i == j) ? 0.1f : 0.0f;

  ekf_initialized = true;
}

// ===== Reset EKF Filter =====
void resetEKFFilter() {
  ekf_q[0] = 1.0f; ekf_q[1] = 0.0f; ekf_q[2] = 0.0f; ekf_q[3] = 0.0f;
  ekf_bias[0] = 0.0f; ekf_bias[1] = 0.0f; ekf_bias[2] = 0.0f;
  ekf_initialized = false;
}

// ===== EKF Update =====
void updateEKFFilter() {
  if (!ekf_initialized) return;

  // --- Accelerometer bias correction & normalization ---
  float ax = ((filtered_accelX - axBias) * axScale) / ACCEL_SENSITIVITY;
  float ay = ((filtered_accelY - ayBias) * ayScale) / ACCEL_SENSITIVITY;
  float az = ((filtered_accelZ - azBias) * azScale) / ACCEL_SENSITIVITY;

  float accelNorm = sqrt(ax*ax + ay*ay + az*az);
  if (accelNorm == 0.0f) return;

  float ax_n = ax / accelNorm;
  float ay_n = ay / accelNorm;
  float az_n = az / accelNorm;

  // --- Impact Detection ---
  float accel_deviation = fabs(accelNorm - 1.0f);
  float gyro_magnitude = sqrt((float)gyroX*gyroX + (float)gyroY*gyroY + (float)gyroZ*gyroZ) / GYRO_SENSITIVITY;

  if (accel_deviation > ACCEL_IMPACT_THRESHOLD_EKF || gyro_magnitude > GYRO_IMPACT_THRESHOLD_EKF) {
    lastImpactTime_ekf = millis();
  }
  bool inImpactBoost = (millis() - lastImpactTime_ekf) < IMPACT_BOOST_DURATION_EKF;

  // Dynamic noise scaling during impact
  float r_accel = ekf_r_accel;
  float q_gyro  = ekf_q_gyro;
  if (inImpactBoost) {
    r_accel *= 0.2f;   // Trust accel 5x more during impact recovery
    q_gyro  *= 5.0f;   // Trust gyro less during impact
  }

  // --- Gyro rates using global axis mapping (same as other quaternion filters) ---
  float gyro_roll_deg  = (GYRO_ROLL_RATE_USED  / GYRO_SENSITIVITY) - GYRO_ROLL_BIAS_USED;
  float gyro_pitch_deg = (GYRO_PITCH_RATE_USED / GYRO_SENSITIVITY) - GYRO_PITCH_BIAS_USED;
  float gyro_yaw_deg   = (GYRO_YAW_RATE_USED   / GYRO_SENSITIVITY) - GYRO_YAW_BIAS_USED;

  // Convert to rad/s and subtract EKF bias estimate
  float wx = gyro_roll_deg  / 57.2958f - ekf_bias[0];
  float wy = gyro_pitch_deg / 57.2958f - ekf_bias[1];
  float wz = gyro_yaw_deg   / 57.2958f - ekf_bias[2];

  // ============================================================
  // PREDICT STEP
  // ============================================================

  // Quaternion derivative: q_dot = 0.5 * q (x) [0, wx, wy, wz]
  float q0 = ekf_q[0], q1 = ekf_q[1], q2 = ekf_q[2], q3 = ekf_q[3];

  float qDot0 = 0.5f * (-q1*wx - q2*wy - q3*wz);
  float qDot1 = 0.5f * ( q0*wx + q2*wz - q3*wy);
  float qDot2 = 0.5f * ( q0*wy - q1*wz + q3*wx);
  float qDot3 = 0.5f * ( q0*wz + q1*wy - q2*wx);

  // Integrate quaternion
  ekf_q[0] += qDot0 * dt;
  ekf_q[1] += qDot1 * dt;
  ekf_q[2] += qDot2 * dt;
  ekf_q[3] += qDot3 * dt;
  ekf_normalize_quat();

  // --- Jacobian F (7x7) of state transition ---
  // F = d(f(x))/dx evaluated at current state
  // For quaternion rows: F_q = I + 0.5*dt*Omega where Omega is the angular rate matrix
  // For bias rows: F_bias = I (random walk model)
  //
  // We compute F*P*F' + Q in a memory-efficient way using the structure
  float hdt = 0.5f * dt;

  // Build F matrix (sparse - mostly identity)
  // F[0:4, 0:4] = I + hdt * Omega(w)
  // F[0:4, 4:7] = -hdt * Xi(q)  (quaternion-bias coupling)
  // F[4:7, 4:7] = I

  // Omega matrix entries (angular velocity matrix for quaternion kinematics)
  // Omega = [  0, -wx, -wy, -wz ]
  //         [ wx,   0,  wz, -wy ]
  //         [ wy, -wz,   0,  wx ]
  //         [ wz,  wy, -wx,   0 ]

  // Xi matrix (maps bias to quaternion rate)
  // Xi = [ -q1, -q2, -q3 ]
  //      [  q0,  q3, -q2 ]
  //      [ -q3,  q0,  q1 ]
  //      [  q2, -q1,  q0 ]

  // For embedded efficiency: directly compute P_new = F*P*F' + Q
  // Using the fact that F is close to identity, compute F*P first

  float F[7][7];
  // Initialize to identity
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++)
      F[i][j] = (i == j) ? 1.0f : 0.0f;

  // Quaternion-to-quaternion block (4x4)
  F[0][1] = -hdt*wx;  F[0][2] = -hdt*wy;  F[0][3] = -hdt*wz;
  F[1][0] =  hdt*wx;  F[1][2] =  hdt*wz;  F[1][3] = -hdt*wy;
  F[2][0] =  hdt*wy;  F[2][1] = -hdt*wz;  F[2][3] =  hdt*wx;
  F[3][0] =  hdt*wz;  F[3][1] =  hdt*wy;  F[3][2] = -hdt*wx;

  // Quaternion-to-bias coupling block (4x3) — how gyro bias affects quaternion
  // d(qDot)/d(bias) = -0.5 * Xi(q)
  q0 = ekf_q[0]; q1 = ekf_q[1]; q2 = ekf_q[2]; q3 = ekf_q[3];  // use updated q

  F[0][4] =  hdt*q1;  F[0][5] =  hdt*q2;  F[0][6] =  hdt*q3;
  F[1][4] = -hdt*q0;  F[1][5] = -hdt*q3;  F[1][6] =  hdt*q2;
  F[2][4] =  hdt*q3;  F[2][5] = -hdt*q0;  F[2][6] = -hdt*q1;
  F[3][4] = -hdt*q2;  F[3][5] =  hdt*q1;  F[3][6] = -hdt*q0;

  // Compute FP = F * P
  float FP[7][7];
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 7; k++)
        sum += F[i][k] * ekf_P[k][j];
      FP[i][j] = sum;
    }

  // Compute P_pred = FP * F' + Q
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 7; k++)
        sum += FP[i][k] * F[j][k];  // F' means transpose, so F[j][k]
      ekf_P[i][j] = sum;
    }

  // Add process noise Q (diagonal)
  for (int i = 0; i < 4; i++) ekf_P[i][i] += q_gyro * dt;
  for (int i = 4; i < 7; i++) ekf_P[i][i] += ekf_q_bias * dt;

  // ============================================================
  // UPDATE STEP (Accelerometer measurement)
  // ============================================================
  // Measurement model: h(x) = predicted gravity in body frame from quaternion
  // g_body = R(q)' * [0, 0, 1]  (gravity unit vector in body frame)

  q0 = ekf_q[0]; q1 = ekf_q[1]; q2 = ekf_q[2]; q3 = ekf_q[3];

  // Predicted gravity direction in body frame
  float hx = 2.0f * (q1*q3 - q0*q2);
  float hy = 2.0f * (q0*q1 + q2*q3);
  float hz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Innovation (measurement residual) y = z - h(x)
  float y0 = ax_n - hx;
  float y1 = ay_n - hy;
  float y2 = az_n - hz;

  // --- Accel confidence weighting ---
  // Reduce trust when total accel deviates from 1g (free-fall, vibration, maneuver)
  float accel_err = fabs(accelNorm - 1.0f);
  float accel_confidence = 1.0f - constrain(accel_err * accel_err * 2.0f, 0.0f, 0.7f);
  float r_eff = r_accel / accel_confidence;  // Higher R when not at 1g

  // Measurement Jacobian H (3x7): d(h)/d(x)
  // h(x) = gravity from quaternion, bias doesn't directly affect accel measurement
  // H[0:3, 0:4] = d(g_body)/d(q),  H[0:3, 4:7] = 0
  float H[3][7];
  memset(H, 0, sizeof(H));

  // d(hx)/dq
  H[0][0] = -2.0f*q2;  H[0][1] =  2.0f*q3;  H[0][2] = -2.0f*q0;  H[0][3] =  2.0f*q1;
  // d(hy)/dq
  H[1][0] =  2.0f*q1;  H[1][1] =  2.0f*q0;  H[1][2] =  2.0f*q3;  H[1][3] =  2.0f*q2;
  // d(hz)/dq
  H[2][0] =  2.0f*q0;  H[2][1] = -2.0f*q1;  H[2][2] = -2.0f*q2;  H[2][3] =  2.0f*q3;

  // Compute S = H * P * H' + R  (3x3)
  // First: HP = H * P (3x7)
  float HP[3][7];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 7; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 7; k++)
        sum += H[i][k] * ekf_P[k][j];
      HP[i][j] = sum;
    }

  // S = HP * H' + R (3x3)
  float S[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 7; k++)
        sum += HP[i][k] * H[j][k];  // H' = transpose
      S[i][j] = sum + ((i == j) ? r_eff : 0.0f);
    }

  // Invert S (3x3) using cofactor method
  float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
            - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
            + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);

  if (fabs(det) < 1e-10f) return;  // Singular, skip update

  float invDet = 1.0f / det;
  float Si[3][3];
  Si[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invDet;
  Si[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invDet;
  Si[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invDet;
  Si[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invDet;
  Si[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invDet;
  Si[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invDet;
  Si[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invDet;
  Si[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invDet;
  Si[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invDet;

  // Kalman Gain K = P * H' * S^-1  (7x3)
  // First: PH' = P * H' (7x3)
  float PHt[7][3];
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 7; k++)
        sum += ekf_P[i][k] * H[j][k];  // H' = transpose
      PHt[i][j] = sum;
    }

  // K = PH' * S^-1 (7x3)
  float K[7][3];
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 3; k++)
        sum += PHt[i][k] * Si[k][j];
      K[i][j] = sum;
    }

  // State update: x = x + K * y
  float dy[3] = {y0, y1, y2};
  for (int i = 0; i < 4; i++) {
    float correction = 0.0f;
    for (int j = 0; j < 3; j++)
      correction += K[i][j] * dy[j];
    ekf_q[i] += correction;
  }
  for (int i = 0; i < 3; i++) {
    float correction = 0.0f;
    for (int j = 0; j < 3; j++)
      correction += K[4+i][j] * dy[j];
    ekf_bias[i] += correction;
  }

  // Re-normalize quaternion after update
  ekf_normalize_quat();

  // Constrain bias estimates
  float bias_limit = inImpactBoost ? 0.35f : 0.09f;  // ~20 deg/s : ~5 deg/s in rad/s
  for (int i = 0; i < 3; i++)
    ekf_bias[i] = constrain(ekf_bias[i], -bias_limit, bias_limit);

  // Covariance update: P = (I - K*H) * P
  // Joseph form for numerical stability: P = (I-KH)*P*(I-KH)' + K*R*K'
  // Simplified: P = P - K*HP (adequate for embedded)
  float KHP[7][7];
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 3; k++)
        sum += K[i][k] * HP[k][j];
      KHP[i][j] = sum;
    }

  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++)
      ekf_P[i][j] -= KHP[i][j];

  // Enforce symmetry (prevent numerical drift)
  for (int i = 0; i < 7; i++)
    for (int j = i+1; j < 7; j++) {
      float avg = 0.5f * (ekf_P[i][j] + ekf_P[j][i]);
      ekf_P[i][j] = avg;
      ekf_P[j][i] = avg;
    }

  // ============================================================
  // Convert quaternion to Euler angles (same convention as other filters)
  // ============================================================
  q0 = ekf_q[0]; q1 = ekf_q[1]; q2 = ekf_q[2]; q3 = ekf_q[3];

  // Roll (phi)
  roll = atan2(2.0f * (q0*q1 + q2*q3),
               1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / PI;

  // Pitch (theta)
  float pitch_arg = 2.0f * (q0*q2 - q3*q1);
  pitch_arg = constrain(pitch_arg, -1.0f, 1.0f);
  pitch = -asin(pitch_arg) * 180.0f / PI;

  // Yaw (psi)
  yaw = -atan2(2.0f * (q0*q3 + q1*q2),
              1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / PI;
}
