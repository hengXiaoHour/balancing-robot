#ifndef EKF_FILTER_H
#define EKF_FILTER_H

/*
 * ===== Extended Kalman Filter (EKF) =====
 *
 * Uses quaternion state representation with full nonlinear prediction
 * and linearized (Jacobian) covariance propagation.
 *
 * State vector x = [q0, q1, q2, q3, bx, by, bz]  (7 elements)
 *   q0..q3 = orientation quaternion
 *   bx,by,bz = gyro bias estimates (rad/s)
 *
 * Prediction: quaternion integration from gyro (nonlinear)
 * Update: accelerometer gravity reference (nonlinear measurement model)
 *
 * WHY EKF over linear Kalman?
 *   - Linear Kalman linearizes the entire system (Euler angles + small angle approx)
 *   - EKF keeps the nonlinear dynamics and only linearizes the covariance propagation
 *   - Result: better accuracy at larger angles, no gimbal lock, proper uncertainty tracking
 *
 * TUNING:
 *   ekf_q_gyro  - Process noise for gyro (higher = trust gyro less, respond faster to accel)
 *   ekf_q_bias  - Process noise for bias (higher = bias adapts faster)
 *   ekf_r_accel - Measurement noise for accel (higher = trust accel less, smoother)
 */

// ===== EKF State Variables =====
extern float pitch, roll, yaw;
extern float pitch_bias, roll_bias, yaw_bias;
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float dt;
extern int16_t gyroX, gyroY, gyroZ;
extern float filtered_accelX, filtered_accelY, filtered_accelZ;
extern float axBias, ayBias, azBias;
extern float axScale, ayScale, azScale;

// Impact-detection timestamp (definition in ekf_filter.cpp)
extern unsigned long lastImpactTime_ekf;

// ===== EKF API (see ekf_filter.cpp) =====
void initEKFFilter();
void resetEKFFilter();
void updateEKFFilter();

#endif
