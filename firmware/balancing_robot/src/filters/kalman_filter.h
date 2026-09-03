#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// ===== Kalman Filter State Variables =====
extern float pitch, roll, yaw;  // 3D angles in degrees
extern float pitch_bias, roll_bias, yaw_bias;  // Gyro biases
extern float P[6][6];  // 6x6 covariance matrix
extern float dt;
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY, gyroZ;
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
extern float axBias, ayBias, azBias;
extern float axScale, ayScale, azScale;
extern float trim_pitch, trim_roll;  // Trim offsets for angle corrections

// ===== Kalman Filter Tuning (active parameters) =====
// Kalman Filter Process/Measurement Noise
// Typical ranges and tuning direction:
//  - KALMAN_Q_ANGLE   : typical 1e-4 .. 1e-2. Higher => faster response to accel (more responsive, noisier). Lower => smoother, trusts gyro more.
//  - KALMAN_Q_BIAS    : typical 1e-8 .. 1e-3 (up to 1e-2 on noisy systems). Higher => bias adapts faster (reduces drift), lower => bias assumed steadier.
//  - KALMAN_R_MEASURE : typical 0.005 .. 0.5. Higher => accel considered noisy (smoother, slower correction), lower => accel trusted more (faster correction, may jitter).
#define KALMAN_Q_ANGLE 0.005f   // Process noise for angle (typical 0.001 to 0.01; higher=faster response = trust accel more, lower=smoother)
#define KALMAN_Q_BIAS 0.003f    // Process noise for bias (typical 1e-8..1e-3; higher=faster bias adaptation)
#define KALMAN_R_MEASURE 0.005f  // Measurement noise (accel) (typical 0.005 to around 0.5; higher=trust accel less)

// Impact-detection timestamp (definition in kalman_filter.cpp)
extern unsigned long lastImpactTime;

// ===== Kalman Filter API (see kalman_filter.cpp) =====
void initKalmanFilter();
void updateKalmanFilter();

#endif
