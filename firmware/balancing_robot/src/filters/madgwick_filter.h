#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

// ===== Madgwick Filter State Variables =====
extern float pitch, roll, yaw;  // 3D angles in degrees
extern float pitch_bias, roll_bias, yaw_bias;  // Gyro biases
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float dt;
extern int16_t gyroX, gyroY, gyroZ;
extern float filtered_accelX, filtered_accelY, filtered_accelZ;
extern float axBias, ayBias, azBias;
extern float axScale, ayScale, azScale;

// Impact-detection timestamp (definition in madgwick_filter.cpp)
extern unsigned long lastImpactTime_madgwick;

// ===== Madgwick Filter API (see madgwick_filter.cpp) =====
void initMadgwickFilter();
void updateMadgwickFilter();

#endif
