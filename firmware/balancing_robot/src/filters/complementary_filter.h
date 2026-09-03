#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

// ===== Complementary Filter State Variables =====
extern float pitch, roll, yaw;  // 3D angles in degrees
extern float pitch_bias, roll_bias, yaw_bias;  // Gyro biases
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float dt;
extern int16_t gyroX, gyroY, gyroZ;
extern float filtered_accelX, filtered_accelY, filtered_accelZ;
extern float axBias, ayBias, azBias;
extern float axScale, ayScale, azScale;

// ===== Complementary Filter API (see complementary_filter.cpp) =====
void updateComplementaryFilter();

#endif
