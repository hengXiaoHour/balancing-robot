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

// Impact-detection timestamp (definition in mahony_filter.cpp)
extern unsigned long lastImpactTime_mahony;

// ===== Mahony Filter API (see mahony_filter.cpp) =====
void initMahonyFilter();
void resetMahonyFilter();
void updateMahonyFilter();

#endif
