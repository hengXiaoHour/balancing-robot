#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>  // int32_t, String (standalone .cpp inclusion)
#include <Preferences.h>

// Calibration states
enum CalibrationState {
  CALIB_IDLE,
  CALIB_GYRO,           // Gyro-only calibration
  CALIB_ACCEL_STEP1,    // Level position
  CALIB_ACCEL_STEP2,    // Pitch positive (tilt forward +90°)
  CALIB_ACCEL_STEP3,    // Pitch negative (tilt backward -90°)
  CALIB_ACCEL_STEP4,    // Roll positive (tilt right +90°)
  CALIB_ACCEL_STEP5,    // Roll negative (tilt left -90°)
  CALIB_ACCEL_STEP6     // Upside down
};

// External variables
extern CalibrationState calibrationState;
extern Preferences prefs;
extern int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
extern int16_t mpu_gyroX, mpu_gyroY, mpu_gyroZ;

// Bias values loaded from NVS at boot (definitions in balancing_robot.ino;
// written by calibration routines, read here)
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float axBias, axScale, ayBias, ayScale, azBias, azScale;
extern float baro_altitude_scale;
extern float accel_z_bias_cal_mps2;
extern float trim_pitch, trim_roll;

// Calibration data storage
struct CalibrationData {
  // Step 1: Stationary level
  float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
  float accelXLevel = 0, accelYLevel = 0, accelZLevel = 0;

  // Step 2-3: Pitch calibration
  float accelXPos = 0, accelXNeg = 0;

  // Step 4-5: Roll calibration
  float accelYPos = 0, accelYNeg = 0;

  // Step 6: Upside down
  float accelXUpside = 0, accelYUpside = 0, accelZUpside = 0;
};

// Calibration globals (definitions in calibration.cpp)
extern CalibrationData calibData;
extern unsigned long lastPrintTime;
extern unsigned long calibrationSaveTime;  // Time when user clicked 'save'
extern bool collectingSamples;  // Flag: currently collecting samples after save
extern int32_t accelXSum, accelYSum, accelZSum;
extern int32_t gyroXSum, gyroYSum, gyroZSum;
extern int sampleCount;

// (Default PID values live in config/config.h — single source of truth.)

// ===== Calibration API (see calibration.cpp) =====
void printCalibrationMenu();
void startGyroCalibration();
void startAccelCalibration();
bool validateAccelOrientationForStep();
void processCalibrationStep(const String& command);
void recordCalibrationStep();
void advanceCalibrationStep();
void loadCalibrationBias();  // read NVS biases at boot (called from setup())
void updateCalibration();
void saveGyroCalibrationToPreferences();
void saveAccelCalibrationToPreferences();
void resetCalibrationToDefaults();  // (moved from pid_tuning.h)

#endif
