#include "calibration.h"
#include "../config/settings.h"  // GYRO_SENSITIVITY

// ===== Start GYRO-ONLY calibration =====
void startGyroCalibration() {
  // Immediately start collecting samples (simpler, more direct approach)
  calibrationState = CALIB_GYRO;
  accelXSum = accelYSum = accelZSum = 0;
  gyroXSum = gyroYSum = gyroZSum = 0;
  sampleCount = 0;
  collectingSamples = true;
  calibrationSaveTime = millis();  // Start counting from now
  lastPrintTime = millis();

  Serial.println("\n========================================");
  Serial.println("GYRO BIAS CALIBRATION");
  Serial.println("========================================");
  Serial.println("Keep the robot LEVEL and ABSOLUTELY STILL");
  Serial.println("\nCalibrating gyro bias with 2000 samples...");
  Serial.println("This will take approximately 4 seconds");
  Serial.println("DO NOT MOVE THE ROBOT!");
  Serial.println("========================================\n");
}

// ===== Save GYRO calibration only =====
void saveGyroCalibrationToPreferences() {
  Serial.println("\n========================================");
  Serial.println("Saving Gyro Calibration...");
  Serial.println("========================================");

  float gyroBiasX = calibData.gyroBiasX;
  float gyroBiasY = calibData.gyroBiasY;
  float gyroBiasZ = calibData.gyroBiasZ;

  // Print in deg/s format (already converted from LSB)
  char buf[96];
  snprintf(buf, sizeof(buf), "CAL_DONE: X_Bias:%.2f deg/s, Y_Bias:%.2f deg/s, Z_Bias:%.2f deg/s", gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.println(buf);

  // Save to preferences
  prefs.begin("mpu6050", false);  // Write mode
  prefs.putFloat("gyroBiasX", gyroBiasX);
  prefs.putFloat("gyroBiasY", gyroBiasY);
  // Legacy key kept for backward compatibility with older firmware.
  prefs.putFloat("gyroBias", gyroBiasY);
  prefs.putFloat("gyroBiasZ", gyroBiasZ);
  prefs.end();
  delay(100);  // Critical: Allow NVS to flush on ESP32-C3

  Serial.println("[DEBUG] Gyro bias values written to NVS");
  Serial.println("[SUCCESS] Gyro calibration saved!");
  Serial.println("========================================\n");
}
