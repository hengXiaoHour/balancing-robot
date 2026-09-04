#include "calibration.h"
#include "../config/config.h"  // ACCEL_SENSITIVITY, ACCEL_RANGE_G

// ===== Start ACCEL-ONLY calibration (6 steps) =====
void startAccelCalibration() {
  calibrationState = CALIB_ACCEL_STEP1;
  accelXSum = accelYSum = accelZSum = 0;
  gyroXSum = gyroYSum = gyroZSum = 0;
  sampleCount = 0;
  collectingSamples = false;
  lastPrintTime = millis();

  Serial.println("\n========================================");
  Serial.println("ACCELEROMETER SCALE CALIBRATION");
  Serial.println("========================================");
  Serial.println("6-step accel calibration (GYRO NOT INVOLVED)");
  Serial.println("\nCalibration Process:");
  Serial.println("  1. Type 'save' - starts 2-second stabilization");
  Serial.println("  2. Then 3-second sample collection");
  Serial.println("  3. Auto-advances to next step");
  Serial.println("  4. Repeat for all 6 orientations");
  Serial.println("\nType 'save' to record | Type 'abort' to cancel\n");

  // Print first step
  Serial.println("\n========================================");
  Serial.println("STEP 1/6: Level Position");
  Serial.println("========================================");
  Serial.println("Keep robot LEVEL and STILL");
  Serial.println("Type 'save' to record baseline accel values");
  Serial.println("Type 'abort' to cancel\n");
}

// ===== Validate accel orientation for current calibration step =====
bool validateAccelOrientationForStep() {
  // Orientation checks are intentionally tolerant to slight frame/sensor misalignment.
  // We validate sign + dominant axis + near-1g magnitude instead of forcing non-target axis ~= 0.
  const int16_t AXIS_THRESHOLD = 8000;
  const int16_t LEVEL_THRESHOLD = 3500;      // Level axis should be small
  const int16_t DOMINANCE_DELTA = 500;       // Target axis should exceed others by this margin
  const float G_NORM_TOLERANCE = 0.35f;      // Accept 0.65g .. 1.35g as static orientation

  int16_t absX = abs(mpu_accelX);
  int16_t absY = abs(mpu_accelY);
  int16_t absZ = abs(mpu_accelZ);

  float ax = (float)mpu_accelX;
  float ay = (float)mpu_accelY;
  float az = (float)mpu_accelZ;
  float accelNorm = sqrtf((ax * ax) + (ay * ay) + (az * az));
  float minNorm = ACCEL_SENSITIVITY * (1.0f - G_NORM_TOLERANCE);
  float maxNorm = ACCEL_SENSITIVITY * (1.0f + G_NORM_TOLERANCE);
  bool gravityMagnitudeValid = (accelNorm >= minNorm) && (accelNorm <= maxNorm);

  switch (calibrationState) {
    case CALIB_ACCEL_STEP1:
      // Level: Z should be dominant (gravity up), X and Y small
      if (absZ > AXIS_THRESHOLD && absX < LEVEL_THRESHOLD && absY < LEVEL_THRESHOLD) {
        return true;
      }
      Serial.print("[CALIB_CHECK] STEP 1: Expected Z-axis dominant (level). Got Ax=");
      Serial.print(mpu_accelX); Serial.print(" Ay="); Serial.print(mpu_accelY);
      Serial.print(" Az="); Serial.println(mpu_accelZ);
      Serial.println("[RETRY] Please keep robot LEVEL and try again.");
      return false;

    case CALIB_ACCEL_STEP2:
      // Pitch +90°: X should be positive and dominant
      if (mpu_accelX > AXIS_THRESHOLD &&
          absX > (absY + DOMINANCE_DELTA) &&
          absX > (absZ + DOMINANCE_DELTA) &&
          gravityMagnitudeValid) {
        return true;
      }
      Serial.print("[CALIB_CHECK] STEP 2: Need Ax positive + dominant. Got Ax=");
      Serial.print(mpu_accelX); Serial.print(" Ay="); Serial.print(mpu_accelY);
      Serial.print(" Az="); Serial.print(mpu_accelZ);
      Serial.print(" | |A|="); Serial.println(accelNorm, 0);
      Serial.println("[RETRY] Please tilt robot FORWARD (+90°) and try again.");
      return false;

    case CALIB_ACCEL_STEP3:
      // Pitch -90°: X should be negative and dominant
      if (mpu_accelX < -AXIS_THRESHOLD &&
          absX > (absY + DOMINANCE_DELTA) &&
          absX > (absZ + DOMINANCE_DELTA) &&
          gravityMagnitudeValid) {
        return true;
      }
      Serial.print("[CALIB_CHECK] STEP 3: Need Ax negative + dominant. Got Ax=");
      Serial.print(mpu_accelX); Serial.print(" Ay="); Serial.print(mpu_accelY);
      Serial.print(" Az="); Serial.print(mpu_accelZ);
      Serial.print(" | |A|="); Serial.println(accelNorm, 0);
      Serial.println("[RETRY] Please tilt robot BACKWARD (-90°) and try again.");
      return false;

    case CALIB_ACCEL_STEP4:
      // Roll +90°: Y should be positive and dominant
      if (mpu_accelY > AXIS_THRESHOLD &&
          absY > (absX + DOMINANCE_DELTA) &&
          absY > (absZ + DOMINANCE_DELTA) &&
          gravityMagnitudeValid) {
        return true;
      }
      Serial.print("[CALIB_CHECK] STEP 4: Need Ay positive + dominant. Got Ax=");
      Serial.print(mpu_accelX); Serial.print(" Ay="); Serial.print(mpu_accelY);
      Serial.print(" Az="); Serial.print(mpu_accelZ);
      Serial.print(" | |A|="); Serial.println(accelNorm, 0);
      Serial.println("[RETRY] Please tilt robot RIGHT (+90°) and try again.");
      return false;

    case CALIB_ACCEL_STEP5:
      // Roll -90°: Y should be negative and dominant
      if (mpu_accelY < -AXIS_THRESHOLD &&
          absY > (absX + DOMINANCE_DELTA) &&
          absY > (absZ + DOMINANCE_DELTA) &&
          gravityMagnitudeValid) {
        return true;
      }
      Serial.print("[CALIB_CHECK] STEP 5: Need Ay negative + dominant. Got Ax=");
      Serial.print(mpu_accelX); Serial.print(" Ay="); Serial.print(mpu_accelY);
      Serial.print(" Az="); Serial.print(mpu_accelZ);
      Serial.print(" | |A|="); Serial.println(accelNorm, 0);
      Serial.println("[RETRY] Please tilt robot LEFT (-90°) and try again.");
      return false;

    case CALIB_ACCEL_STEP6:
      // Upside down: Z should be negative and large, X and Y small
      if (mpu_accelZ < -AXIS_THRESHOLD && absX < LEVEL_THRESHOLD && absY < LEVEL_THRESHOLD) {
        return true;
      }
      Serial.print("[CALIB_CHECK] STEP 6: Expected Z-axis negative dominant (upside down). Got Ax=");
      Serial.print(mpu_accelX); Serial.print(" Ay="); Serial.print(mpu_accelY);
      Serial.print(" Az="); Serial.println(mpu_accelZ);
      Serial.println("[RETRY] Please flip robot UPSIDE DOWN and try again.");
      return false;

    default:
      return true;  // GYRO or other states pass through
  }
}

// ===== Save ACCEL calibration only =====
void saveAccelCalibrationToPreferences() {
  Serial.println("\n========================================");
  Serial.println("Calculating Accelerometer Parameters...");
  Serial.println("========================================");

  // Calculate accel scale and bias using configured accel full-scale range.
  float accelSensitivity = 16384.0f;  // ±2g default
  if (ACCEL_RANGE_G == 4) accelSensitivity = 8192.0f;
  else if (ACCEL_RANGE_G == 8) accelSensitivity = 4096.0f;
  else if (ACCEL_RANGE_G == 16) accelSensitivity = 2048.0f;
  float target_90deg = accelSensitivity;  // Expected 1g at 90°

  float axScale = target_90deg / ((calibData.accelXPos - calibData.accelXNeg) / 2.0);
  float axBias = (calibData.accelXPos + calibData.accelXNeg) / 2.0;

  float ayScale = target_90deg / ((calibData.accelYPos - calibData.accelYNeg) / 2.0);
  float ayBias = (calibData.accelYPos + calibData.accelYNeg) / 2.0;

  // Z-axis: average of level and upside-down
  float azBias = (calibData.accelZLevel + calibData.accelZUpside) / 2.0;
  float azScale = 1.0;  // Z-axis doesn't need scaling in this calibration

  // Print calculated values
  Serial.print("Accel X Bias: "); Serial.println(axBias, 2);
  Serial.print("Accel X Scale: "); Serial.println(axScale, 2);
  Serial.print("Accel Y Bias: "); Serial.println(ayBias, 2);
  Serial.print("Accel Y Scale: "); Serial.println(ayScale, 2);
  Serial.print("Accel Z Bias: "); Serial.println(azBias, 2);
  Serial.print("Accel Z Scale: "); Serial.println(azScale, 2);

  // Save to preferences (keep existing gyro bias)
  prefs.begin("mpu6050", false);  // Write mode
  prefs.putFloat("axBias", axBias);
  prefs.putFloat("axScale", axScale);
  prefs.putFloat("ayBias", ayBias);
  prefs.putFloat("ayScale", ayScale);
  prefs.putFloat("azBias", azBias);
  prefs.putFloat("azScale", azScale);
  prefs.end();
  delay(100);  // Critical: Allow NVS to flush on ESP32-C3

  Serial.println("\n[DEBUG] Accel bias and scale values written to NVS");
  Serial.println("[SUCCESS] Accelerometer calibration saved!");
  Serial.println("========================================\n");
}
