#include "calibration.h"
#include "../config/config.h"  // ACCEL_SENSITIVITY, GYRO_SENSITIVITY, ACCEL_RANGE_G

CalibrationData calibData;
unsigned long lastPrintTime = 0;
unsigned long calibrationSaveTime = 0;  // Time when user clicked 'save'
bool collectingSamples = false;  // Flag: currently collecting samples after save
int32_t accelXSum = 0, accelYSum = 0, accelZSum = 0;
int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
int sampleCount = 0;

// Definitions live here (were in balancing_robot.ino); externs in calibration.h
CalibrationState calibrationState = CALIB_IDLE;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float axBias = 0.0f, axScale = 1.0f;
float ayBias = 0.0f, ayScale = 1.0f;
float azBias = 0.0f, azScale = 1.0f;
float baro_altitude_scale = 1.0f;
float accel_z_bias_cal_mps2 = 0.0f;

// NVS handle defined here (was in balancing_robot.ino); extern in calibration.h
Preferences prefs;

// ===== Print calibration menu =====
void printCalibrationMenu() {
  Serial.println("\n========================================");
  Serial.println("6-Step Sensor Calibration");
  Serial.println("========================================");
  Serial.println("\nType 'calibrate' to start calibration");
  Serial.println("\nDuring calibration:");
  Serial.println("  'save'  - Record current step");
  Serial.println("  'abort' - Cancel calibration");
  Serial.println("========================================\n");
}

// (Gyro flow: see calibration_gyro.cpp)
// (Accel start + validation + save: see calibration_accel.cpp)

// ===== Process calibration step =====
void processCalibrationStep(const String& command) {
  if (command == "save") {
    // User pressed save - validate accel orientation before starting collection
    if (calibrationState != CALIB_GYRO) {
      // Check if drone is in the correct orientation for this step
      if (!validateAccelOrientationForStep()) {
        // Wrong orientation - user must retry
        return;
      }

      calibrationSaveTime = millis();
      collectingSamples = true;
      accelXSum = accelYSum = accelZSum = 0;
      gyroXSum = gyroYSum = gyroZSum = 0;
      sampleCount = 0;
      Serial.println("\n[CALIBRATION] Orientation validated! Starting sample collection (2 sec stabilization delay)...");
    }
  }
  else if (command == "abort") {
    calibrationState = CALIB_IDLE;
    collectingSamples = false;
    Serial.println("\n[ERROR] Calibration ABORTED!");
  }
}

// ===== Record current step data =====
void recordCalibrationStep() {
  // Calculate averages
  float samples = (float)max(sampleCount, 1);
  float avgAccelX = (float)accelXSum / samples;
  float avgAccelY = (float)accelYSum / samples;
  float avgAccelZ = (float)accelZSum / samples;
  float avgGyroX = (float)gyroXSum / samples;
  float avgGyroY = (float)gyroYSum / samples;
  float avgGyroZ = (float)gyroZSum / samples;

  Serial.print("\n[RECORDED] Samples: "); Serial.println(sampleCount);

  // GYRO CALIBRATION - Convert from raw LSB to deg/s (÷16.4 for ±2000°/s range)
  if (calibrationState == CALIB_GYRO) {
    calibData.gyroBiasX = avgGyroX / GYRO_SENSITIVITY;
    calibData.gyroBiasY = avgGyroY / GYRO_SENSITIVITY;
    calibData.gyroBiasZ = avgGyroZ / GYRO_SENSITIVITY;
    Serial.print("[GYRO RAW] Bias X: "); Serial.print(avgGyroX, 2);
    Serial.print(" LSB  →  [GYRO DEG/S] Bias X: "); Serial.println(calibData.gyroBiasX, 2);
    Serial.print("[GYRO RAW] Bias Y: "); Serial.print(avgGyroY, 2);
    Serial.print(" LSB  →  [GYRO DEG/S] Bias Y: "); Serial.println(calibData.gyroBiasY, 2);
    Serial.print("[GYRO RAW] Bias Z: "); Serial.print(avgGyroZ, 2);
    Serial.print(" LSB  →  [GYRO DEG/S] Bias Z: "); Serial.println(calibData.gyroBiasZ, 2);
  }

  // ACCEL CALIBRATION (6 steps)
  else if (calibrationState == CALIB_ACCEL_STEP1) {
    calibData.accelXLevel = avgAccelX;
    calibData.accelYLevel = avgAccelY;
    calibData.accelZLevel = avgAccelZ;
    Serial.print("[ACCEL] Step 1 - Level: Ax="); Serial.print(avgAccelX);
    Serial.print(", Ay="); Serial.print(avgAccelY);
    Serial.print(", Az="); Serial.println(avgAccelZ);
  }
  else if (calibrationState == CALIB_ACCEL_STEP2) {
    calibData.accelXPos = avgAccelX;
    Serial.print("[ACCEL] Step 2 - Pitch +90°: Ax = "); Serial.println(avgAccelX);
  }
  else if (calibrationState == CALIB_ACCEL_STEP3) {
    calibData.accelXNeg = avgAccelX;
    Serial.print("[ACCEL] Step 3 - Pitch -90°: Ax = "); Serial.println(avgAccelX);
  }
  else if (calibrationState == CALIB_ACCEL_STEP4) {
    calibData.accelYPos = avgAccelY;
    Serial.print("[ACCEL] Step 4 - Roll +90°: Ay = "); Serial.println(avgAccelY);
  }
  else if (calibrationState == CALIB_ACCEL_STEP5) {
    calibData.accelYNeg = avgAccelY;
    Serial.print("[ACCEL] Step 5 - Roll -90°: Ay = "); Serial.println(avgAccelY);
  }
  else if (calibrationState == CALIB_ACCEL_STEP6) {
    calibData.accelXUpside = avgAccelX;
    calibData.accelYUpside = avgAccelY;
    calibData.accelZUpside = avgAccelZ;
    Serial.print("[ACCEL] Step 6 - Upside Down: Ax="); Serial.print(avgAccelX);
    Serial.print(", Ay="); Serial.print(avgAccelY);
    Serial.print(", Az="); Serial.println(avgAccelZ);
  }
}

// ===== Advance to next calibration step =====
void advanceCalibrationStep() {
  accelXSum = accelYSum = accelZSum = 0;
  gyroXSum = gyroYSum = gyroZSum = 0;
  sampleCount = 0;
  lastPrintTime = millis();

  // GYRO calibration - just one step, save and exit
  if (calibrationState == CALIB_GYRO) {
    saveGyroCalibrationToPreferences();
    calibrationState = CALIB_IDLE;
    return;
  }

  // ACCEL calibration - advance through 6 steps
  calibrationState = (CalibrationState)(calibrationState + 1);

  if (calibrationState > CALIB_ACCEL_STEP6) {
    saveAccelCalibrationToPreferences();
    calibrationState = CALIB_IDLE;
    return;
  }

  // Print next step instructions
  Serial.println("\n========================================");
  switch (calibrationState) {
    case CALIB_ACCEL_STEP2:
      Serial.println("STEP 2/6: Pitch Calibration (+90°)");
      Serial.println("========================================");
      Serial.println("Tilt robot FORWARD until 90 degrees (perpendicular)");
      Serial.println("Type 'save' to record Accel X at +90°");
      break;

    case CALIB_ACCEL_STEP3:
      Serial.println("STEP 3/6: Pitch Calibration (-90°)");
      Serial.println("========================================");
      Serial.println("Tilt robot BACKWARD until -90 degrees (perpendicular)");
      Serial.println("Type 'save' to record Accel X at -90°");
      break;

    case CALIB_ACCEL_STEP4:
      Serial.println("STEP 4/6: Roll Calibration (+90°)");
      Serial.println("========================================");
      Serial.println("Tilt robot RIGHT until 90 degrees (perpendicular)");
      Serial.println("Type 'save' to record Accel Y at +90°");
      break;

    case CALIB_ACCEL_STEP5:
      Serial.println("STEP 5/6: Roll Calibration (-90°)");
      Serial.println("========================================");
      Serial.println("Tilt robot LEFT until 90 degrees (perpendicular)");
      Serial.println("Type 'save' to record Accel Y at -90°");
      break;

    case CALIB_ACCEL_STEP6:
      Serial.println("STEP 6/6: Upside Down Calibration");
      Serial.println("========================================");
      Serial.println("Flip robot UPSIDE DOWN and keep STILL");
      Serial.println("Type 'save' to record Accel Z inverted");
      break;

    default:
      break;
  }
  Serial.println("Type 'abort' to cancel\n");
}

// ===== Update calibration (called every loop) =====
void updateCalibration() {
  if (calibrationState == CALIB_IDLE) return;

  // GYRO CALIBRATION: Collect 2000 samples automatically
  if (calibrationState == CALIB_GYRO) {
    if (collectingSamples) {
      // Collect raw gyro samples
      gyroXSum += mpu_gyroX;
      gyroYSum += mpu_gyroY;
      gyroZSum += mpu_gyroZ;
      sampleCount++;

      // Print progress every 500 samples
      if (sampleCount % 500 == 0) {
        Serial.print("[GYRO] Collected "); Serial.print(sampleCount); Serial.println(" samples...");
      }

      // After 2000 samples, stop and save
      if (sampleCount >= 2000) {
        recordCalibrationStep();
        collectingSamples = false;
        advanceCalibrationStep();
      }
    }
    return;
  }

  // ACCEL CALIBRATION: Original timing logic
  if (collectingSamples) {
    unsigned long timeSinceSave = millis() - calibrationSaveTime;

    // For ACCEL: normal stabilization (2 sec) + sampling (3 sec) = 5 sec total
    if (timeSinceSave >= 2000 && timeSinceSave < 5000) {
      // Collect samples during this window
      accelXSum += mpu_accelX;
      accelYSum += mpu_accelY;
      accelZSum += mpu_accelZ;
      gyroXSum += mpu_gyroX;
      gyroYSum += mpu_gyroY;
      gyroZSum += mpu_gyroZ;
      sampleCount++;

      // Print live values every 1 second (only for accel steps)
      if (millis() - lastPrintTime >= 1000 && calibrationState != CALIB_GYRO) {
        lastPrintTime = millis();
        if (calibrationState == CALIB_ACCEL_STEP2 || calibrationState == CALIB_ACCEL_STEP3) {
          Serial.print("[COLLECTING] Ax = "); Serial.println(mpu_accelX);
        }
        else if (calibrationState == CALIB_ACCEL_STEP4 || calibrationState == CALIB_ACCEL_STEP5) {
          Serial.print("[COLLECTING] Ay = "); Serial.println(mpu_accelY);
        }
      }
    }
    // After 5 seconds for accel, recording is complete
    else if (timeSinceSave >= 5000 && calibrationState != CALIB_GYRO) {
      recordCalibrationStep();
      collectingSamples = false;
      advanceCalibrationStep();
    }
  }
}

// (Gyro save: see calibration_gyro.cpp; Accel save: see calibration_accel.cpp)

// ===== Reset Calibration to defaults (moved from pid_tuning.h) =====
void resetCalibrationToDefaults() {
  prefs.begin("mpu6050", false);  // Write mode
  prefs.putFloat("gyroBiasX", 0.0);
  prefs.putFloat("gyroBias", 0.0);
  prefs.putFloat("gyroBiasZ", 0.0);
  prefs.putFloat("axBias", 0.0);
  prefs.putFloat("axScale", 1.0);
  prefs.putFloat("ayBias", 0.0);
  prefs.putFloat("ayScale", 1.0);
  prefs.putFloat("azBias", 0.0);
  prefs.putFloat("azScale", 1.0);
  prefs.putFloat("baroScale", 1.0);
  prefs.putFloat("altAzBias", 0.0);
  prefs.end();

  Serial.println("\n[SUCCESS] Calibration reset to default values!");
  Serial.println("Please restart the ESP32 to load defaults\n");
}

// ===== Load calibration bias from NVS (moved from setup() verbatim) =====
void loadCalibrationBias() {
  prefs.begin("mpu6050", true);  // Read-only mode
  gyroBiasY = prefs.getFloat("gyroBiasY", prefs.getFloat("gyroBias", 0.0f));
  gyroBiasX = prefs.getFloat("gyroBiasX", 0.0f);
  gyroBiasZ = prefs.getFloat("gyroBiasZ", 0.0f);
  axBias = prefs.getFloat("axBias", 0.0f);
  axScale = prefs.getFloat("axScale", 1.0f);
  ayBias = prefs.getFloat("ayBias", 0.0f);
  ayScale = prefs.getFloat("ayScale", 1.0f);
  azBias = prefs.getFloat("azBias", 0.0f);
  azScale = prefs.getFloat("azScale", 1.0f);
  baro_altitude_scale = prefs.getFloat("baroScale", 1.0f);
  accel_z_bias_cal_mps2 = prefs.getFloat("altAzBias", 0.0f);
  trim_pitch = prefs.getFloat("trim_pitch", 0.0f);
  trim_roll = prefs.getFloat("trim_roll", 0.0f);
  prefs.end();

  Serial.println("[OK] Calibration bias loaded from preferences");
  Serial.println("[BARO_CAL] Loaded offsets from NVS:");
  Serial.print("[BARO_CAL] baroScale(avg) = "); Serial.println(baro_altitude_scale, 5);
  Serial.print("[BARO_CAL] accelZBias_mps2(avg) = "); Serial.println(accel_z_bias_cal_mps2, 5);

  // Print all loaded bias values
  Serial.println("\n===== Loaded Calibration Bias =====");
  Serial.print("Gyro Bias X: "); Serial.println(gyroBiasX);
  Serial.print("Gyro Bias Y: "); Serial.println(gyroBiasY);
  Serial.print("Gyro Bias Z: "); Serial.println(gyroBiasZ);
  Serial.print("Accel X Bias: "); Serial.println(axBias);
  Serial.print("Accel X Scale: "); Serial.println(axScale);
  Serial.print("Accel Y Bias: "); Serial.println(ayBias);
  Serial.print("Accel Y Scale: "); Serial.println(ayScale);
  Serial.print("Accel Z Bias: "); Serial.println(azBias);
  Serial.print("Accel Z Scale: "); Serial.println(azScale);
  Serial.print("Baro Scale: "); Serial.println(baro_altitude_scale, 5);
  Serial.print("Alt Z-Accel Bias (m/s^2): "); Serial.println(accel_z_bias_cal_mps2, 5);
  Serial.println("\n===== Trimmed Angle Bias =====");
  Serial.print("Trim Pitch: "); Serial.print(trim_pitch, 4); Serial.println("°");
  Serial.print("Trim Roll: "); Serial.print(trim_roll, 4); Serial.println("°");
  Serial.println("===================================\n");
}
