#ifndef CALIBRATION_H
#define CALIBRATION_H

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

// Forward declarations
void startGyroCalibration();
void startAccelCalibration();
void recordCalibrationStep();
void advanceCalibrationStep();
void saveGyroCalibrationToPreferences();
void saveAccelCalibrationToPreferences();

// External variables
extern CalibrationState calibrationState;
extern Preferences prefs;
extern int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
extern int16_t mpu_gyroX, mpu_gyroY, mpu_gyroZ;

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

CalibrationData calibData;
unsigned long lastPrintTime = 0;
unsigned long calibrationSaveTime = 0;  // Time when user clicked 'save'
bool collectingSamples = false;  // Flag: currently collecting samples after save
int32_t accelXSum = 0, accelYSum = 0, accelZSum = 0;
int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
int sampleCount = 0;

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
      Serial.println("Tilt robot LEFT until -90 degrees (perpendicular)");
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

// ===== Default PID values (moved from pid_tuning.h, which is deleted) =====
// NOTE: these intentionally override config.h's DEFAULT_KP/KI/KD for the axis
// gains. Do not "fix" the duplication without retuning the robot (Phase 5).
#define DEFAULT_KP 8.0
#define DEFAULT_KI 0.2
#define DEFAULT_KD 5.0

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

#endif
