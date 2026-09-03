#ifndef PID_TUNING_H
#define PID_TUNING_H

#include <Preferences.h>

// Default PID values (single controller)
#define DEFAULT_KP 8.0
#define DEFAULT_KI 0.2
#define DEFAULT_KD 5.0

// External variables (Single PID)
extern float KP, KI, KD;
extern float motorScale_Left, motorScale_Right;

// External variables (Dual-Axis PID)
extern float KP_Pitch, KI_Pitch, KD_Pitch;
extern float KP_Roll, KI_Roll, KD_Roll;

extern Preferences prefs;

// ===== Reset Calibration to defaults =====
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
