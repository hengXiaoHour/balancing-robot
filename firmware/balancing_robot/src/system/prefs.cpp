#include "prefs.h"

// PID Preference Handling for ESP32/ESP32-C3
void savePIDToPreferences() {
  Preferences prefs;
  prefs.begin("pid_tuning", false);  // false = read-write mode

  // Save general PID gains
  prefs.putFloat("KP", KP);
  prefs.putFloat("KI", KI);
  prefs.putFloat("KD", KD);

  // Save Pitch PID gains
  prefs.putFloat("KP_Pitch", KP_Pitch);
  prefs.putFloat("KI_Pitch", KI_Pitch);
  prefs.putFloat("KD_Pitch", KD_Pitch);

  // Save Roll PID gains
  prefs.putFloat("KP_Roll", KP_Roll);
  prefs.putFloat("KI_Roll", KI_Roll);
  prefs.putFloat("KD_Roll", KD_Roll);

  // Save Yaw PID gains
  prefs.putFloat("KP_Yaw", KP_Yaw);
  prefs.putFloat("KI_Yaw", KI_Yaw);
  prefs.putFloat("KD_Yaw", KD_Yaw);

  #if ENABLE_CASCADE_PID
  // Save cascade pitch angle loop gains
  prefs.putFloat("cpa_kp", cascade_pitch_angle_kp);
  prefs.putFloat("cpa_ki", cascade_pitch_angle_ki);
  prefs.putFloat("cpa_kd", cascade_pitch_angle_kd);

  // Save cascade pitch rate loop gains
  prefs.putFloat("cpr_kp", cascade_pitch_rate_kp);
  prefs.putFloat("cpr_ki", cascade_pitch_rate_ki);
  prefs.putFloat("cpr_kd", cascade_pitch_rate_kd);

  // Save cascade roll angle loop gains
  prefs.putFloat("cra_kp", cascade_roll_angle_kp);
  prefs.putFloat("cra_ki", cascade_roll_angle_ki);
  prefs.putFloat("cra_kd", cascade_roll_angle_kd);

  // Save cascade roll rate loop gains
  prefs.putFloat("crr_kp", cascade_roll_rate_kp);
  prefs.putFloat("crr_ki", cascade_roll_rate_ki);
  prefs.putFloat("crr_kd", cascade_roll_rate_kd);
  #endif

  prefs.end();
  delay(100);  // Critical: Allow NVS to flush on ESP32-C3
  Serial.println("[OK] PID values saved to NVS preferences");
}

void resetPIDToDefaults() {
  // Reset to default values from settings.h
  KP = DEFAULT_KP;
  KI = DEFAULT_KI;
  KD = DEFAULT_KD;

  KP_Pitch = DEFAULT_KP_PITCH;
  KI_Pitch = DEFAULT_KI_PITCH;
  KD_Pitch = DEFAULT_KD_PITCH;

  KP_Roll = DEFAULT_KP_ROLL;
  KI_Roll = DEFAULT_KI_ROLL;
  KD_Roll = DEFAULT_KD_ROLL;

  KP_Yaw = DEFAULT_KP_YAW;
  KI_Yaw = DEFAULT_KI_YAW;
  KD_Yaw = DEFAULT_KD_YAW;

  yaw_rate_target = 0.0f;
  pitch_rate_target = 0.0f;
  roll_rate_target = 0.0f;

  #if ENABLE_CASCADE_PID
  // Reset cascade gains to compile-time defaults
  cascade_pitch_angle_kp = CASCADE_PITCH_ANGLE_KP;
  cascade_pitch_angle_ki = CASCADE_PITCH_ANGLE_KI;
  cascade_pitch_angle_kd = CASCADE_PITCH_ANGLE_KD;

  cascade_pitch_rate_kp = CASCADE_PITCH_RATE_KP;
  cascade_pitch_rate_ki = CASCADE_PITCH_RATE_KI;
  cascade_pitch_rate_kd = CASCADE_PITCH_RATE_KD;

  cascade_roll_angle_kp = CASCADE_ROLL_ANGLE_KP;
  cascade_roll_angle_ki = CASCADE_ROLL_ANGLE_KI;
  cascade_roll_angle_kd = CASCADE_ROLL_ANGLE_KD;

  cascade_roll_rate_kp = CASCADE_ROLL_RATE_KP;
  cascade_roll_rate_ki = CASCADE_ROLL_RATE_KI;
  cascade_roll_rate_kd = CASCADE_ROLL_RATE_KD;
  #endif

  // Save defaults to preferences
  savePIDToPreferences();
  delay(50);  // Extra safety delay for ESP32-C3

  Serial.println("[OK] PID values reset to defaults and saved");
}

void loadPIDFromPreferences() {
  Preferences prefs;
  prefs.begin("pid_tuning", true);  // true = read-only mode

  // Load general PID gains with defaults
  KP = prefs.getFloat("KP", DEFAULT_KP);
  KI = prefs.getFloat("KI", DEFAULT_KI);
  KD = prefs.getFloat("KD", DEFAULT_KD);

  // Load Pitch PID gains with defaults
  KP_Pitch = prefs.getFloat("KP_Pitch", DEFAULT_KP_PITCH);
  KI_Pitch = prefs.getFloat("KI_Pitch", DEFAULT_KI_PITCH);
  KD_Pitch = prefs.getFloat("KD_Pitch", DEFAULT_KD_PITCH);

  // Load Roll PID gains with defaults
  KP_Roll = prefs.getFloat("KP_Roll", DEFAULT_KP_ROLL);
  KI_Roll = prefs.getFloat("KI_Roll", DEFAULT_KI_ROLL);
  KD_Roll = prefs.getFloat("KD_Roll", DEFAULT_KD_ROLL);

  // Load Yaw PID gains with defaults
  KP_Yaw = prefs.getFloat("KP_Yaw", DEFAULT_KP_YAW);
  KI_Yaw = prefs.getFloat("KI_Yaw", DEFAULT_KI_YAW);
  KD_Yaw = prefs.getFloat("KD_Yaw", DEFAULT_KD_YAW);

  #if ENABLE_CASCADE_PID
  // Load cascade pitch angle loop gains
  cascade_pitch_angle_kp = prefs.getFloat("cpa_kp", CASCADE_PITCH_ANGLE_KP);
  cascade_pitch_angle_ki = prefs.getFloat("cpa_ki", CASCADE_PITCH_ANGLE_KI);
  cascade_pitch_angle_kd = prefs.getFloat("cpa_kd", CASCADE_PITCH_ANGLE_KD);

  // Load cascade pitch rate loop gains
  cascade_pitch_rate_kp = prefs.getFloat("cpr_kp", CASCADE_PITCH_RATE_KP);
  cascade_pitch_rate_ki = prefs.getFloat("cpr_ki", CASCADE_PITCH_RATE_KI);
  cascade_pitch_rate_kd = prefs.getFloat("cpr_kd", CASCADE_PITCH_RATE_KD);

  // Load cascade roll angle loop gains
  cascade_roll_angle_kp = prefs.getFloat("cra_kp", CASCADE_ROLL_ANGLE_KP);
  cascade_roll_angle_ki = prefs.getFloat("cra_ki", CASCADE_ROLL_ANGLE_KI);
  cascade_roll_angle_kd = prefs.getFloat("cra_kd", CASCADE_ROLL_ANGLE_KD);

  // Load cascade roll rate loop gains
  cascade_roll_rate_kp = prefs.getFloat("crr_kp", CASCADE_ROLL_RATE_KP);
  cascade_roll_rate_ki = prefs.getFloat("crr_ki", CASCADE_ROLL_RATE_KI);
  cascade_roll_rate_kd = prefs.getFloat("crr_kd", CASCADE_ROLL_RATE_KD);
  #endif

  prefs.end();
}
