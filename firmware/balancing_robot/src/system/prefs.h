#ifndef PREFS_H
#define PREFS_H

#include <Arduino.h>
#include <Preferences.h>
#include "../config/settings.h"  // ENABLE_CASCADE_PID, CASCADE_*_KP/... defaults

// PID gains (definitions in control/pid_controller.cpp)
extern float KP_Pitch, KI_Pitch, KD_Pitch;
extern float KP_Roll, KI_Roll, KD_Roll;
extern float KP_Yaw, KI_Yaw, KD_Yaw;
extern float yaw_rate_target;
extern float pitch_rate_target;
extern float roll_rate_target;

#if ENABLE_CASCADE_PID
// Cascade gains (definitions in cascade_pid_controller.cpp)
extern float cascade_pitch_angle_kp;
extern float cascade_pitch_angle_ki;
extern float cascade_pitch_angle_kd;
extern float cascade_pitch_rate_kp;
extern float cascade_pitch_rate_ki;
extern float cascade_pitch_rate_kd;
extern float cascade_roll_angle_kp;
extern float cascade_roll_angle_ki;
extern float cascade_roll_angle_kd;
extern float cascade_roll_rate_kp;
extern float cascade_roll_rate_ki;
extern float cascade_roll_rate_kd;
#endif

// ===== PID prefs API (see prefs.cpp) =====
void savePIDToPreferences();
void resetPIDToDefaults();
void loadPIDFromPreferences();

#endif
