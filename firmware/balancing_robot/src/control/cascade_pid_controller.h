#ifndef CASCADE_PID_CONTROLLER_H
#define CASCADE_PID_CONTROLLER_H

#include "../config/settings.h"

/*
 * ===== CASCADE PID CONTROLLER =====
 *
 * Two-loop cascade control:
 *  OUTER LOOP: Angle error → desired rate setpoint (slower, smoother)
 *  INNER LOOP: Gyro rate error → motor output (faster, responsive)
 *
 * Cascade PID controller for balancing robot pitch and roll axes.
 * Motor mixing is handled in motor_control.h for left/right motor output
 */

// ===== UNIFIED CASCADE PID GAINS =====
#ifndef CASCADE_PITCH_ANGLE_KP
#define CASCADE_PITCH_ANGLE_KP 0.0
#endif
#ifndef CASCADE_PITCH_ANGLE_KI
#define CASCADE_PITCH_ANGLE_KI 0.0
#endif
#ifndef CASCADE_PITCH_ANGLE_KD
#define CASCADE_PITCH_ANGLE_KD 0.0
#endif
#ifndef CASCADE_PITCH_RATE_KP
#define CASCADE_PITCH_RATE_KP 0.0  // Inner loop response: 50->120 for rate mode responsiveness
#endif
#ifndef CASCADE_PITCH_RATE_KI
#define CASCADE_PITCH_RATE_KI 0.0
#endif
#ifndef CASCADE_PITCH_RATE_KD
#define CASCADE_PITCH_RATE_KD 0.0
#endif

#ifndef CASCADE_ROLL_ANGLE_KP
#define CASCADE_ROLL_ANGLE_KP 0.0
#endif
#ifndef CASCADE_ROLL_ANGLE_KI
#define CASCADE_ROLL_ANGLE_KI 0.0
#endif
#ifndef CASCADE_ROLL_ANGLE_KD
#define CASCADE_ROLL_ANGLE_KD 0.0
#endif
#ifndef CASCADE_ROLL_RATE_KP
#define CASCADE_ROLL_RATE_KP 0.0
#endif
#ifndef CASCADE_ROLL_RATE_KI
#define CASCADE_ROLL_RATE_KI 0.0
#endif
#ifndef CASCADE_ROLL_RATE_KD
#define CASCADE_ROLL_RATE_KD 0.0
#endif

// Yaw control uses single PID parameters (KP_Yaw, KI_Yaw, KD_Yaw from NVS)

// From balancing_robot.ino - sensor data
extern float pitch, roll, yaw;
extern float pitch_final, roll_final;
extern float dt;
extern int16_t gyroX, gyroY, gyroZ;
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float throttle;

// From joystick input
extern float pitch_setpoint;
extern float roll_setpoint;
extern float yaw_setpoint;
extern float pitch_rate_target;  // Rate mode: target pitch rate (deg/s)
extern float roll_rate_target;   // Rate mode: target roll rate (deg/s)
extern float yaw_rate_target;    // Rate mode: target yaw rate (deg/s)

// Motor output - Balancing robot uses pidOutput_Pitch and pidOutput_Yaw
extern float pidOutput_Pitch;
extern float pidOutput_Roll;
extern float pidOutput_Yaw;

// ===== CASCADE PID GAINS (TUNABLE) - shared with prefs save/load =====
// Angle (Outer) Loop - Pitch
extern float cascade_pitch_angle_kp;
extern float cascade_pitch_angle_ki;
extern float cascade_pitch_angle_kd;
// Rate (Inner) Loop - Pitch
extern float cascade_pitch_rate_kp;
extern float cascade_pitch_rate_ki;
extern float cascade_pitch_rate_kd;
// Angle (Outer) Loop - Roll
extern float cascade_roll_angle_kp;
extern float cascade_roll_angle_ki;
extern float cascade_roll_angle_kd;
// Rate (Inner) Loop - Roll
extern float cascade_roll_rate_kp;
extern float cascade_roll_rate_ki;
extern float cascade_roll_rate_kd;

// ===== CASCADE PID API (see cascade_pid_controller.cpp) =====
float getGyroPitchRate_DPS();
float getGyroRollRate_DPS();
void updateCascadePID_Pitch();
void updateCascadePID_Roll();
void updateCascadePID();
void resetCascadePID();
void resetCascadePIDToDefaults();

#endif
