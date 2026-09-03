#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <Arduino.h>
#include <Preferences.h>
#include "../utils/i2c_scan.h"
#include "../sensors/battery_state.h"
#include "../sensors/calibration.h"  // calibrationState, CALIB_IDLE, resetCalibrationToDefaults()

// Forward declarations (defined in balancing_robot.ino)
void retryMPUInitialization();
void savePIDToPreferences();
void resetPIDToDefaults();
void loadPIDFromPreferences();
void toggleMotorTest();

// ===== External Variables =====
extern bool motorsArmed;
extern bool motorsActive;
extern bool statusMonitoring;
extern bool debugMonitoring;
extern float pitch, roll, yaw;  // 3D angles
extern float filtered_pitch;    // Low-pass filtered pitch for failsafe check
extern float filtered_roll;     // Low-pass filtered roll for failsafe check
extern float pitch_setpoint;    // Pitch setpoint (angle target)
extern float roll_setpoint;     // Roll setpoint (angle target)
extern float yaw_setpoint;      // Yaw setpoint (angle target)
extern float pidOutput;
extern float pidOutput_Left;
extern float pidOutput_Right;
extern float pidIntegral;
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY;
extern unsigned long lastLoopTime;
extern unsigned long lastPIDUpdateTime;  // For pausing telemetry on PID updates
extern float KP, KI, KD;  // PID gains
extern float KP_Pitch, KI_Pitch, KD_Pitch;  // Pitch axis gains
extern float KP_Roll, KI_Roll, KD_Roll;    // Roll axis gains
extern float KP_Yaw, KI_Yaw, KD_Yaw;      // Yaw axis gains
extern float yaw_rate_target;              // Yaw rate setpoint
extern bool mpuInitialized;                 // MPU6050 initialization status
extern float throttle;  // Throttle setpoint
extern float throttle_increment;  // Throttle increment per command
extern float trim_pitch;  // Pitch sensor trim bias
extern float trim_roll;   // Roll sensor trim bias
extern float battery_voltage;  // Battery voltage in volts
extern BatteryState batteryState;  // Battery monitoring state
extern bool useAPMode;  // WiFi mode: false = STA, true = AP
extern bool safeToArm;  // ARM HYSTERESIS: Drone is level enough to arm
extern bool filterInitialized;  // Filter warm-up complete
extern bool testMotorActive;
extern void switchWiFiMode();  // Function to switch WiFi modes
extern void printWiFiStatus();  // Function to print WiFi status

// ===== CASCADE PID EXTERNAL VARIABLES =====
#if ENABLE_CASCADE_PID
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
extern float cascade_rate_setpoint_pitch;
extern float cascade_rate_setpoint_roll;
extern float cascade_pitch_output;
extern float cascade_roll_output;
void resetCascadePIDToDefaults();
void resetCascadePID();
#endif

// ===== Serial Commands API (see serial_commands.cpp) =====
void printWelcomeBanner();
void handleSerialCommand();

#endif
