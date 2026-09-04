#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// BALANCING ROBOT CONFIGURATION FILE
// ============================================================
// Modify these values to tune your robot without recompiling
// All important parameters are centralized here
// ============================================================

// ===== HARDWARE/SENSOR SELECTORS =====
// Board and sensor choices live in dedicated files:
//   board.h  -> board model + pin mapping
//   sensor.h -> IMU selection
#include "board.h"
#include "sensor.h"

// ===== VEHICLE MODE: BALANCING ROBOT ONLY =====

// ===== PID CONTROLLER CONFIGURATION =====
// SELECT CONTROL MODE: Uncomment ONE of the following
#define ENABLE_CASCADE_PID 0  // Cascade (outer angle + inner rate) PID

// Default PID Tuning Values (used in SINGLE angle PID mode)
// Single source of truth — axis defaults formerly duplicated in
// sensors/calibration.h (8.0/0.2/5.0 won at boot; config's 200/1/20 only
// leaked into prefs reset). Unified here so boot + reset agree.
#define DEFAULT_KP 8.0   // Proportional gain (main correction)
                          // Higher = stronger response to angle error
#define DEFAULT_KI 0.2    // Integral gain (steady-state error correction)
                          // Controls long-term drift correction
#define DEFAULT_KD 5.0    // Derivative gain (damping, based on gyro rate)
                          // Controls oscillation dampening
                          // Set to 0 to disable, increase slowly if needed

#define PID_MAX 4095      // Maximum PID output (motor speed limit)
#define PID_INTEGRAL_LIMIT 500  // Integral windup protection limit
#define PID_CONTROL_LOOP_HZ 1000  // Control loop frequency in Hz

// ===== CASCADE PID CONFIGURATION (only used if ENABLE_CASCADE_PID = 1) =====
// SELECT CASCADE MODE: angle control (outer loop) or direct rate control
#define CASCADE_MODE_ANGLE_CONTROL 1  // 1 = Angle mode (outer loop + inner rate loop)
                                      // 0 = Rate mode (direct rate control, skip angle loop)

// MODE-SPECIFIC CONFIGS: Balancing robot only
// Cascade PID configuration moved to cascade_config_overrides.h
#define CASCADE_MAX_RATE_SETPOINT 90.0f
#define CASCADE_RATE_INTEGRAL_LIMIT 200
#define CASCADE_ANGLE_INTEGRAL_LIMIT 100

// Minimum throttle when armed (used by serial and WebSocket arm commands)
#define MIN_THROTTLE 0.0f

// ===== THROTTLE GATE (ARMED, MOTORS OFF UNTIL STICK LOW) =====
// Uses raw joystick throttle input range (-2048 to +2048)
#define THROTTLE_GATE_ENABLED 1
#define THROTTLE_GATE_MIN_INPUT -1885     // Stick must go below this to arm motor output
#define THROTTLE_GATE_RELEASE_INPUT -1700 // Stick must rise above this to start motors

// ===== CONTROLLER INPUT CURVE =====
// 0 = linear, 1 = exponential
#define CONTROLLER_INPUT_CURVE 1
// Exponential rate for roll/pitch/yaw inputs (0.0 = linear, 1.0 = strong expo)
#define CONTROLLER_EXPO 1.0f

// ===== BALANCING ROBOT INPUT SAFETY LIMITS =====
// Clamp commanded setpoint angles to prevent over-tilt collapse from joystick input.
// These limits are applied only in VEHICLE_MODE_BALANCING_ROBOT.
#define ROBOT_MAX_PITCH_SETPOINT_DEG 5.0f
#define ROBOT_PITCH_INPUT_DEADBAND_DEG 0.6f
#define ROBOT_YAW_INPUT_DEADBAND 0.10f
// Additional guard based on ACTUAL measured tilt (not joystick setpoint).
#define ROBOT_MAX_ACTUAL_TILT_DEG 5.0f
#define ROBOT_TILT_GUARD_BAND_DEG 1.0f
// Max rate of pitch setpoint change (deg/s) to prevent command-induced oscillation.
#define ROBOT_SETPOINT_SLEW_DEG_PER_S 40.0f

// ===== ESPNOW JOYSTICK INPUT DEADZONE =====
// Applied to raw joystick inputs (ADC values: -2048 to +2048)
// Prevents drift when stick is centered
#define ESPNOW_PITCH_DEADZONE 200      // ±200 raw ADC value deadzone for pitch
#define ESPNOW_ROLL_DEADZONE 200       // ±200 raw ADC value deadzone for roll
#define ESPNOW_YAW_DEADZONE 1000        // ±200 raw ADC value deadzone for yaw

// ===== JOYSTICK YAW INPUT TOGGLE =====
// 1 = yaw stick controls yaw target, 0 = yaw stick ignored (yaw target forced to zero)
#define JOYSTICK_YAW_INPUT_ENABLED 1    

// ===== CONTROLLER INPUT SIGN (AXIS REVERSAL) =====
// Set to -1.0f to reverse an axis, +1.0f for normal direction.
#define CONTROLLER_PITCH_SIGN 1.0f
#define CONTROLLER_ROLL_SIGN 1.0f  // NOT USED: Roll disabled in balancing robot
#define CONTROLLER_YAW_SIGN -1.0f

// ===== AXIS INVERSION (NOT USED: Roll disabled) =====
#define ROLL_AXIS_INVERT 0

// ===== CALIBRATION CONFIGURATION =====
// Failsafe Settings
#define FAILSAFE_ANGLE 60.0   // Auto-disarm if tilt exceeds ±45°
                              // Prevents robot from falling over

// Calibration Parameters
#define CALIBRATION_SAMPLES 200  // Samples per calibration step

// ===== CONTROL LOOP TIMING =====
#define LOOP_INTERVAL 1   // Control loop interval in milliseconds
                          // 1ms = 1000Hz, 2ms = 500Hz, 10ms = 100Hz

#define PID_UPDATE_PAUSE 5000  // Milliseconds to pause telemetry after PID update
                               // Allows user to observe PID change effects

// ===== SERIAL COMMUNICATION =====
#define SERIAL_BAUD 115200  // Serial monitor baud rate
                            // Must match your serial monitor setting

// ===== AXIS INVERSION =====
// 0 = normal, 1 = invert axis direction
#define PITCH_AXIS_INVERT 0
#define YAW_AXIS_INVERT 0  // Set to 1 to reverse yaw input from joystick

// Gyro axis inversion (raw gyro axes)
#define GYRO_X_INVERT 0
#define GYRO_Y_INVERT 0
#define GYRO_Z_INVERT 0

#if PITCH_AXIS_INVERT
    #define PITCH_AXIS_SIGN -1.0f
#else
    #define PITCH_AXIS_SIGN 1.0f
#endif

#if ROLL_AXIS_INVERT
    #define ROLL_AXIS_SIGN -1.0f
#else
    #define ROLL_AXIS_SIGN 1.0f
#endif

#if YAW_AXIS_INVERT
    #define YAW_AXIS_SIGN -1.0f
#else
    #define YAW_AXIS_SIGN 1.0f
#endif

#if GYRO_X_INVERT
    #define GYRO_X_SIGN -1.0f
#else
    #define GYRO_X_SIGN 1.0f
#endif

#if GYRO_Y_INVERT
    #define GYRO_Y_SIGN -1.0f
#else
    #define GYRO_Y_SIGN 1.0f
#endif

#if GYRO_Z_INVERT
    #define GYRO_Z_SIGN -1.0f
#else
    #define GYRO_Z_SIGN 1.0f
#endif

#define GYRO_X_USED (GYRO_X_SIGN * gyroX)
#define GYRO_Y_USED (GYRO_Y_SIGN * gyroY)
#define GYRO_Z_USED (GYRO_Z_SIGN * gyroZ)
#define GYRO_BIAS_X_USED (GYRO_X_SIGN * gyroBiasX)
#define GYRO_BIAS_Y_USED (GYRO_Y_SIGN * gyroBiasY)
#define GYRO_BIAS_Z_USED (GYRO_Z_SIGN * gyroBiasZ)

// Balancing robot axis mapping (pitch-only control, no roll)
#define PITCH_ANGLE_RAW pitch
#define PITCH_ANGLE_FINAL pitch_final
#define ROLL_ANGLE_RAW roll
#define ROLL_ANGLE_FINAL roll_final

#define GYRO_PITCH_RATE_RAW GYRO_Y_USED
#define GYRO_PITCH_BIAS_RAW GYRO_BIAS_Y_USED

#define PITCH_ANGLE_RAW_USED (PITCH_AXIS_SIGN * PITCH_ANGLE_RAW)
#define PITCH_ANGLE_FINAL_USED (PITCH_AXIS_SIGN * PITCH_ANGLE_FINAL)
#define GYRO_PITCH_RATE_USED (PITCH_AXIS_SIGN * GYRO_PITCH_RATE_RAW)
#define GYRO_PITCH_BIAS_USED (PITCH_AXIS_SIGN * GYRO_PITCH_BIAS_RAW)
#define GYRO_YAW_RATE_USED GYRO_Z_USED
#define GYRO_YAW_BIAS_USED GYRO_BIAS_Z_USED

// Roll axis stubs (balancing robot has no roll control - completely disabled)
#define GYRO_ROLL_RATE_USED 0.0f
#define GYRO_ROLL_BIAS_USED 0.0f
#define ROLL_ANGLE_RAW_USED 0.0f
#define ROLL_ANGLE_FINAL_USED 0.0f

// ===== FILTER & CONTROL PARAMETERS =====
#define GYRO_LPF_ALPHA 0.7f              // Low-pass filter for gyroscope data
#define ACCEL_LPF_ALPHA 0.7f             // Low-pass filter for accelerometer data
#define BATTERY_LPF_ALPHA 0.15f          // Low-pass filter for battery voltage
#define TIME_ALPHA 0.1                   // Low-pass filter for timing measurements
#define FAILSAFE_LPF_ALPHA 0.2           // Stronger filter for failsafe angle detection
#define SAFE_ANGLE_THRESHOLD 40.0f       // Arm when angle < 40° (5° hysteresis from 45° limit)
#define SAFE_ANGLE_HYSTERESIS_CHECKS 10  // Require 10 consecutive safe readings (200ms at 50Hz)

// ===== DEBUG & MONITORING =====
#define ENABLE_DEBUG_PRINTS 1  // Set to 1 to enable debug output
                               // Set to 0 to disable (saves memory/performance)

#define TELEMETRY_UPDATE_RATE 100  // Telemetry print interval in ms (100ms = 10Hz)
                                    // Higher = less frequent prints

// ===== CONTROL PROTOCOL SELECTION =====
// NOTE: Only ONE main protocol can be active (Serial OR WebSocket OR ESP-NOW)
//       Serial can work alongside ESP-NOW, but WebSocket cannot
// Set the desired protocol to 1, others to 0
#define ENABLE_SERIAL_COMMANDS_AND_STATUS 1  // Set to 1 to enable serial commands + status monitoring
                                             // Set to 0 to disable both (saves CPU, disables CLI)

#define ENABLE_WEBSOCKET_CONTROL 1           // Set to 1 to enable WebSocket server for Web UI
                                             // Set to 0 to disable (saves CPU, disables Web UI)
                                             // NOTE: Cannot work with ESP-NOW (causes conflicts)

#define ENABLE_ESPNOW 0                      // Set to 1 to enable ESP-NOW wireless controller
                                             // Set to 0 to disable (saves CPU, disables remote control)
                                             // NOTE: Can work alongside Serial monitoring

// ===== ESP-NOW CONFIGURATION =====
#define ESPNOW_UPDATE_RATE 20                // ESP-NOW update interval in ms (50 Hz)
#define ESPNOW_TIMEOUT 2000                  // Connection timeout in ms (no data = failsafe)
#define ESPNOW_RSSI_TIMEOUT 2000             // RSSI ACK timeout in ms

// Controller MAC addresses (ESP32-C3 transmitter)
// In this case : 1c:db:d4:17:2a:f8
#define CONTROLLER_MAC_0 0x1c
#define CONTROLLER_MAC_1 0xdb
#define CONTROLLER_MAC_2 0xd4
#define CONTROLLER_MAC_3 0x17 
#define CONTROLLER_MAC_4 0x2a
#define CONTROLLER_MAC_5 0xf8

// ===== ESP-NOW SETPOINT CONFIGURATION =====
#define ESPNOW_MAX_PITCH 12.0f               // Maximum pitch setpoint in degrees (±)
#define ESPNOW_MAX_ROLL 12.0f                // Maximum roll setpoint in degrees (±)
#define ESPNOW_MAX_YAW_RATE 100.0f           // Maximum yaw rate target in deg/s (±)
#define ESPNOW_THROTTLE_MAX 100.0f           // Maximum throttle percentage (0-100)
#define ESPNOW_THROTTLE_MIN 0.0f            // Minimum throttle when armed percentage

// ESP-NOW throttle shaping (piecewise curve)
// stick <= 0.0: maps to [THROTTLE_MIN .. THROTTLE_MID]
// stick >  0.0: maps to [THROTTLE_MID .. THROTTLE_MAX]
// This gives slow motor spin in lower stick region and stronger power above center.
#define ESPNOW_THROTTLE_MID_PERCENT 30.0f
#define ESPNOW_THROTTLE_LOW_EXPO 1.8f   // >1.0 = gentler near low stick
#define ESPNOW_THROTTLE_HIGH_EXPO 0.7f  // <1.0 = stronger response above center
// ===== ADVANCED CONFIGURATION =====
// Angle Wrapping
#define ANGLE_WRAP_ENABLED 1  // Enable angle wrapping to ±180°
                              // Prevents huge angle values for continuous rotation

// Gyro Sensitivity Conversion
#define GYRO_SENSITIVITY 16.4  // LSB/°/s for ±2000°/s range
                               // For ±500°/s: 65.5
                               // For ±250°/s: 131.0
                               // For ±2000°/s: 16.4

// Accelerometer Range & Sensitivity
#define ACCEL_RANGE_G 2  // Accelerometer range in G
                         // ±2G, ±4G, ±8G, or ±16G

// Accel Sensitivity Conversion (LSB/G)
#if ACCEL_RANGE_G == 2
  #define ACCEL_SENSITIVITY 16384.0f  // ±2G range
#elif ACCEL_RANGE_G == 4
  #define ACCEL_SENSITIVITY 8192.0f   // ±4G range
#elif ACCEL_RANGE_G == 8
  #define ACCEL_SENSITIVITY 4096.0f   // ±8G range
#elif ACCEL_RANGE_G == 16
  #define ACCEL_SENSITIVITY 2048.0f   // ±16G range
#else
  #define ACCEL_SENSITIVITY 16384.0f  // Default to ±2G
#endif

// Accelerometer Z-Axis Sign (controls world-Z acceleration sign)
#define AZ_SIGN 1  // 1 = descend/drop positive, lift/upward negative | 0 = inverted

// ===== MOTOR CALIBRATION OFFSET =====
// Fine-tune motor balance if one motor is stronger than the other
#define MOTOR_LEFT_SPEED_MULTIPLIER 1.0   // Adjust left motor speed (0.8-1.2)
#define MOTOR_RIGHT_SPEED_MULTIPLIER 1.0  // Adjust right motor speed (0.8-1.2)

// ===== SYSTEM BEHAVIOR =====
// Auto-load preferences on startup
#define AUTO_LOAD_PID 1              // Auto-load PID from preferences
#define AUTO_LOAD_CALIBRATION 0      // Auto-load calibration from preferences

// Print calibration on startup
#define PRINT_CALIBRATION_ON_STARTUP 1  // Display loaded calibration values

#endif
