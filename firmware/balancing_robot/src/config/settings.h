#ifndef SETTINGS_H
#define SETTINGS_H

// ============================================================
// BALANCING ROBOT — USER SETTINGS
// ============================================================
// THE ONLY FILE YOU NEED TO EDIT. Two selectors below retarget
// the whole firmware; everything else is tuning values.
//   board_pins.h / imu_select.h → parts bins (do not edit)
//   axis_map.h                  → macro machinery (do not edit)
// ============================================================

// ===== HARDWARE SELECTORS =====
#define ACTIVE_BOARD 2  // 1=ESP32, 2=ESP32-C3, 3=ESP32-S3
#define ACTIVE_IMU   1  // 1=MPU6050 (I2C), 2=MPU6500 (SPI), 3=MPU6500 (I2C)

#include "board_pins.h"
#include "imu_select.h"

// ===== CONTROL MODE =====
#define ENABLE_CASCADE_PID 0  // 1 = cascade (angle + rate), 0 = single angle PID

// ===== PID GAINS (single source of truth — boot + reset agree) =====
#define DEFAULT_KP 8.0   // Proportional: response strength to angle error
#define DEFAULT_KI 0.2   // Integral: long-term drift correction
#define DEFAULT_KD 5.0   // Derivative: oscillation damping (gyro rate)

#define PID_MAX 4095           // Maximum PID output (motor speed limit)
#define PID_INTEGRAL_LIMIT 500  // Integral windup protection
#define PID_CONTROL_LOOP_HZ 1000  // Control loop frequency in Hz

// ===== CASCADE PID (only used if ENABLE_CASCADE_PID = 1) =====
#define CASCADE_MODE_ANGLE_CONTROL 1  // 1 = angle mode, 0 = direct rate mode
#define CASCADE_MAX_RATE_SETPOINT 90.0f
#define CASCADE_RATE_INTEGRAL_LIMIT 200
#define CASCADE_ANGLE_INTEGRAL_LIMIT 100

// ===== THROTTLE =====
#define MIN_THROTTLE 0.0f  // Minimum throttle when armed
#define THROTTLE_GATE_ENABLED 1
#define THROTTLE_GATE_MIN_INPUT -1885     // Stick must go below this to arm
#define THROTTLE_GATE_RELEASE_INPUT -1700  // Stick must rise above this to start

// ===== CONTROLLER INPUT CURVE (0 = linear, 1 = exponential) =====
#define CONTROLLER_INPUT_CURVE 1
#define CONTROLLER_EXPO 1.0f  // 0.0 = linear, 1.0 = strong expo

// ===== BALANCING ROBOT INPUT SAFETY LIMITS =====
#define ROBOT_MAX_PITCH_SETPOINT_DEG 15.0f  // Firmware hard cap; UI maxAngle (default 10) stays under this
#define ROBOT_PITCH_INPUT_DEADBAND_DEG 0.6f
#define ROBOT_YAW_INPUT_DEADBAND 0.10f
#define ROBOT_MAX_ACTUAL_TILT_DEG 5.0f
#define ROBOT_TILT_GUARD_BAND_DEG 1.0f
#define ROBOT_SETPOINT_SLEW_DEG_PER_S 40.0f  // Max setpoint change rate

// ===== ESP-NOW JOYSTICK DEADZONES (raw ADC, stick ±2048) =====
#define ESPNOW_PITCH_DEADZONE 200
#define ESPNOW_ROLL_DEADZONE 200
#define ESPNOW_YAW_DEADZONE 1000

// ===== JOYSTICK YAW TOGGLE (1 = yaw stick active, 0 = forced zero) =====
#define JOYSTICK_YAW_INPUT_ENABLED 1

// ===== CONTROLLER INPUT SIGN (-1.0f reverses an axis) =====
#define CONTROLLER_PITCH_SIGN 1.0f
#define CONTROLLER_ROLL_SIGN 1.0f  // NOT USED: roll disabled
#define CONTROLLER_YAW_SIGN -1.0f

// ===== AXIS INVERSION (0 = normal, 1 = invert; consumed by axis_map.h) =====
#define ROLL_AXIS_INVERT 0   // NOT USED: roll disabled
#define PITCH_AXIS_INVERT 0
#define YAW_AXIS_INVERT 0    // Set to 1 to reverse yaw joystick input
#define GYRO_X_INVERT 0
#define GYRO_Y_INVERT 0
#define GYRO_Z_INVERT 0

// ===== FAILSAFE + CALIBRATION =====
#define FAILSAFE_ANGLE 60.0   // Auto-disarm if tilt exceeds this (deg)
#define CALIBRATION_SAMPLES 200  // Samples per calibration step

// ===== LOOP TIMING =====
#define LOOP_INTERVAL 1   // ms (1 = 1000Hz, 2 = 500Hz, 10 = 100Hz)
#define PID_UPDATE_PAUSE 5000  // ms to pause telemetry after a PID change

// ===== SERIAL =====
#define SERIAL_BAUD 115200  // Must match your serial monitor setting

// ===== WIFI (STA + AP + OTA) — sole WiFi config, consumed by comms/wifi_ota.h =====
// NOTE: real credentials live only in the local working copy, never in git.
#define WIFI_SSID "YOUR_WIFI_SSID"          // Change to your WiFi network name
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"  // Change to your WiFi password
#define AP_SSID "ESP32_QUAD"           // Access Point name when in AP mode
#define AP_PASSWORD "12345678"         // Access Point password

// ===== FILTER PARAMETERS =====
#define GYRO_LPF_ALPHA 1.0f       // Gyroscope software low-pass
#define ACCEL_LPF_ALPHA 0.1f      // Accelerometer software low-pass
#define BATTERY_LPF_ALPHA 0.15f   // Battery voltage low-pass
#define TIME_ALPHA 0.1            // Timing measurement smoothing
#define FAILSAFE_LPF_ALPHA 0.2    // Failsafe angle detection filter
#define SAFE_ANGLE_THRESHOLD 40.0f       // Arm when angle below this (deg)
#define SAFE_ANGLE_HYSTERESIS_CHECKS 10  // Consecutive safe readings to arm

// ===== DEBUG & MONITORING =====
#define ENABLE_DEBUG_PRINTS 1  // 1 = debug output, 0 = saves CPU/flash
#define TELEMETRY_UPDATE_RATE 100  // Telemetry print interval in ms

// ===== CONTROL PROTOCOL (only ONE main protocol: serial XOR websocket XOR esp-now) =====
#define ENABLE_SERIAL_COMMANDS_AND_STATUS 1  // Serial CLI + status
#define ENABLE_WEBSOCKET_CONTROL 1           // Web UI (conflicts with ESP-NOW)
#define ENABLE_ESPNOW 0                      // Wireless controller (works with serial)

// ===== ESP-NOW =====
#define ESPNOW_UPDATE_RATE 20    // Update interval in ms (50 Hz)
#define ESPNOW_TIMEOUT 2000      // Connection timeout in ms (failsafe)
#define ESPNOW_RSSI_TIMEOUT 2000  // RSSI ACK timeout in ms

// Controller MAC address (your transmitter)
#define CONTROLLER_MAC_0 0xAA
#define CONTROLLER_MAC_1 0xBB
#define CONTROLLER_MAC_2 0xCC
#define CONTROLLER_MAC_3 0xDD
#define CONTROLLER_MAC_4 0xEE
#define CONTROLLER_MAC_5 0xFF

// ESP-NOW setpoint limits (WS pitch shares ESPNOW_MAX_PITCH parse cap)
#define ESPNOW_MAX_PITCH 15.0f
#define ESPNOW_MAX_ROLL 12.0f
#define ESPNOW_MAX_YAW_RATE 100.0f
#define ESPNOW_THROTTLE_MAX 100.0f
#define ESPNOW_THROTTLE_MIN 0.0f

// ESP-NOW throttle shaping (piecewise curve around stick center)
#define ESPNOW_THROTTLE_MID_PERCENT 30.0f
#define ESPNOW_THROTTLE_LOW_EXPO 1.8f   // >1.0 = gentler near low stick
#define ESPNOW_THROTTLE_HIGH_EXPO 0.7f  // <1.0 = stronger above center

// ===== IMU RANGES =====
#define ANGLE_WRAP_ENABLED 1  // Wrap angles to ±180°
#define ACCEL_RANGE_G 2       // ±2G, ±4G, ±8G, or ±16G
#define AZ_SIGN 1  // 1 = descend positive, 0 = inverted

// ===== MOTOR BALANCE TRIM (0.8–1.2 if one motor is stronger) =====
#define MOTOR_LEFT_SPEED_MULTIPLIER 1.0
#define MOTOR_RIGHT_SPEED_MULTIPLIER 1.0

// ===== STARTUP BEHAVIOR =====
#define AUTO_LOAD_PID 1               // Auto-load PID from NVS
#define AUTO_LOAD_CALIBRATION 0       // Auto-load calibration from NVS
#define PRINT_CALIBRATION_ON_STARTUP 1  // Display loaded calibration values

// Axis-mapping machinery derives from the INVERT flags above.
#include "axis_map.h"

#endif  // SETTINGS_H
