// Balancing Robot with Multiple Filtering Methods for Control Theory Study
// ESP32 + L298N mosfet Motor Driver + MPU6050
// Supports: Kalman, Mahony, Madgwick, Complementary Filters

#include <Wire.h>
#include <Preferences.h>
#include "src/config/config.h"  // Load all configuration parameters

// Preference storage: defined in sensors/calibration.cpp (main NVS user)

// PID gains + state: defined in control/pid_controller.cpp

// Include header files AFTER config.h
#include "src/filters/filter_selector.h"        // Filter selection (choose ONE)
#include "src/control/motor_control.h"
#ifdef USE_KALMAN_FILTER
#include "src/filters/kalman_filter.h"
#endif
#ifdef USE_MAHONY_FILTER
#include "src/filters/mahony_filter.h"
#endif
#ifdef USE_MADGWICK_FILTER
#include "src/filters/madgwick_filter.h"
#endif
#ifdef USE_COMPLEMENTARY_FILTER
#include "src/filters/complementary_filter.h"
#endif
#ifdef USE_COMPLEMENTARY_QUATERNION_FILTER
#include "src/filters/complementary_quaternion_filter.h"
#endif
#ifdef USE_EKF_FILTER
#include "src/filters/ekf_filter.h"
#endif
#include "src/control/pid_controller.h"
#if ENABLE_CASCADE_PID
#include "src/control/cascade_pid_controller.h"  // Cascade PID with unified gains
#endif
#include "src/sensors/calibration.h"
#include "src/utils/timing.h"
#include "src/utils/i2c_scan.h"  // scanI2CBus()
#include "src/system/prefs.h"  // PID save/load/reset (NVS)
#include "src/system/battery.h"  // Battery voltage + monitoring
#include "src/system/imu.h"  // IMU object + init
#include "src/system/state.h"  // shared live state (attitude, sensors, setpoints)
#include "src/system/control_task.h"  // 1kHz Core-0 control loop

#include "src/sensors/battery_state.h"  // BatteryState enum + LOW_VOLTAGE_THRESHOLD

// ESP-NOW throttle gating: defined in comms/esp_now_handler.cpp

#include "src/comms/serial_commands.h"
#include "src/comms/wifi_ota.h"  // WiFi + OTA support
#include "src/comms/websocket_handler.h"  // WebSocket server and communication
#include "src/comms/esp_now_handler.h"  // ESP-NOW controller communication

// External variables from ESP-NOW handler
extern volatile bool espnow_connected;

// Calibration state + bias values: defined in sensors/calibration.cpp

// ===== GLOBAL VARIABLE DECLARATIONS (Core Control Variables) =====

// Global Variables: live state now defined in system/state.cpp
// (lastLoopTime was write-only — setup() wrote it, nobody read it — deleted)

// PID state + per-axis gains + velocity estimation: defined in control/pid_controller.cpp
// (pidLastError was dead — deleted, no references anywhere)

// Motor scaling + outputs + arm hysteresis: defined in control/motor_control.cpp

// Motor test mode: defined in control/motor_control.cpp

// Filter I/O: defined in system/state.cpp

// Battery voltage monitoring: defined in system/battery.cpp

// Motor PIDs: defined in control/motor_control.cpp

// Telemetry timestamp: defined in comms/serial_pid_commands.cpp

// Failsafe filtering: defined in system/state.cpp

// Arm hysteresis: defined in control/motor_control.cpp

// Dual-core task variables: defined in system/control_task.cpp
// (controlLoopRunning was dead — declared, never read — deleted)

// Computational complexity tracking: defined in system/control_task.cpp

// Barometer stubs: baro_altitude_scale + accel_z_bias_cal_mps2 defined in
// sensors/calibration.cpp (NVS-loaded); accel_z_world_mps2 in comms/serial_commands.cpp.

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("[MODE] Active vehicle mode: BALANCING_ROBOT");
  
  // Test LED functionality
  Serial.printf("[LED TEST] Testing LED on pin %d...\n", LOW_VOLTAGE_LED_PIN);
  pinMode(LOW_VOLTAGE_LED_PIN, OUTPUT);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);  // LED ON
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL); // LED OFF
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);  // LED ON again
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL); // LED OFF
  Serial.println("[LED TEST] LED test complete");
  
  // Configure motor outputs based on selected vehicle mode
  initVehicleMotors();
  
  // Configure ADC for battery voltage monitoring
  analogReadResolution(12);  // 12-bit ADC resolution (0-4095)
  
  // Initialize IMU (I2C bus + sensor, see src/system/imu.cpp)
  initIMU();

  // Barometer disabled - removed
  Serial.println("[INFO] Config settings loaded");
  
  // ===== I2C BUS SCAN (see utils/i2c_scan.cpp) =====
  if (debugMonitoring) {
    scanI2CBus();
  }
  
  // Load calibration bias from NVS (see sensors/calibration.cpp)
  loadCalibrationBias();
  
  // Load PID values from preferences
  loadPIDFromPreferences();
  
  #if ENABLE_CASCADE_PID
  Serial.println("[CASCADE_PID] Cascade gains loaded from NVS (or defaults if not saved)");
  #endif
  
  // Initialize WiFi + OTA (disabled when using ESP-NOW)
  #if !ENABLE_ESPNOW
  initWiFi();
  #endif
  
  // DO NOT init filter here - sensors not warmed up yet
  // Will init after control loop starts with stable accel data
  
  // Create mutex for shared data
  dataLock = xSemaphoreCreateMutex();
  
  // Create control loop task pinned to Core 0 (no interruptions)
  xTaskCreatePinnedToCore(
    controlLoopTask,       // Task function
    "ControlLoop",         // Task name
    4096,                  // Stack size (bytes)
    NULL,                  // Parameters
    2,                     // Priority (high)
    NULL,                  // Task handle
    0                      // Core 0 (dedicated for control)
  );
  
  Serial.println("[OK] Control loop task created on Core 0");
  Serial.println("[OK] Serial/telemetry running on Core 1\n");
  
  // Initialize WebSocket server
  #if ENABLE_WEBSOCKET_CONTROL
  initWebSocket();
  #endif
  
  // Initialize ESP-NOW controller communication
  #if ENABLE_ESPNOW
  initESPNOW();
  #endif
  
  // Print welcome message
  printWelcomeBanner();;
  printCalibrationMenu();
}

void loop() {
  // CORE 1: Handle all non-critical tasks (serial commands, telemetry, calibration UI)
  // Control loop runs uninterrupted on Core 0
  
  // TELEMETRY (rate-limited [LOOP] status line, see serial_commands.cpp)
  printTelemetryStatus();
  
  // Check for OTA updates (disabled when using ESP-NOW)
  #if !ENABLE_ESPNOW
  ArduinoOTA.handle();
  #endif
  
  // Handle WiFi connection in STA mode (disabled when using ESP-NOW)
  #if !ENABLE_ESPNOW
  handleWiFiConnection();
  #endif
  
  // Handle WebSocket server
  #if ENABLE_WEBSOCKET_CONTROL
  handleWebSocketLoop();
  #endif
  
  // Handle serial commands (don't block)
  #if ENABLE_SERIAL_COMMANDS_AND_STATUS
  handleSerialCommand();
  #endif

  updateMotorTest();

  // Barometer disabled - removed
  
  // Update ESP-NOW controller input
  #if ENABLE_ESPNOW
  updateESPNOWService();
  #endif
  
  // Update calibration if active (non-blocking on Core 1)
  if (calibrationState != CALIB_IDLE) {
    updateCalibration();
  }

  
  // Periodic MPU6050 initialization retry (every 30s if not initialized, see imu.cpp)
  pollMpuRetry();
  
  // Small delay to prevent Core 1 from starving other tasks
  delay(1);
}
