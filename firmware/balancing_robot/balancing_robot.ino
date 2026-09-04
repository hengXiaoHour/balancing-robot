// Balancing Robot with Multiple Filtering Methods for Control Theory Study
// ESP32 + L298N mosfet Motor Driver + MPU6050
// Supports: Kalman, Mahony, Madgwick, Complementary Filters

#include <Wire.h>
#include <Preferences.h>
#include "src/config/config.h"  // Load all configuration parameters

// Preference storage
Preferences prefs;

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
#include "src/system/prefs.h"  // PID save/load/reset (NVS)
#include "src/system/battery.h"  // Battery voltage + monitoring
#include "src/system/imu.h"  // IMU object + init
#include "src/system/control_task.h"  // 1kHz Core-0 control loop

#include "src/sensors/battery_state.h"  // BatteryState enum + LOW_VOLTAGE_THRESHOLD

// ESP-NOW throttle gating variables (declared before includes so esp_now_handler.h can use them)
bool throttle_gate_ready = false;  // Throttle gating for ESP-NOW
float last_throttle = 0.0f;        // Previous throttle value for ESP-NOW gating

#include "src/comms/serial_commands.h"
#include "src/comms/wifi_ota.h"  // WiFi + OTA support
#include "src/comms/websocket_handler.h"  // WebSocket server and communication
#include "src/comms/esp_now_handler.h"  // ESP-NOW controller communication

// External variables from ESP-NOW handler
extern volatile bool espnow_connected;

// Calibration state + bias values: defined in sensors/calibration.cpp

// ===== GLOBAL VARIABLE DECLARATIONS (Core Control Variables) =====

// Global Variables
unsigned long lastLoopTime = 0;
bool filterInitialized = false;
bool motorsArmed = false;
bool motorsActive = false;
bool statusMonitoring = false;
bool debugMonitoring = false;
float pitch_setpoint = 0.0;
float roll_setpoint = 0.0;
float yaw_setpoint = 0.0;

// Sensor variables
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Raw MPU data
int16_t mpu_accelX, mpu_accelY, mpu_accelZ;
int16_t mpu_gyroX, mpu_gyroY, mpu_gyroZ;

// Attitude filter variables
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float pitch_final = 0.0, roll_final = 0.0;
float pitch_bias = 0.0, roll_bias = 0.0, yaw_bias = 0.0;
float trim_pitch = 0.0;
float trim_roll = 0.0;
float P[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1}
};
float dt = 0.02;

// PID state + per-axis gains + velocity estimation: defined in control/pid_controller.cpp
// (pidLastError was dead — deleted, no references anywhere)

// Motor scaling
float motorScale_Left = 1.0f;
float motorScale_Right = 1.0f;
float throttle = 25.0f;
float throttle_increment = 5.0f;
float throttle_input_normalized = 0.0f;

// Motor test mode (stubs — test mode is disabled in BALANCING_ROBOT mode)
bool testMotorActive = false;

// Filter variables
float filtered_accelX = 0.0, filtered_accelY = 0.0, filtered_accelZ = 0.0;
float filtered_gyroX = 0.0, filtered_gyroY = 0.0, filtered_gyroZ = 0.0;

// Battery voltage monitoring
float battery_voltage = 0.0f;
float battery_voltage_filtered = 0.0f;
float battery_samples[BATTERY_SAMPLE_SIZE] = {0};
int battery_sample_index = 0;
bool batteryFilterPrimed = false;
float lastValidBatteryRawVoltage = 0.0f;

BatteryState batteryState = BATTERY_NORMAL;
unsigned long lastLEDBlink = 0;
bool ledState = false;

// Motor PIDs
float pidOutput_Left = 0.0;
float pidOutput_Right = 0.0;

// Telemetry timestamp: defined in comms/serial_pid_commands.cpp

// Failsafe filtering
float filtered_pitch = 0.0;
float filtered_roll = 0.0;

// Arm hysteresis
bool safeToArm = false;
uint16_t safeAngleCounter = 0;

// Dual-core task variables
SemaphoreHandle_t dataLock = NULL;
volatile bool controlLoopRunning = false;
volatile unsigned long loopTimeMs = 0;

// Computational complexity tracking
volatile unsigned long filterExecutionTime = 0;
volatile uint32_t freeHeapMemory = 0;
volatile uint32_t minFreeHeap = 999999;
float avgFilterTime = 0.0;

// Barometer stubs: baro_altitude_scale + accel_z_bias_cal_mps2 defined in
// sensors/calibration.cpp (NVS-loaded); accel_z_world_mps2 moves in a later step.

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
  
  // ===== I2C BUS SCAN =====
  if (debugMonitoring) {
    Serial.println("\n[DEBUG] Scanning I2C bus...");
    Serial.printf("[DEBUG] I2C pins - SDA: GPIO %d, SCL: GPIO %d\n", I2C_SDA, I2C_SCL);
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ ) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        Serial.printf("[DEBUG] I2C device found at address 0x%02X", address);
        if (address == 0x68) Serial.print(" (IMU/I2C device)");
        Serial.println();
        nDevices++;
      } else if (error == 4) {
        Serial.printf("[DEBUG] Unknown error at address 0x%02X\n", address);
      }
    }
    if (nDevices == 0) {
      Serial.println("[DEBUG] No I2C devices found!");
    } else {
      Serial.printf("[DEBUG] Found %d I2C device(s)\n", nDevices);
    }
    Serial.println("[DEBUG] I2C scan complete\n");
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
  
  lastLoopTime = millis();
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

  
  // Periodic MPU6050 initialization retry (every 30 seconds if not initialized)
  static unsigned long lastMPURetry = 0;
  if (!mpuInitialized && (millis() - lastMPURetry > 30000)) {
    lastMPURetry = millis();
    retryMPUInitialization();
  }
  
  // Small delay to prevent Core 1 from starving other tasks
  delay(1);
}
