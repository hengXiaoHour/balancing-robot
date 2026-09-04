// Balancing Robot with Multiple Filtering Methods for Control Theory Study
// ESP32 + L298N mosfet Motor Driver + MPU6050
// Supports: Kalman, Mahony, Madgwick, Complementary Filters

#include <Wire.h>
#include <Preferences.h>
#include "src/config/config.h"  // Load all configuration parameters

// Preference storage
Preferences prefs;

// PID Controller Variables (loaded from preferences)
float KP = DEFAULT_KP;
float KI = DEFAULT_KI;
float KD = DEFAULT_KD;

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

// Calibration state
CalibrationState calibrationState = CALIB_IDLE;

// ===== GLOBAL VARIABLE DECLARATIONS (Core Control Variables) =====

// Calibration bias values from preferences
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float axBias = 0.0f;
float axScale = 1.0f;
float ayBias = 0.0f;
float ayScale = 1.0f;
float azBias = 0.0f;
float azScale = 1.0f;

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

// PID variables
float pidError = 0.0;
float pidIntegral = 0.0;
float pidLastError = 0.0;
float pidOutput = 0.0;

// DUAL PID - PITCH
float pidError_Pitch = 0.0;
float pidIntegral_Pitch = 0.0;
float pidOutput_Pitch = 0.0;
float KP_Pitch = DEFAULT_KP;
float KI_Pitch = DEFAULT_KI;
float KD_Pitch = DEFAULT_KD;

// DUAL PID - ROLL
float pidError_Roll = 0.0;
float pidIntegral_Roll = 0.0;
float pidOutput_Roll = 0.0;
float KP_Roll = DEFAULT_KP;
float KI_Roll = DEFAULT_KI;
float KD_Roll = DEFAULT_KD;

// YAW AXIS
float pidIntegral_Yaw = 0.0;
float pidOutput_Yaw = 0.0;
float KP_Yaw = DEFAULT_KP;
float KI_Yaw = DEFAULT_KI;
float KD_Yaw = DEFAULT_KD;

// Cascaded control
float vel_x = 0.0f;
float vel_y = 0.0f;
float speed_x_setpoint = 0.0f;
float speed_y_setpoint = 0.0f;
float speedKp = 0.1f, speedKi = 0.01f, speedKd = 0.0f;
float speedIntegral_x = 0.0f, speedIntegral_y = 0.0f;
float speedDeadband = 0.05f;
float maxAngleFromSpeed = 5.0f;
float accel_x_filtered = 0.0f, accel_y_filtered = 0.0f;
float accelAlpha = 0.1f;
float velDecay = 0.9f;
float yaw_rate_target = 0.0f;
float pitch_rate_target = 0.0f;
float roll_rate_target = 0.0f;
float LOW_THROTTLE_THRESHOLD = 15.0f;

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

// Telemetry
unsigned long lastPIDUpdateTime = 0;

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

// Barometer stubs (disabled but variables retained for NVS compatibility)
float baro_altitude_scale = 1.0f;
float accel_z_bias_cal_mps2 = 0.0f;
float accel_z_world_mps2 = 0.0f;
float vertical_velocity_mps = 0.0f;
float altitude_est_m = 0.0f;
bool altitudeHoldEnabled = false;

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
  
  // Load calibration bias from preferences
  prefs.begin("mpu6050", true);  // Read-only mode
  gyroBiasY = prefs.getFloat("gyroBiasY", prefs.getFloat("gyroBias", 0.0f));
  gyroBiasX = prefs.getFloat("gyroBiasX", 0.0f);
  gyroBiasZ = prefs.getFloat("gyroBiasZ", 0.0f);
  axBias = prefs.getFloat("axBias", 0.0f);
  axScale = prefs.getFloat("axScale", 1.0f);
  ayBias = prefs.getFloat("ayBias", 0.0f);
  ayScale = prefs.getFloat("ayScale", 1.0f);
  azBias = prefs.getFloat("azBias", 0.0f);
  azScale = prefs.getFloat("azScale", 1.0f);
  baro_altitude_scale = prefs.getFloat("baroScale", 1.0f);
  accel_z_bias_cal_mps2 = prefs.getFloat("altAzBias", 0.0f);
  trim_pitch = prefs.getFloat("trim_pitch", 0.0f);
  trim_roll = prefs.getFloat("trim_roll", 0.0f);
  prefs.end();
  
  Serial.println("[OK] Calibration bias loaded from preferences");
  Serial.println("[BARO_CAL] Loaded offsets from NVS:");
  Serial.print("[BARO_CAL] baroScale(avg) = "); Serial.println(baro_altitude_scale, 5);
  Serial.print("[BARO_CAL] accelZBias_mps2(avg) = "); Serial.println(accel_z_bias_cal_mps2, 5);
  
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
  
  // Print all loaded bias values
  Serial.println("\n===== Loaded Calibration Bias =====");
  Serial.print("Gyro Bias X: "); Serial.println(gyroBiasX);
  Serial.print("Gyro Bias Y: "); Serial.println(gyroBiasY);
  Serial.print("Gyro Bias Z: "); Serial.println(gyroBiasZ);
  Serial.print("Accel X Bias: "); Serial.println(axBias);
  Serial.print("Accel X Scale: "); Serial.println(axScale);
  Serial.print("Accel Y Bias: "); Serial.println(ayBias);
  Serial.print("Accel Y Scale: "); Serial.println(ayScale);
  Serial.print("Accel Z Bias: "); Serial.println(azBias);
  Serial.print("Accel Z Scale: "); Serial.println(azScale);
  Serial.print("Baro Scale: "); Serial.println(baro_altitude_scale, 5);
  Serial.print("Alt Z-Accel Bias (m/s^2): "); Serial.println(accel_z_bias_cal_mps2, 5);
  Serial.println("\n===== Trimmed Angle Bias =====");
  Serial.print("Trim Pitch: "); Serial.print(trim_pitch, 4); Serial.println("Ã‚Â°");
  Serial.print("Trim Roll: "); Serial.print(trim_roll, 4); Serial.println("Ã‚Â°");
  Serial.println("===================================\n");
  
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
  
  // TELEMETRY
  static unsigned long lastTelemetry = 0;
  if (statusMonitoring && (millis() - lastTelemetry >= TELEMETRY_UPDATE_RATE)) {
    lastTelemetry = millis();
    
    float loopRate = 1000000.0f / (loopTimeUs > 0 ? loopTimeUs : 1);
    
    Serial.print("[LOOP] ");
    Serial.print("[Filter: "); Serial.print(getActiveFilterName()); Serial.print("] | ");
    Serial.print("Mode: BALANCING_ROBOT | ");
    Serial.print("Status: "); Serial.print(motorsArmed ? "ARMED" : "DISARMED"); Serial.print(" | ");
    Serial.print("Rate: "); Serial.print(loopRate, 2); Serial.print("Hz | ");
    Serial.print("Pitch: "); Serial.print(PITCH_ANGLE_FINAL_USED, 2); Serial.print("deg | ");
    Serial.print("Roll: "); Serial.print(ROLL_ANGLE_FINAL_USED, 2); Serial.print("deg | ");
    Serial.print("Yaw: "); Serial.print(yaw, 2); Serial.print("deg | ");
    Serial.print("PID[P,R]: "); Serial.print(pidOutput_Pitch, 0); Serial.print(",");
    Serial.print(pidOutput_Roll, 0); Serial.print(" | ");
    Serial.print("Motors[L,R]: "); Serial.print(pidOutput_Left, 0); Serial.print(",");
    Serial.print(pidOutput_Right, 0); Serial.print(" | ");
    Serial.print("Vbat: "); Serial.print(battery_voltage, 2); Serial.print("V | ");
    Serial.print("Az: "); Serial.print(accel_z_world_mps2, 2); Serial.print("m/s2 | ");
    Serial.print("[COMP] Filter: "); Serial.print(filterExecutionTime); Serial.print("us (avg: ");
    Serial.print(avgFilterTime, 1); Serial.print("us) | Heap: "); Serial.print(freeHeapMemory);
    Serial.print("B (min: "); Serial.print(minFreeHeap); Serial.print("B) | Load: ");
    float cpuLoad = (avgFilterTime / 10000.0) * 100.0;
    Serial.print(cpuLoad, 1); Serial.println("%");
  }
  
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
