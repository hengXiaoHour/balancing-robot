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
#include "src/filters/kalman_filter.h"
#include "src/filters/mahony_filter.h"
#include "src/filters/madgwick_filter.h"
#include "src/filters/complementary_filter.h"
#include "src/filters/complementary_quaternion_filter.h"
#include "src/filters/ekf_filter.h"
#include "src/control/pid_controller.h"
#if ENABLE_CASCADE_PID
#include "src/control/cascade_pid_controller.h"  // Cascade PID with unified gains
#endif
#include "src/sensors/calibration.h"
#include "src/utils/timing.h"

// ===== Battery Low Voltage Monitoring =====
#define LOW_VOLTAGE_THRESHOLD 3.5f  // Voltage threshold for low battery detection
enum BatteryState {
  BATTERY_NORMAL,        // Normal operation
  BATTERY_LOW_CONFIRMED  // Low battery detected - LED blinking
};

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

// Global IMU object
IMU_Custom mpu;
// ===== GLOBAL VARIABLE DECLARATIONS (Core Control Variables) =====

// MPU6050 initialization status
bool mpuInitialized = false;

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

// Motor test mode
const float TEST_MOTOR_THROTTLE_PERCENT = 20.0f;
const unsigned long TEST_MOTOR_DURATION_MS = 2000;
bool testMotorActive = false;
int testMotorIndex = 0;
int testMotorLastPrinted = -1;
unsigned long testMotorStartMs = 0;

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

// ===== BATTERY VOLTAGE MANAGEMENT FUNCTIONS =====
void updateBatteryVoltage() {
  int adc_raw = analogRead(BATTERY_PIN);
  float raw_voltage = (adc_raw / (float)ADC_MAX) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;

  if (!batteryFilterPrimed) {
    for (int i = 0; i < BATTERY_SAMPLE_SIZE; i++) {
      battery_samples[i] = raw_voltage;
    }
    battery_voltage_filtered = raw_voltage;
    battery_voltage = raw_voltage;
    lastValidBatteryRawVoltage = raw_voltage;
    batteryFilterPrimed = true;
    return;
  }

  float raw_delta = raw_voltage - lastValidBatteryRawVoltage;
  float max_step_down = BATTERY_GLITCH_REJECT_V;
  float max_step_up = BATTERY_GLITCH_REJECT_V * 2.0f;
  if (raw_delta > max_step_up) {
    raw_voltage = lastValidBatteryRawVoltage + max_step_up;
  } else if (raw_delta < -max_step_down) {
    raw_voltage = lastValidBatteryRawVoltage - max_step_down;
  }
  lastValidBatteryRawVoltage = raw_voltage;
  
  battery_samples[battery_sample_index] = raw_voltage;
  battery_sample_index = (battery_sample_index + 1) % BATTERY_SAMPLE_SIZE;
  
  float voltage_sum = 0.0f;
  for (int i = 0; i < BATTERY_SAMPLE_SIZE; i++) {
    voltage_sum += battery_samples[i];
  }
  float voltage_average = voltage_sum / BATTERY_SAMPLE_SIZE;
  
  battery_voltage_filtered = (BATTERY_LPF_ALPHA * voltage_average) + ((1.0f - BATTERY_LPF_ALPHA) * battery_voltage_filtered);
  battery_voltage = battery_voltage_filtered;
  
  if (debugMonitoring) {
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 1000 && millis() < 10000) {
      Serial.print("[BATTERY DEBUG] Raw ADC: ");
      Serial.print(adc_raw);
      Serial.print(", Raw Voltage: ");
      Serial.print(raw_voltage, 3);
      Serial.print("V, Filtered: ");
      Serial.print(battery_voltage, 3);
      Serial.println("V");
      lastDebugTime = millis();
    }
  }
}

void updateBatteryMonitoring() {
  static unsigned long initStartTime = 0;
  if (initStartTime == 0) initStartTime = millis();
  
  if (battery_voltage < 3.0f || (millis() - initStartTime) < 2000) {
    digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL);
    return;
  }
  
  static int lowVoltageCount = 0;
  
  if (debugMonitoring) {
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 2000 && millis() < 30000) {
      Serial.print("[LED DEBUG] ESP-NOW Connected: ");
      Serial.print(espnow_connected ? "YES" : "NO");
      Serial.print(", Battery State: ");
      Serial.print(batteryState == BATTERY_NORMAL ? "NORMAL" : "LOW");
      Serial.print(", LED should be: ");
      if (batteryState == BATTERY_LOW_CONFIRMED) {
        Serial.println("BLINKING");
      } else if (espnow_connected) {
        Serial.println("ON (connected)");
      } else {
        Serial.println("OFF (disconnected)");
      }
      lastDebugTime = millis();
    }
  }
  
  switch (batteryState) {
    case BATTERY_NORMAL:
      pinMode(LOW_VOLTAGE_LED_PIN, OUTPUT);
      
      if (espnow_connected) {
        digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);
      } else {
        digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL);
      }
      
      if (battery_voltage <= LOW_VOLTAGE_THRESHOLD) {
        lowVoltageCount++;
        if (lowVoltageCount >= 5) {
          batteryState = BATTERY_LOW_CONFIRMED;
          Serial.println("[BATTERY] LOW BATTERY DETECTED! Voltage has been 3.3V or below for 100ms. LED will blink until battery changed.");
          pinMode(LOW_VOLTAGE_LED_PIN, OUTPUT);
          lowVoltageCount = 0;
        }
      } else {
        lowVoltageCount = 0;
      }
      break;
      
    case BATTERY_LOW_CONFIRMED:
      unsigned long now = millis();
      if (now - lastLEDBlink >= 500) {
        ledState = !ledState;
        digitalWrite(LOW_VOLTAGE_LED_PIN, ledState ? LED_ON_LEVEL : LED_OFF_LEVEL);
        lastLEDBlink = now;
      }
      break;
  }
}
// ===== BALANCING ROBOT MODE FUNCTIONS===="

inline void toggleMotorTest() {
  Serial.println("[MODE] Motor test command is disabled in BALANCING_ROBOT mode");
}

inline void updateMotorTest() {
}

inline void applyVehicleInputLimits() {
  #if ENABLE_CASCADE_PID && !CASCADE_MODE_ANGLE_CONTROL
  // In rate mode, don't modify pitch_setpoint - let pitch_rate_target handle it
  roll_setpoint = 0.0f;  // Roll always disabled for balancing robot
  return;
  #endif
  
  // ===== ANGLE MODE: Apply pitch setpoint limits and slew-rate limiting =====
  float measured_pitch = PITCH_ANGLE_FINAL_USED;
  float raw_setpoint = constrain(pitch_setpoint, -ROBOT_MAX_PITCH_SETPOINT_DEG, ROBOT_MAX_PITCH_SETPOINT_DEG);

  roll_setpoint = 0.0f;

  // Soft guard: only block outward commands near/over tilt boundary.
  // Do NOT inject opposite recovery commands (can cause oscillation).
  float guarded_setpoint = raw_setpoint;
  float guard_start_pos = ROBOT_MAX_ACTUAL_TILT_DEG - ROBOT_TILT_GUARD_BAND_DEG;
  float guard_start_neg = -ROBOT_MAX_ACTUAL_TILT_DEG + ROBOT_TILT_GUARD_BAND_DEG;

  if (measured_pitch >= ROBOT_MAX_ACTUAL_TILT_DEG && guarded_setpoint > 0.0f) {
    guarded_setpoint = 0.0f;
  } else if (measured_pitch <= -ROBOT_MAX_ACTUAL_TILT_DEG && guarded_setpoint < 0.0f) {
    guarded_setpoint = 0.0f;
  } else {
    // Linearly attenuate outward command inside guard band.
    if (measured_pitch > guard_start_pos && guarded_setpoint > 0.0f) {
      float t = (ROBOT_MAX_ACTUAL_TILT_DEG - measured_pitch) / max(ROBOT_TILT_GUARD_BAND_DEG, 0.001f);
      t = constrain(t, 0.0f, 1.0f);
      guarded_setpoint *= t;
    } else if (measured_pitch < guard_start_neg && guarded_setpoint < 0.0f) {
      float t = (measured_pitch + ROBOT_MAX_ACTUAL_TILT_DEG) / max(ROBOT_TILT_GUARD_BAND_DEG, 0.001f);
      t = constrain(-t, 0.0f, 1.0f);
      guarded_setpoint *= t;
    }
  }

  // Slew-rate limit setpoint changes to avoid jerky PID excitation.
  static float limited_setpoint = 0.0f;
  float max_step = ROBOT_SETPOINT_SLEW_DEG_PER_S * dt;
  float delta = guarded_setpoint - limited_setpoint;
  delta = constrain(delta, -max_step, max_step);
  limited_setpoint += delta;

  pitch_setpoint = limited_setpoint;
}

inline void initVehicleMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  setLeftMotorSpeed(0.0f);
  setRightMotorSpeed(0.0f);
}

inline void updateVehicleMotorControl() {
  // Balancing Robot Motor Mixing:
  // - pidOutput_Pitch: Balance command (both motors equal)
  // - pidOutput_Yaw: Steering command (differential speed)
  // - pidOutputRoll is NOT used in balancing mode (no sideways tilt)
  // NOTE: Yaw uses ONLY the single PID controller (unified across all modes)
  //       Whether in CASCADE_PID or single PID mode, yaw control is identical
  
  #if ENABLE_CASCADE_PID
  // Cascade PID: pitch from cascade loop, yaw from single PID controller
  float balance_cmd = pidOutput_Pitch;  // From cascade pitch loop
  float turn_cmd = pidOutput_Yaw;       // From single PID yaw controller (KP_Yaw, KI_Yaw, KD_Yaw from NVS)
  #else
  // Single PID: both pitch and yaw from single PID controller
  float balance_cmd = pidOutput_Pitch;  // From single PID pitch controller
  float turn_cmd = pidOutput_Yaw;       // From single PID yaw controller (KP_Yaw, KI_Yaw, KD_Yaw from NVS)
  #endif
  
  // Motor mixing for 2-wheel differentials:
  // LEFT = Balance + Turn
  // RIGHT = Balance - Turn
  float left_cmd = balance_cmd + turn_cmd;
  float right_cmd = balance_cmd - turn_cmd;

  left_cmd = constrain(left_cmd, -PID_MAX, PID_MAX);
  right_cmd = constrain(right_cmd, -PID_MAX, PID_MAX);

  setLeftMotorSpeed(left_cmd);
  setRightMotorSpeed(right_cmd);
}

// ===== CORE 0: Control Loop (IMU + Kalman + PID) =====
void controlLoopTask(void *pvParameters) {
  unsigned long lastLoopTime = millis();
  unsigned long warmupStartTime = millis();
  const unsigned long WARMUP_TIME = 2000;  // 2 seconds for sensors to stabilize
  
  while (1) {
    // Maintain 1kHz control loop
    if (millis() - lastLoopTime >= LOOP_INTERVAL) {
      unsigned long loopStart = millis();
      dt = (loopStart - lastLoopTime) / 1000.0;
      lastLoopTime = loopStart;

      // Fast ESP-NOW path (1kHz) for joystick input
      #if ENABLE_ESPNOW
      updateESPNOWDataOnly();
      #endif

      // Barometer disabled - no updateBarometer() call
      
      // Read sensor data (only if MPU is initialized)
      if (mpuInitialized) {
        mpu.readAll();
        
        // Store raw sensor values
        mpu_accelX = mpu.accelX;
        mpu_accelY = mpu.accelY;
        mpu_accelZ = mpu.accelZ;
        mpu_gyroX = mpu.gyroX;
        mpu_gyroY = mpu.gyroY;
        mpu_gyroZ = mpu.gyroZ;
      } else {
        // Set default values if MPU not available
        mpu_accelX = mpu_accelY = mpu_accelZ = 0;
        mpu_gyroX = mpu_gyroY = mpu_gyroZ = 0;
      }
      
      // Apply low-pass filter to accelerometer data
      filtered_accelX = ACCEL_LPF_ALPHA * mpu_accelX + (1.0 - ACCEL_LPF_ALPHA) * filtered_accelX;
      filtered_accelY = ACCEL_LPF_ALPHA * mpu_accelY + (1.0 - ACCEL_LPF_ALPHA) * filtered_accelY;
      filtered_accelZ = ACCEL_LPF_ALPHA * mpu_accelZ + (1.0 - ACCEL_LPF_ALPHA) * filtered_accelZ;
      
      // Apply low-pass filter to gyroscope data
      filtered_gyroX = GYRO_LPF_ALPHA * mpu_gyroX + (1.0 - GYRO_LPF_ALPHA) * filtered_gyroX;
      filtered_gyroY = GYRO_LPF_ALPHA * mpu_gyroY + (1.0 - GYRO_LPF_ALPHA) * filtered_gyroY;
      filtered_gyroZ = GYRO_LPF_ALPHA * mpu_gyroZ + (1.0 - GYRO_LPF_ALPHA) * filtered_gyroZ;
      
      // Apply calibration bias to raw data
      accelX = (mpu_accelX - axBias) * axScale;
      accelY = (mpu_accelY - ayBias) * ayScale;
      accelZ = (mpu_accelZ - azBias) * azScale;
      // Keep gyro in filtered raw LSB here; each filter converts to deg/s and applies stored deg/s bias.
      gyroX = filtered_gyroX;
      gyroY = filtered_gyroY;
      gyroZ = filtered_gyroZ;
      
      // Initialize filter after warmup period with stable accel data (only if MPU initialized)
      if (!filterInitialized && mpuInitialized && (loopStart - warmupStartTime) >= WARMUP_TIME) {
        #ifdef USE_KALMAN_FILTER
          initKalmanFilter();
        #elif defined(USE_MAHONY_FILTER)
          initMahonyFilter();
        #elif defined(USE_MADGWICK_FILTER)
          initMadgwickFilter();
        #elif defined(USE_COMPLEMENTARY_QUATERNION_FILTER)
          initComplementaryQuaternionFilter();
        #elif defined(USE_EKF_FILTER)
          initEKFFilter();
        #endif
        filterInitialized = true;
        if (debugMonitoring) {
          Serial.println("[OK] Filter initialized with stable accel data");
        }
      }
      
      // ===== UPDATE FILTER (Choose ONE - comment out other 4) =====
      // Measure filter execution time
      unsigned long filterStart = micros();
      
      #ifdef USE_KALMAN_FILTER
        updateKalmanFilter();
      #elif defined(USE_MAHONY_FILTER)
        if (filterInitialized) updateMahonyFilter();
      #elif defined(USE_MADGWICK_FILTER)
        if (filterInitialized) updateMadgwickFilter();
      #elif defined(USE_COMPLEMENTARY_FILTER)
        updateComplementaryFilter();
      #elif defined(USE_COMPLEMENTARY_QUATERNION_FILTER)
        if (filterInitialized) updateComplementaryQuaternionFilter();
      #elif defined(USE_EKF_FILTER)
        if (filterInitialized) updateEKFFilter();
      #else
        #error "NO FILTER SELECTED! Enable one filter in filter_selector.h"
      #endif
      
      filterExecutionTime = micros() - filterStart;
      avgFilterTime = TIME_ALPHA * filterExecutionTime + (1.0 - TIME_ALPHA) * avgFilterTime;
      
      // Apply trim on final angle output so it follows axis swap + inversion logic
      pitch_final = PITCH_ANGLE_RAW + trim_pitch;
      roll_final = ROLL_ANGLE_RAW + trim_roll;

      // Update ToF sensor (non-blocking state machine)
      #if TOF_ENABLED
      tof_update();
      #endif

      // Altitude fusion disabled
      
      freeHeapMemory = ESP.getFreeHeap();
      if (freeHeapMemory < minFreeHeap) {
        minFreeHeap = freeHeapMemory;
      }
      
      // Apply low-pass filter to FINAL trimmed angles for failsafe (smooth out spikes)
      filtered_pitch = FAILSAFE_LPF_ALPHA * PITCH_ANGLE_FINAL_USED + (1.0 - FAILSAFE_LPF_ALPHA) * filtered_pitch;
      filtered_roll = FAILSAFE_LPF_ALPHA * ROLL_ANGLE_FINAL_USED + (1.0 - FAILSAFE_LPF_ALPHA) * filtered_roll;
      
      // ===== UPDATE ARM HYSTERESIS =====
      // Check if drone is level enough to allow arming (only if MPU is initialized)
      if (mpuInitialized && abs(filtered_pitch) < SAFE_ANGLE_THRESHOLD && abs(filtered_roll) < SAFE_ANGLE_THRESHOLD) {
        safeAngleCounter++;
        if (safeAngleCounter >= SAFE_ANGLE_HYSTERESIS_CHECKS) {
          safeToArm = true;  // Drone has been level for required duration
          safeAngleCounter = SAFE_ANGLE_HYSTERESIS_CHECKS;  // Cap the counter
        }
      } else {
        // Angle exceeded safe threshold - reset counter and disallow arming
        safeAngleCounter = 0;
        safeToArm = false;
      }
      
      if (!motorsArmed) {
        motorsActive = false;
      }

      if (testMotorActive) {
        // Motor test runs on Core 1; skip control loop motor output.
      }
      else if (motorsArmed && motorsActive) {
        // Failsafe: Disarm if MPU6050 becomes disconnected
        if (!mpuInitialized) {
          motorsArmed = false;
          motorsActive = false;
          safeToArm = false;
          safeAngleCounter = 0;
          statusMonitoring = false;
          debugMonitoring = false;
          stopMotors();
          
          // Reset all control variables
          throttle = 0.0f;
          pidIntegral_Pitch = 0.0f;
          pidIntegral_Roll = 0.0f;
          pidIntegral_Yaw = 0.0f;
          #if ENABLE_CASCADE_PID
          resetCascadePID();
          #endif
          
          Serial.println("\n[FAILSAFE] MPU6050 disconnected during flight - Motors DISARMED!");
          
          // Force immediate broadcast to WebSocket UI
          broadcastTelemetryImmediate();
        }
        // Failsafe: Disarm if filtered pitch or roll exceeds Ã‚Â±45Ã‚Â° angle limit
        else if (abs(filtered_pitch) > 45.0f || abs(filtered_roll) > 45.0f) {
          motorsArmed = false;
          motorsActive = false;
          safeToArm = false;  // Also reset arm hysteresis on failsafe
          safeAngleCounter = 0;
          statusMonitoring = false;  // Stop status printing on failsafe
          debugMonitoring = false;   // Stop debug printing on failsafe
          stopMotors();
          
          // Reset all control variables to prevent motor twitching/spin-up
          throttle = 0.0f;
          pidIntegral_Pitch = 0.0f;
          pidIntegral_Roll = 0.0f;
          pidIntegral_Yaw = 0.0f;
          #if ENABLE_CASCADE_PID
          resetCascadePID();
          #endif
          
          Serial.print("\n[FAILSAFE] Pitch: "); Serial.print(filtered_pitch, 2);
          Serial.print("Ã‚Â° | Roll: "); Serial.print(filtered_roll, 2);
          Serial.println("Ã‚Â° exceeded Ã‚Â±45Ã‚Â° - Motors DISARMED! (All control vars reset)");
          
          // FORCE immediate broadcast to WebSocket UI so button syncs immediately
          // Don't wait for rate-limited broadcastTelemetry()
          broadcastTelemetryImmediate();
        } else {
          // ===== SIMPLE BRAKING SYSTEM =====
          // Apply braking when joystick is released for faster stopping
          applyBraking();

          // ===== VEHICLE INPUT SAFETY LIMITS =====
          // Balancing mode clamps joystick-driven tilt setpoints.
          applyVehicleInputLimits();
          
          // ===== SELECT PID CONTROLLER =====
          #if ENABLE_CASCADE_PID
          // Cascade PID: outer angle loop + inner rate loop
          updateCascadePID();
          #else
          // Standard dual-axis PID (pitch + roll + yaw control)
          updateDualPID();
          #endif
          
          updateVehicleMotorControl();
        }
      } else {
        stopMotors();
      }
      
      loopTimeMs = millis() - loopStart;
      loopTimeUs = loopTimeMs * 1000UL;
    }
    
    // Prevent watchdog timeout
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

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
  
  // Initialize I2C bus (used by MPU6050 path and optional barometer)
  Wire.begin(I2C_SDA, I2C_SCL);  // SDA, SCL from config
  Wire.setClock(I2C_SPEED);      // I2C speed from config
  
  // Initialize IMU
  delay(100);
  if (!mpu.initialize()) {
    #if IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6500_SPI
    Serial.println("[ERROR] MPU6500 (SPI) not found! Drone cannot be armed until sensor is properly connected and initialized.");
    #elif IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6500_I2C
    Serial.println("[ERROR] MPU6500 (I2C) not found! Drone cannot be armed until sensor is properly connected and initialized.");
    Serial.printf("[ERROR] I2C config -> SDA: GPIO %d, SCL: GPIO %d, Speed: %d Hz\n", I2C_SDA, I2C_SCL, I2C_SPEED);
    Serial.println("[ERROR] Check wiring + pull-ups, and confirm MPU address (0x68/0x69)");
    #else
    Serial.println("[ERROR] MPU6050 not found! Drone cannot be armed until sensor is properly connected and initialized.");
    Serial.printf("[ERROR] I2C config -> SDA: GPIO %d, SCL: GPIO %d, Speed: %d Hz\n", I2C_SDA, I2C_SCL, I2C_SPEED);
    Serial.println("[ERROR] Check wiring + pull-ups, and confirm MPU address (0x68/0x69)");
    #endif
    mpuInitialized = false;
  } else {
    #if IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6500_SPI
    Serial.println("[OK] MPU6500 (SPI) initialized");
    #elif IMU_SENSOR_PROFILE == IMU_SENSOR_MPU6500_I2C
    Serial.println("[OK] MPU6500 (I2C) initialized");
    #else
    Serial.println("[OK] MPU6050 initialized");
    #endif
    mpuInitialized = true;
  }
  delay(500);

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
  // Reset to default values from config.h
  KP = DEFAULT_KP;
  KI = DEFAULT_KI;
  KD = DEFAULT_KD;
  
  KP_Pitch = DEFAULT_KP;
  KI_Pitch = DEFAULT_KI;
  KD_Pitch = DEFAULT_KD;
  
  KP_Roll = DEFAULT_KP;
  KI_Roll = DEFAULT_KI;
  KD_Roll = DEFAULT_KD;
  
  KP_Yaw = DEFAULT_KP;
  KI_Yaw = DEFAULT_KI;
  KD_Yaw = DEFAULT_KD;
  
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
  KP_Pitch = prefs.getFloat("KP_Pitch", DEFAULT_KP);
  KI_Pitch = prefs.getFloat("KI_Pitch", DEFAULT_KI);
  KD_Pitch = prefs.getFloat("KD_Pitch", DEFAULT_KD);
  
  // Load Roll PID gains with defaults
  KP_Roll = prefs.getFloat("KP_Roll", DEFAULT_KP);
  KI_Roll = prefs.getFloat("KI_Roll", DEFAULT_KI);
  KD_Roll = prefs.getFloat("KD_Roll", DEFAULT_KD);
  
  // Load Yaw PID gains with defaults
  KP_Yaw = prefs.getFloat("KP_Yaw", DEFAULT_KP);
  KI_Yaw = prefs.getFloat("KI_Yaw", DEFAULT_KI);
  KD_Yaw = prefs.getFloat("KD_Yaw", DEFAULT_KD);

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
  Serial.println("[OK] PID values loaded from NVS preferences");
}

// ===== MPU INITIALIZATION RETRY (Recovery Function) =====
void retryMPUInitialization() {
  Serial.println("[INFO] Attempting to retry MPU initialization...");
  
  // Re-initialize I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  // Attempt to re-initialize MPU object
  if (mpu.initialize()) {
    mpuInitialized = true;
    Serial.println("[OK] MPU successfully re-initialized");
    filterInitialized = false;  // Reset filter to warm up from next reading
    Serial.println("[INFO] Filter state reset - will warm up from next reading");
  } else {
    Serial.println("[ERROR] MPU re-initialization failed - check sensor connection");
    mpuInitialized = false;
  }
}
