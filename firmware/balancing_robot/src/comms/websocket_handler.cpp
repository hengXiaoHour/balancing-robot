#include <Arduino.h>
#include <Preferences.h>
#include "websocket_handler.h"
#include "../control/motor_control.h"  // stopMotors()
#include "../control/pid_controller.h"  // speed_x/y_setpoint, yaw_rate_target

// ===== WebSocket Server =====
WebServer webServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// WebSocket connection flag
bool webSocketConnected = false;

// ===== Telemetry rate limiting (local to this TU) =====
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 100;  // Send every 100ms

// ===== Initialize WebSocket Server =====
void initWebSocket() {
  Serial.println("\n===== WebSocket Setup =====");

  // Setup WebSocket server
  webSocket.begin();
  webSocket.onEvent(handleWebSocketEvent);
  Serial.println("[WS] WebSocket server started on port 81");

  Serial.println("=============================\n");
}

// ===== WebSocket Event Handler =====
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.print("[WS] Client #"); Serial.print(num); Serial.println(" disconnected");
      webSocketConnected = false;
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.print("[WS] Client #"); Serial.print(num); Serial.print(" connected from ");
      Serial.println(ip);
      webSocketConnected = true;
      // Send current state to new client
      broadcastState();
      break;
    }

    case WStype_TEXT: {
      // Parse incoming JSON command
      String msg = "";
      for(size_t i = 0; i < length; i++) {
        msg += (char)payload[i];
      }
      Serial.print("[WS] Received: "); Serial.println(msg);
      handleWebSocketCommand(msg);
      break;
    }

    case WStype_BIN:
      Serial.println("[WS] Binary data received (not supported)");
      break;

    default:
      break;
  }
}

// ===== Handle WebSocket Commands =====
void handleWebSocketCommand(const String& jsonStr) {
  // Simple JSON parsing without ArduinoJson to avoid library conflicts
  // Parse format: {"command":"value"} or {"pitch":0.5,"roll":0.3,"yaw":0.1,"throttle":50,"arm":true}

  // Handle joystick/control commands. UI sends stick degrees; firmware fans
  // them out to BOTH control paths: angle setpoints (single/dual PID,
  // cascade angle mode) and speed setpoints (cascade velocity loop).
  if (jsonStr.indexOf("\"pitch\":") != -1) {
    int start = jsonStr.indexOf("\"pitch\":") + 8;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String pitchStr = jsonStr.substring(start, end);
    float pitchInput = pitchStr.toFloat() * CONTROLLER_PITCH_SIGN;
    // Angle-mode path: stick degrees -> lean target (deg)
    pitch_setpoint = constrain(pitchInput, -ESPNOW_MAX_PITCH, ESPNOW_MAX_PITCH);
    // Cascade path: stick degrees -> desired forward/backward speed (m/s)
    #if JOYSTICK_SWAP_ROLL_PITCH_INPUT
    speed_y_setpoint = pitchInput * 2.0f;
    #else
    speed_x_setpoint = pitchInput * 2.0f;
    #endif
  }
  if (jsonStr.indexOf("\"roll\":") != -1) {
    int start = jsonStr.indexOf("\"roll\":") + 7;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String rollStr = jsonStr.substring(start, end);
    float rollInput = rollStr.toFloat() * CONTROLLER_ROLL_SIGN;
    // Angle-mode path: stick degrees -> lean target (deg)
    roll_setpoint = constrain(rollInput, -ESPNOW_MAX_ROLL, ESPNOW_MAX_ROLL);
    // Cascade path: stick degrees -> desired lateral speed (m/s)
    #if JOYSTICK_SWAP_ROLL_PITCH_INPUT
    speed_x_setpoint = rollInput * 2.0f;
    #else
    speed_y_setpoint = rollInput * 2.0f;
    #endif
  }
  if (jsonStr.indexOf("\"yaw\":") != -1) {
    int start = jsonStr.indexOf("\"yaw\":") + 6;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String yawStr = jsonStr.substring(start, end);
    float yawInput = yawStr.toFloat() * CONTROLLER_YAW_SIGN;
    #if JOYSTICK_YAW_INPUT_ENABLED
    // Yaw is always rate mode: updateYawPID() reads yaw_rate_target (deg/s)
    yaw_rate_target = constrain(yawInput, -ESPNOW_MAX_YAW_RATE, ESPNOW_MAX_YAW_RATE);
    yaw_setpoint = yaw_rate_target;
    #else
    yaw_setpoint = 0.0f;
    yaw_rate_target = 0.0f;
    #endif
  }
  if (jsonStr.indexOf("\"throttle\":") != -1) {
    int start = jsonStr.indexOf("\"throttle\":") + 11;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String throttleStr = jsonStr.substring(start, end);
    throttle = constrain(throttleStr.toFloat(), 0.0f, 100.0f);
  }

  // Handle arm/disarm commands
  if (jsonStr.indexOf("\"arm\":") != -1) {
    int start = jsonStr.indexOf("\"arm\":") + 6;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String armStr = jsonStr.substring(start, end);
    bool armCmd = (armStr == "true" || armStr == "1");

    if (armCmd && !motorsArmed) {
      // Check MPU6050 initialization
      if (!mpuInitialized) {
        broadcastConsoleMessage("[ERROR] ARM BLOCKED: MPU6050 not initialized! Check sensor connection.");
      }
      // Check filter is warmed up
      else if (!filterInitialized) {
        broadcastConsoleMessage("[WARN] ARM BLOCKED: Filter still warming up, please wait...");
      }
      // Check if robot is level using hysteresis (safeToArm flag)
      else if (!safeToArm) {
        broadcastConsoleMessage("[WARN] ARM BLOCKED: Robot must be level! Pitch: " + String(filtered_pitch, 1) + "° Roll: " + String(filtered_roll, 1) + "° (Must be < 40° for 200ms to allow arming)");
      } else {
        motorsArmed = true;
        motorsActive = true;
        throttle = MIN_THROTTLE;  // Set to minimum throttle on arm
        broadcastConsoleMessage("[INFO] Balancing Robot ARMED via WebSocket - Motors ready");
        pidIntegral = 0;
      }
    }
    else if (!armCmd && motorsArmed) {
      motorsArmed = false;
      motorsActive = false;
      throttle = 0.0f;
      pidIntegral = 0.0f;
      stopMotors();
      broadcastConsoleMessage("[INFO] Balancing Robot DISARMED via WebSocket");
    }
  }

  // Handle PID tuning (simple string parsing)
  if (jsonStr.indexOf("\"pid_tune\":") != -1) {
    // Parse PID tuning commands like {"pid_tune":{"pitch_p":1.5,"pitch_i":0.1,"pitch_d":0.5}}

    if (jsonStr.indexOf("\"pitch_p\":") != -1) {
      int start = jsonStr.indexOf("\"pitch_p\":") + 10;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KP_Pitch = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Pitch P updated via WebSocket");
    }
    if (jsonStr.indexOf("\"pitch_i\":") != -1) {
      int start = jsonStr.indexOf("\"pitch_i\":") + 10;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KI_Pitch = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Pitch I updated via WebSocket");
    }
    if (jsonStr.indexOf("\"pitch_d\":") != -1) {
      int start = jsonStr.indexOf("\"pitch_d\":") + 10;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KD_Pitch = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Pitch D updated via WebSocket");
    }

    if (jsonStr.indexOf("\"roll_p\":") != -1) {
      int start = jsonStr.indexOf("\"roll_p\":") + 9;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KP_Roll = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Roll P updated via WebSocket");
    }
    if (jsonStr.indexOf("\"roll_i\":") != -1) {
      int start = jsonStr.indexOf("\"roll_i\":") + 9;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KI_Roll = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Roll I updated via WebSocket");
    }
    if (jsonStr.indexOf("\"roll_d\":") != -1) {
      int start = jsonStr.indexOf("\"roll_d\":") + 9;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KD_Roll = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Roll D updated via WebSocket");
    }

    if (jsonStr.indexOf("\"yaw_p\":") != -1) {
      int start = jsonStr.indexOf("\"yaw_p\":") + 8;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KP_Yaw = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Yaw P updated via WebSocket");
    }
    if (jsonStr.indexOf("\"yaw_i\":") != -1) {
      int start = jsonStr.indexOf("\"yaw_i\":") + 8;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KI_Yaw = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Yaw I updated via WebSocket");
    }
    if (jsonStr.indexOf("\"yaw_d\":") != -1) {
      int start = jsonStr.indexOf("\"yaw_d\":") + 8;
      int end = jsonStr.indexOf(",", start);
      if (end == -1) end = jsonStr.indexOf("}", start);
      String valStr = jsonStr.substring(start, end);
      KD_Yaw = valStr.toFloat();
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      broadcastConsoleMessage("[PID] Yaw D updated via WebSocket");
    }
  }

  // Handle trim commands
  if (jsonStr.indexOf("\"trim_pitch\":") != -1) {
    int start = jsonStr.indexOf("\"trim_pitch\":") + 13;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String valStr = jsonStr.substring(start, end);
    trim_pitch = constrain(valStr.toFloat(), -45.0f, 45.0f);
    Preferences prefs;
    prefs.begin("mpu6050", false);
    prefs.putFloat("trim_pitch", trim_pitch);
    prefs.end();
    delay(100);  // Critical: Allow NVS to flush on ESP32-C3
    broadcastConsoleMessage("[TRIM] Pitch offset updated via WebSocket");
  }
  if (jsonStr.indexOf("\"trim_roll\":") != -1) {
    int start = jsonStr.indexOf("\"trim_roll\":") + 12;
    int end = jsonStr.indexOf(",", start);
    if (end == -1) end = jsonStr.indexOf("}", start);
    String valStr = jsonStr.substring(start, end);
    trim_roll = constrain(valStr.toFloat(), -45.0f, 45.0f);
    Preferences prefs;
    prefs.begin("mpu6050", false);
    prefs.putFloat("trim_roll", trim_roll);
    prefs.end();
    delay(100);  // Critical: Allow NVS to flush on ESP32-C3
    broadcastConsoleMessage("[TRIM] Roll offset updated via WebSocket");
  }

  // Handle calibration commands
  if (jsonStr.indexOf("\"calibrate_gyro\"") != -1) {
    startGyroCalibration();
    broadcastConsoleMessage("[CALIB] Gyro calibration started via WebSocket");
  }
  if (jsonStr.indexOf("\"calibrate_accel\"") != -1) {
    startAccelCalibration();
    broadcastConsoleMessage("[CALIB] Accel calibration started via WebSocket");
  }

  // Handle calibration step advancement
  if (jsonStr.indexOf("\"calib_next\"") != -1) {
    if (calibrationState != CALIB_IDLE) {
      recordCalibrationStep();
      advanceCalibrationStep();
    }
  }

  // Handle calibration abort
  if (jsonStr.indexOf("\"calib_abort\"") != -1) {
    calibrationState = CALIB_IDLE;
    broadcastConsoleMessage("[CALIB] Calibration aborted via WebSocket");
  }

  // Handle reset commands
  if (jsonStr.indexOf("\"reset_pid\"") != -1) {
    resetPIDToDefaults();
    broadcastConsoleMessage("[RESET] PID values reset to defaults");
  }
  if (jsonStr.indexOf("\"reset_calibration\"") != -1) {
    resetCalibrationToDefaults();
    broadcastConsoleMessage("[RESET] Calibration data reset to defaults");
  }

  // Handle state load request
  if (jsonStr.indexOf("\"load\"") != -1) {
    broadcastState();
  }
}

// ===== Broadcast Current State (PID, Calibration, Trim) =====
void broadcastState() {
  if (!webSocketConnected) return;

  String output = "{";
  output += "\"pitch_p\":" + String(KP_Pitch) + ",";
  output += "\"pitch_i\":" + String(KI_Pitch) + ",";
  output += "\"pitch_d\":" + String(KD_Pitch) + ",";
  output += "\"roll_p\":" + String(KP_Roll) + ",";
  output += "\"roll_i\":" + String(KI_Roll) + ",";
  output += "\"roll_d\":" + String(KD_Roll) + ",";
  output += "\"yaw_p\":" + String(KP_Yaw) + ",";
  output += "\"yaw_i\":" + String(KI_Yaw) + ",";
  output += "\"yaw_d\":" + String(KD_Yaw) + ",";
  output += "\"trim_pitch\":" + String(trim_pitch) + ",";
  output += "\"trim_roll\":" + String(trim_roll) + ",";
  output += "\"gyro_bias_x\":" + String(gyroBiasX) + ",";
  output += "\"gyro_bias_y\":" + String(gyroBiasY) + ",";
  output += "\"gyro_bias_z\":" + String(gyroBiasZ) + ",";
  output += "\"accel_bias_x\":" + String(axBias) + ",";
  output += "\"accel_bias_y\":" + String(ayBias) + ",";
  output += "\"accel_bias_z\":" + String(azBias);
  output += "}";

  webSocket.broadcastTXT(output);
}

// ===== Broadcast Telemetry (Angles, Motors, Battery) =====
void broadcastTelemetryImmediate() {
  // Force immediate broadcast without rate limiting (for critical events like failsafe)
  if (!webSocketConnected) return;

  lastTelemetryTime = millis();  // Reset timer

  String output = "{";
  output += "\"pitch\":" + String(PITCH_ANGLE_FINAL_USED) + ",";
  output += "\"roll\":" + String(ROLL_ANGLE_FINAL_USED) + ",";
  output += "\"yaw\":" + String(yaw) + ",";
  output += "\"motor_left\":" + String(pidOutput_Left) + ",";
  output += "\"motor_right\":" + String(pidOutput_Right) + ",";
  output += "\"pid_pitch\":" + String(pidOutput_Pitch) + ",";
  output += "\"pid_yaw\":" + String(pidOutput_Yaw) + ",";
  output += "\"armed\":" + String(motorsArmed ? "true" : "false") + ",";
  output += "\"throttle\":" + String(throttle) + ",";
  output += "\"battery\":" + String(battery_voltage) + ",";

  float loopRate = 1000000.0f / (loopTimeUs > 0 ? loopTimeUs : 1);
  output += "\"loop_rate\":" + String(loopRate) + ",";
  output += "\"filter_time\":" + String(filterExecutionTime) + ",";
  output += "\"avg_filter_time\":" + String(avgFilterTime) + ",";
  output += "\"heap_free\":" + String(freeHeapMemory) + ",";
  output += "\"heap_min\":" + String(minFreeHeap) + ",";

  float cpuLoad = (avgFilterTime / 10000.0) * 100.0;
  output += "\"cpu_load\":" + String(cpuLoad) + ",";
  output += "\"filter\":\"" + String(getActiveFilterName()) + "\"";
  output += "}";

  webSocket.broadcastTXT(output);
}

void broadcastTelemetry() {
  if (!webSocketConnected) return;

  unsigned long now = millis();
  if (now - lastTelemetryTime < TELEMETRY_INTERVAL) {
    return;  // Not time yet
  }
  lastTelemetryTime = now;

  String output = "{";
  output += "\"pitch\":" + String(PITCH_ANGLE_FINAL_USED) + ",";
  output += "\"roll\":" + String(ROLL_ANGLE_FINAL_USED) + ",";
  output += "\"yaw\":" + String(yaw) + ",";
  output += "\"motor_left\":" + String(pidOutput_Left) + ",";
  output += "\"motor_right\":" + String(pidOutput_Right) + ",";
  output += "\"pid_pitch\":" + String(pidOutput_Pitch) + ",";
  output += "\"pid_yaw\":" + String(pidOutput_Yaw) + ",";
  output += "\"armed\":" + String(motorsArmed ? "true" : "false") + ",";
  output += "\"throttle\":" + String(throttle) + ",";
  output += "\"battery\":" + String(battery_voltage) + ",";

  float loopRate = 1000000.0f / (loopTimeUs > 0 ? loopTimeUs : 1);
  output += "\"loop_rate\":" + String(loopRate) + ",";
  output += "\"filter_time\":" + String(filterExecutionTime) + ",";
  output += "\"avg_filter_time\":" + String(avgFilterTime) + ",";
  output += "\"heap_free\":" + String(freeHeapMemory) + ",";
  output += "\"heap_min\":" + String(minFreeHeap) + ",";

  float cpuLoad = (avgFilterTime / 10000.0) * 100.0;
  output += "\"cpu_load\":" + String(cpuLoad) + ",";
  output += "\"filter\":\"" + String(getActiveFilterName()) + "\"";
  output += "}";

  webSocket.broadcastTXT(output);
}

// ===== Broadcast Console Message =====
void broadcastConsoleMessage(const String& message) {
  // Always print to serial
  Serial.println(message);

  if (!webSocketConnected) return;

  String output = "{\"console\":\"" + message + "\"}";
  webSocket.broadcastTXT(output);
}

// ===== Process WebSocket in Main Loop =====
void handleWebSocketLoop() {
  webSocket.loop();
  broadcastTelemetry();
}
