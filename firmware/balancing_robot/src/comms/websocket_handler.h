#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "../config/settings.h"
#include "../sensors/calibration.h"  // CalibrationState, CALIB_IDLE, record/advance helpers

// ===== External Variables =====
// Global variables from BALANCING_ROBOT.ino
extern volatile unsigned long loopTimeUs;
extern volatile unsigned long filterExecutionTime;
extern volatile uint32_t freeHeapMemory;
extern volatile uint32_t minFreeHeap;
extern float avgFilterTime;

// Control state variables
extern bool motorsArmed;
extern bool motorsActive;
extern bool safeToArm;  // ARM HYSTERESIS: Robot is level enough to arm
extern bool mpuInitialized;  // MPU6050 initialization status
extern bool filterInitialized;  // Filter warm-up complete
extern float pitch_setpoint;
extern float roll_setpoint;
extern float yaw_setpoint;
extern float throttle;
extern float pidIntegral;

// Angle/sensor variables
extern float pitch;
extern float pitch_final;
extern float roll;
extern float roll_final;
extern float filtered_pitch;
extern float filtered_roll;
extern float yaw;

// Motor outputs
extern float pidOutput_Left;
extern float pidOutput_Right;

// PID outputs
extern float pidOutput_Pitch;
extern float pidOutput_Yaw;

// Calibration and trim
extern float trim_pitch;
extern float trim_roll;
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;
extern float axBias;
extern float ayBias;
extern float azBias;

// PID gains
extern float KP_Pitch;
extern float KI_Pitch;
extern float KD_Pitch;
extern float KP_Roll;
extern float KI_Roll;
extern float KD_Roll;
extern float KP_Yaw;
extern float KI_Yaw;
extern float KD_Yaw;

// Battery and other sensors
extern float battery_voltage;

// Calibration state
extern unsigned long lastPIDUpdateTime;

// Forward declarations of functions (defined elsewhere)
extern void stopMotors();
extern void savePIDToPreferences();
extern void resetPIDToDefaults();
#include "../filters/filter_selector.h"  // getActiveFilterName() + USE_* selection

// ===== WebSocket Server (definitions in websocket_handler.cpp) =====
extern WebServer webServer;
extern WebSocketsServer webSocket;

// WebSocket connection flag
extern bool webSocketConnected;

// ===== WebSocket API (see websocket_handler.cpp) =====
void initWebSocket();
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void handleWebSocketCommand(const String& jsonStr);
String buildStateJson();
void printStateJsonSerial();  // same payload over USB serial for the WebUI
void broadcastState();
void broadcastTelemetry();
void broadcastTelemetryImmediate();  // Force immediate broadcast (e.g. failsafe events)
void broadcastConsoleMessage(const String& message);
void handleWebSocketLoop();

#endif
