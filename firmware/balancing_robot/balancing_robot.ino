// Balancing Robot — ESP32 + L298N + MPU6050
// Sketch entry point: includes, setup(), loop(). Everything else lives in src/.
// See firmware/ARCHITECTURE.md for the module layout.

#include <Wire.h>
#include <Preferences.h>
#include "src/config/settings.h"

// Filters
#include "src/filters/filter_selector.h"
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

// Control
#include "src/control/motor_control.h"
#include "src/control/pid_controller.h"
#if ENABLE_CASCADE_PID
#include "src/control/cascade_pid_controller.h"
#endif

// Sensors
#include "src/sensors/calibration.h"
#include "src/sensors/battery_state.h"

// Utils
#include "src/utils/timing.h"
#include "src/utils/i2c_scan.h"

// System
#include "src/system/prefs.h"
#include "src/system/battery.h"
#include "src/system/led.h"
#include "src/system/imu.h"
#include "src/system/state.h"
#include "src/system/control_task.h"

// Comms
#include "src/comms/serial_commands.h"
#include "src/comms/wifi_ota.h"
#include "src/comms/websocket_handler.h"
#include "src/comms/esp_now_handler.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("[BOOT] balancing robot");

  ledBootTest();
  initVehicleMotors();

  analogReadResolution(12);
  initIMU();

  if (debugMonitoring) {
    scanI2CBus();
  }

  loadCalibrationBias();
  loadPIDFromPreferences();

  #if !ENABLE_ESPNOW
  initWiFi();
  #endif

  // Filter inits lazily on Core 0 once sensors warm up — not here.
  dataLock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    controlLoopTask,
    "ControlLoop",
    4096,
    NULL,
    2,
    NULL,
    0
  );

  #if ENABLE_WEBSOCKET_CONTROL
  initWebSocket();
  #endif

  #if ENABLE_ESPNOW
  initESPNOW();
  #else
  Serial.println("[ESPNOW] off");
  #endif

  Serial.println("[READY] type help");
}

void loop() {
  // Core 1: non-critical tasks. Time-critical control runs on Core 0.
  printTelemetryStatus();

  #if !ENABLE_ESPNOW
  ArduinoOTA.handle();
  handleWiFiConnection();
  #endif

  #if ENABLE_WEBSOCKET_CONTROL
  handleWebSocketLoop();
  #endif

  #if ENABLE_SERIAL_COMMANDS_AND_STATUS
  handleSerialCommand();
  #endif

  updateMotorTest();

  #if ENABLE_ESPNOW
  updateESPNOWService();
  #endif

  if (calibrationState != CALIB_IDLE) {
    updateCalibration();
  }

  pollMpuRetry();

  delay(1);
}
