#include "../config/settings.h"  // FIRST: macros used by serial_commands.h (#if guards)
#include "serial_commands.h"
#include "../control/motor_control.h"  // stopMotors()
#include "../control/pid_controller.h"  // speed setpoints
#include "../control/cascade_pid_controller.h"  // cascade externs (when enabled)
#include "wifi_ota.h"  // switchWiFiMode(), printWiFiStatus()
#include "websocket_handler.h"  // printStateJsonSerial() — shared state JSON for WebUI
#include "../system/led.h"  // ledBootTest(), ledSet(), ledSetRGB()
#include "../config/pins_live.h"  // pins show / pin set / pins save|reset (NVS)
#include "../system/imu.h"  // printImuStatus() for 'imu show'
#include "wifi_creds.h"  // wifi show / wifi set / wifi save|reset (NVS, passwords masked)

// Definitions live here (were in balancing_robot.ino); externs in serial_commands.h
float accel_z_world_mps2 = 0.0f;
bool statusMonitoring = false;
bool debugMonitoring = false;

// ===== Print Welcome Banner =====
void printWelcomeBanner() {
  Serial.println("\n========================================");
  Serial.println("Balancing Robot v1.0 - MPU6050 Protected");
  Serial.println("========================================");
  Serial.println("\n[SAFETY] Robot cannot arm without MPU6050 initialization");
  Serial.println("         Use 'retry_mpu' command if sensor fails to connect");
  Serial.println("\nAvailable Commands:");
  Serial.println("  help           - Display this help message");
  Serial.println("  arm            - Enable motors");
  Serial.println("  disarm         - Disable motors");
  Serial.println("  reboot         - Disarm motors and restart ESP32 (also: restart)");
  Serial.println("  t <value>      - Set throttle (0-100%)");
  Serial.println("\nSpeed Setpoint Control (Cascaded):");
  Serial.println("  sp <val>       - Set forward/backward speed (-2 to +2 m/s)");
  Serial.println("  sr <val>       - Set left/right speed (-2 to +2 m/s)");
  Serial.println("  sy <val>       - Set yaw angle");
  Serial.println("  rsp            - Reset all setpoints to 0");
  Serial.println("\nTrim & Calibration:");
  Serial.println("  trim_pitch <val> - Pitch trim bias offset");
  Serial.println("  trim_roll <val>  - Roll trim bias offset");
  Serial.println("  calibrate_gyro - Calibrate gyro bias only");
  Serial.println("  calibrate_accel - Calibrate accel scale (4-step)");
  Serial.println("  retry_mpu - Retry MPU6050 initialization");
  Serial.println("  imu show       - Show IMU detect mode + active driver");
  Serial.println("\nPID Tuning (Pitch Axis):");
  Serial.println("  pitch_p/pp <val> - Set KP_Pitch");
  Serial.println("  pitch_i/pi <val> - Set KI_Pitch");
  Serial.println("  pitch_d/pd <val> - Set KD_Pitch");
  Serial.println("\nPID Tuning (Roll Axis):");
  Serial.println("  roll_p/rp <val>  - Set KP_Roll");
  Serial.println("  roll_i/ri <val>  - Set KI_Roll");
  Serial.println("  roll_d/rd <val>  - Set KD_Roll");
  Serial.println("\nPID Tuning (Yaw Axis):");
  Serial.println("  yaw_p/yp <val>   - Set KP_Yaw");
  Serial.println("  yaw_i/yi <val>   - Set KI_Yaw");
  Serial.println("  yaw_d/yd <val>   - Set KD_Yaw");
  Serial.println("  yaw_rate/yr <val> - Set yaw rate target (deg/s)");
#if ENABLE_CASCADE_PID
  Serial.println("\n*** CASCADE PID MODE (RUNTIME-ONLY) ***");
  #if CASCADE_MODE_ANGLE_CONTROL
  Serial.println("  Mode: ANGLE CONTROL (outer angle loop + inner rate loop)");
  Serial.println("  Pitch/Roll setpoints are angle targets (degrees)");
  #else
  Serial.println("  Mode: RATE CONTROL (direct rate control, skip angle loop)");
  Serial.println("  Pitch/Roll setpoints are rate targets (deg/s)");
  Serial.println("  pitch_rate/pr <val> - Set pitch rate target (deg/s)");
  Serial.println("  roll_rate/rr <val> - Set roll rate target (deg/s)");
  #endif
  Serial.println("  app <val> - Set cascade pitch angle KP");
  Serial.println("  api <val> - Set cascade pitch angle KI");
  Serial.println("  apd <val> - Set cascade pitch angle KD");
  Serial.println("  rpp <val> - Set cascade pitch rate KP");
  Serial.println("  rpi <val> - Set cascade pitch rate KI");
  Serial.println("  rpd <val> - Set cascade pitch rate KD");
  Serial.println("  arp <val> - Set cascade roll angle KP");
  Serial.println("  ari <val> - Set cascade roll angle KI");
  Serial.println("  ard <val> - Set cascade roll angle KD");
  Serial.println("  rrp <val> - Set cascade roll rate KP");
  Serial.println("  rri <val> - Set cascade roll rate KI");
  Serial.println("  rrd <val> - Set cascade roll rate KD");
  Serial.println("  Note: Values are auto-saved to NVS and restored on reboot");

#else
  Serial.println("\n*** SINGLE ANGLE PID MODE ENABLED ***");
#endif
  Serial.println("\nOther Commands:");
  Serial.println("  load           - Print current PID values");
  Serial.println("  pins show      - List board pins (live values + NVS/defaults source)");
  Serial.println("  pin set <N> <gpio> - Stage a pin (ENA IN1 IN2 ENB IN3 IN4 SDA SCL BAT LED SCK MOSI MISO CS; -1 = unused for SPI/BAT/LED)");
  Serial.println("  pins save      - Persist staged pins to NVS (reboot to apply)");
  Serial.println("  pins reset     - Clear pin overrides, restore board defaults (reboot to apply)");
  Serial.println("  reset_pid      - Reset PID to defaults");
  Serial.println("  reset_calibration - Reset calibration to defaults");
  Serial.println("  battery_reset/bat_reset - Reset low voltage warning");
  Serial.println("  i2c_scan       - Scan I2C bus for connected devices");
  Serial.println("  led test       - Run status LED boot self-test");
  Serial.println("  led on/off     - Force status LED on or off");
  Serial.println("  led rgb R G B  - Set status LED color 0-255 (RGB boards only)");
  Serial.println("  test_motor     - Wheel test: LEFT fwd/rev then RIGHT fwd/rev, 60%, 2s each");
  Serial.println("  status         - Toggle sensor monitoring");
  Serial.println("  debug          - Toggle debug information");
  Serial.println("  ws_debug       - Toggle WebSocket message log (default OFF)");
  Serial.println("\nWiFi & OTA Commands:");
  Serial.println("  wifi/wifi_status - Show WiFi status (passwords masked)");
  Serial.println("  wifi show      - Show WiFi config + NVS/defaults source");
  Serial.println("  wifi set <ssid|pass|ap_ssid|ap_pass> <value> - Stage credential (case-sensitive)");
  Serial.println("  wifi save      - Persist WiFi config to NVS (reboot to apply)");
  Serial.println("  wifi reset     - Clear WiFi overrides back to settings.h defaults");
  Serial.println("  wifi_sta/sta   - Switch to STA mode (connect to WiFi)");
  Serial.println("  wifi_ap/ap     - Switch to AP mode (create WiFi hotspot)");
  Serial.println("  (OTA ready: Arduino IDE > Tools > Port > Network Ports)");
  Serial.println("\nType a command and press Enter:");
  Serial.println("========================================\n");
}

// ===== Handle Serial Commands =====
void handleSerialCommand() {
  if (Serial.available() > 0) {
    String raw = Serial.readStringUntil('\n');
    raw.trim();
    String command = raw;  // lowercased dispatch copy; raw keeps case for wifi creds
    command.trim();
    command.toLowerCase();

    // Handle calibration commands
    if (calibrationState != CALIB_IDLE) {
      processCalibrationStep(command);
      return;
    }

    if (command == "calibrate_gyro") {
      startGyroCalibration();
    }
    else if (command == "calibrate_accel") {
      startAccelCalibration();
    }
    else if (command == "retry_mpu") {
      retryMPUInitialization();
    }
    else if (command == "imu show") {
      printImuStatus();
    }
    else if (command == "test_motor") {
      toggleMotorTest();
    }
    else if (command == "arm") {
      // Check MPU6050 initialization
      if (testMotorActive) {
        Serial.println("\n[WARN] ARM BLOCKED: Motor test running. Type 'test_motor' to stop.");
      }
      else if (!mpuInitialized) {
        Serial.println("\n[ERROR] ARM BLOCKED: MPU6050 not initialized! Check sensor connection.");
      }
      // Check filter is warmed up
      else if (!filterInitialized) {
        Serial.println("\n[WARN] ARM BLOCKED: Filter still warming up, please wait...");
      }
      // Check if robot is level using hysteresis (safeToArm flag)
      else if (!safeToArm) {
        Serial.print("\n[WARN] ARM BLOCKED: Robot must be level! Pitch: "); Serial.print(filtered_pitch, 1);
        Serial.print("° Roll: "); Serial.print(filtered_roll, 1);
        Serial.println("° (Must be < 40° for 200ms to allow arming)");
      } else {
        motorsArmed = true;
        motorsActive = true;
        throttle = 0.0f;  // Minimum throttle on arm
        Serial.println("\n[INFO] Balancing Robot ARMED - Motors ready");
        pidIntegral_Pitch = 0;
      }
    }
    else if (command == "disarm") {
      motorsArmed = false;
      motorsActive = false;
      throttle = 0.0f;  // Reset throttle on disarm
      pidIntegral_Pitch = 0.0;  // Reset integral on disarm
      stopMotors();
      Serial.println("\n[INFO] Balancing Robot DISARMED - Motors stopped");
    }
    else if (command == "reboot" || command == "restart") {
      motorsArmed = false;  // safety: never reboot with motors live
      motorsActive = false;
      throttle = 0.0f;
      stopMotors();
      Serial.println("\n[INFO] Rebooting...");
      Serial.flush();
      delay(200);
      ESP.restart();
    }
    else if (command.startsWith("t ")) {
      float value = command.substring(2).toFloat();
      throttle = constrain(value, 0, 100);
      Serial.print("\n[THROTTLE] Set to: "); Serial.print((int)throttle); Serial.println("%");
    }
    else if (command.startsWith("trim_pitch ")) {
      float value = command.substring(11).toFloat();
      trim_pitch = constrain(value, -45.0f, 45.0f);
      // Save to preferences
      Preferences prefs;
      prefs.begin("mpu6050", false);
      prefs.putFloat("trim_pitch", trim_pitch);
      prefs.end();
      delay(100);  // Critical: Allow NVS to flush on ESP32-C3
      Serial.print("\n[TRIM] Pitch offset set to: "); Serial.print(trim_pitch, 2); Serial.println("° (saved)");
    }
    else if (command.startsWith("trim_roll ")) {
      float value = command.substring(10).toFloat();
      trim_roll = constrain(value, -45.0f, 45.0f);
      // Save to preferences
      Preferences prefs;
      prefs.begin("mpu6050", false);
      prefs.putFloat("trim_roll", trim_roll);
      prefs.end();
      delay(100);  // Critical: Allow NVS to flush on ESP32-C3
      Serial.print("\n[TRIM] Roll offset set to: "); Serial.print(trim_roll, 2); Serial.println("° (saved)");
    }
    else if (command == "help") {
      printWelcomeBanner();
    }
    else if (command == "i2c_scan") {
      scanI2CBus();
    }
    else if (command == "led test") {
      ledBootTest();
    }
    else if (command == "led on") {
      ledSet(true);
      Serial.println("[LED] forced ON (battery monitor resumes control next update)");
    }
    else if (command == "led off") {
      ledSet(false);
      Serial.println("[LED] forced OFF (battery monitor resumes control next update)");
    }
    else if (command.startsWith("led rgb ")) {
#ifdef STATUS_LED_RGB
      int r, g, b;
      if (sscanf(command.c_str(), "led rgb %d %d %d", &r, &g, &b) == 3) {
        ledSetRGB((uint8_t)constrain(r, 0, 255), (uint8_t)constrain(g, 0, 255), (uint8_t)constrain(b, 0, 255));
        Serial.printf("[LED] color set to R=%d G=%d B=%d\n", constrain(r, 0, 255), constrain(g, 0, 255), constrain(b, 0, 255));
      } else {
        Serial.println("[LED] usage: led rgb <0-255> <0-255> <0-255>");
      }
#else
      Serial.println("[LED] this board has a plain LED, no RGB support");
#endif
    }
    else if (command == "status") {
      statusMonitoring = !statusMonitoring;  // Toggle status monitoring
      if (statusMonitoring) {
        Serial.println("\n[INFO] Status monitoring ENABLED - Type 'status' again to disable");
      } else {
        Serial.println("\n[INFO] Status monitoring DISABLED\n");
      }
    }
    else if (command == "debug") {
      debugMonitoring = !debugMonitoring;  // Toggle debug monitoring
      if (debugMonitoring) {
        Serial.println("\n[DEBUG] Debug monitoring ENABLED - Type 'debug' again to disable");
      } else {
        Serial.println("\n[DEBUG] Debug monitoring DISABLED\n");
      }
    }
    else if (command == "ws_debug") {
      wsDebugMonitoring = !wsDebugMonitoring;  // Toggle WebSocket message log
      if (wsDebugMonitoring) {
        Serial.println("\n[WS] Message log ENABLED - Type 'ws_debug' again to disable");
      } else {
        Serial.println("\n[WS] Message log DISABLED\n");
      }
    }
    // ===== SPEED SETPOINT CONTROL (Cascaded Control) =====
    // sp = speed pitch (forward/backward speed in m/s)
    else if (command.startsWith("sp ")) {
      float value = command.substring(3).toFloat();
      speed_x_setpoint = constrain(value, -2.0f, 2.0f);
      Serial.print("\n[SP] Speed X: "); Serial.print(speed_x_setpoint, 2); Serial.println(" m/s");
    }
    // sr = speed roll (left/right speed in m/s)
    else if (command.startsWith("sr ")) {
      float value = command.substring(3).toFloat();
      speed_y_setpoint = constrain(value, -2.0f, 2.0f);
      Serial.print("[SR] Speed Y: "); Serial.print(speed_y_setpoint, 2); Serial.println(" m/s");
    }
    // ===== YAW SETPOINT CONTROL (sy) =====
    else if (command.startsWith("sy ")) {
      float value = command.substring(3).toFloat();
      yaw_setpoint = value;
      Serial.print("[SY] Y: "); Serial.print(yaw_setpoint, 2); Serial.println("°");
    }
    // ===== RESET ALL SETPOINTS =====
    else if (command == "rsp") {
      speed_x_setpoint = 0.0f;
      speed_y_setpoint = 0.0f;
      yaw_setpoint = 0.0f;
      Serial.println("\n[RSP] All setpoints reset to 0");
    }
    else if (command == "pins show") {
      printPinsToSerial();
    }
    else if (command == "pins save") {
      savePinsToNVS();
      Serial.println("[PINS] saved to NVS - reboot to apply");
    }
    else if (command == "pins reset") {
      resetPinsToDefaults();
      Serial.println("[PINS] overrides cleared, defaults restored - reboot to apply");
    }
    else if (command.startsWith("pin set ") || command.startsWith("pins set ")) {
      // Staged in RAM only; 'pins save' persists, reboot applies.
      String args = command.startsWith("pin set ")
          ? command.substring(8) : command.substring(9);
      args.trim();
      int sp = args.indexOf(' ');
      if (sp <= 0) {
        Serial.println("[PINS] usage: pin set <NAME> <gpio>");
      } else {
        String pname = args.substring(0, sp);
        int gpio = args.substring(sp + 1).toInt();
        String perr;
        if (setStagedPin(pname, gpio, perr)) {
          Serial.printf("[PINS] %s staged to GPIO %d (unsaved - 'pins save' + reboot)\n",
                        pname.c_str(), gpio);
        } else {
          Serial.print("[PINS] rejected: ");
          Serial.println(perr);
        }
      }
    }
    else if (handlePIDCommand(command)) {
      // PID-tuning sub-commands (see serial_pid_commands.cpp)
    }
    else if (command == "reset_calibration") {
      resetCalibrationToDefaults();
    }
    else if (command == "load") {
      // Print the same state JSON the WebSocket broadcasts, so the WebUI
      // (USB serial transport) can autofill PID/trim inputs.
      printStateJsonSerial();
    }

    // WiFi & OTA Commands
    else if (command == "wifi_status" || command == "wifi") {
      printWiFiStatus();
    }
    else if (command == "wifi show") {
      printWifiConfigMasked();
    }
    else if (command == "wifi save") {
      saveWifiToNVS();
      Serial.println("[WIFI] saved to NVS - reboot to apply");
    }
    else if (command == "wifi reset") {
      resetWifiToDefaults();
      Serial.println("[WIFI] overrides cleared, settings.h defaults restored - reboot to apply");
    }
    else if (command.startsWith("wifi set ")) {
      // Value parsed from raw to preserve case (SSID/passwords are case-sensitive).
      String args = raw.substring(9);
      args.trim();
      int sp = args.indexOf(' ');
      if (sp <= 0) {
        Serial.println("[WIFI] usage: wifi set <ssid|pass|ap_ssid|ap_pass> <value>");
      } else {
        String fname = args.substring(0, sp);
        fname.toLowerCase();
        String fvalue = args.substring(sp + 1);
        fvalue.trim();
        String werr;
        if (setStagedWifi(fname, fvalue, werr)) {
          Serial.printf("[WIFI] %s staged (unsaved - 'wifi save' + reboot)\n", fname.c_str());
        } else {
          Serial.print("[WIFI] rejected: ");
          Serial.println(werr);
        }
      }
    }
    else if (command == "wifi_ap" || command == "ap") {
      Serial.println("[WiFi] Switching to AP mode...");
      useAPMode = true;
      switchWiFiMode();
    }
    else if (command == "wifi_sta" || command == "sta") {
      Serial.println("[WiFi] Switching to STA mode...");
      useAPMode = false;
      switchWiFiMode();
    }    // Battery Commands
    else if (command == "battery_reset" || command == "bat_reset") {
      batteryState = BATTERY_NORMAL;
      // LED will be set by updateBatteryMonitoring() based on ESP-NOW connection status
      Serial.println("[BATTERY] Low voltage warning reset. Monitoring restarted.");
    }    else if (command != "") {
      Serial.println("[ERROR] Unknown command: " + command);
    }
  }
}

// ===== Rate-limited [LOOP] telemetry status (moved from loop() verbatim) =====
void printTelemetryStatus() {
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
    Serial.print("[FILT] Exec: "); Serial.print(filterExecutionTime); Serial.print("us (avg: ");
    Serial.print(avgFilterTime, 1); Serial.print("us) | Heap: "); Serial.print(freeHeapMemory);
    Serial.print("B (min: "); Serial.print(minFreeHeap); Serial.print("B) | Load: ");
    float cpuLoad = (avgFilterTime / 10000.0) * 100.0;
    Serial.print(cpuLoad, 1); Serial.println("%");
  }
}
