#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include "i2c_scan.h"

// Forward declarations
void startGyroCalibration();
void startAccelCalibration();
void processCalibrationStep(const String& command);
void retryMPUInitialization();
void savePIDToPreferences();
void resetPIDToDefaults();
void loadPIDFromPreferences();
void toggleMotorTest();

// ===== External Variables =====
extern bool motorsArmed;
extern bool motorsActive;
extern bool statusMonitoring;
extern bool debugMonitoring;
extern float pitch, roll, yaw;  // 3D angles
extern float filtered_pitch;    // Low-pass filtered pitch for failsafe check
extern float filtered_roll;     // Low-pass filtered roll for failsafe check
extern float pitch_setpoint;    // Pitch setpoint (angle target)
extern float roll_setpoint;     // Roll setpoint (angle target)
extern float yaw_setpoint;      // Yaw setpoint (angle target)
extern float pidOutput;
extern float pidOutput_Left;
extern float pidOutput_Right;
extern float pidIntegral;
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY;
extern unsigned long lastLoopTime;
extern unsigned long lastPIDUpdateTime;  // For pausing telemetry on PID updates
extern float KP, KI, KD;  // PID gains
extern float KP_Pitch, KI_Pitch, KD_Pitch;  // Pitch axis gains
extern float KP_Roll, KI_Roll, KD_Roll;    // Roll axis gains
extern float KP_Yaw, KI_Yaw, KD_Yaw;      // Yaw axis gains
extern float yaw_rate_target;              // Yaw rate setpoint
extern bool mpuInitialized;                 // MPU6050 initialization status
extern float throttle;  // Throttle setpoint
extern float throttle_increment;  // Throttle increment per command
extern float trim_pitch;  // Pitch sensor trim bias
extern float trim_roll;   // Roll sensor trim bias
extern float battery_voltage;  // Battery voltage in volts
extern BatteryState batteryState;  // Battery monitoring state
extern bool useAPMode;  // WiFi mode: false = STA, true = AP
extern bool safeToArm;  // ARM HYSTERESIS: Drone is level enough to arm
extern bool filterInitialized;  // Filter warm-up complete
extern bool testMotorActive;
extern void switchWiFiMode();  // Function to switch WiFi modes
extern void printWiFiStatus();  // Function to print WiFi status

// ===== CASCADE PID EXTERNAL VARIABLES =====
#if ENABLE_CASCADE_PID
extern float cascade_pitch_angle_kp;
extern float cascade_pitch_angle_ki;
extern float cascade_pitch_angle_kd;
extern float cascade_pitch_rate_kp;
extern float cascade_pitch_rate_ki;
extern float cascade_pitch_rate_kd;
extern float cascade_roll_angle_kp;
extern float cascade_roll_angle_ki;
extern float cascade_roll_angle_kd;
extern float cascade_roll_rate_kp;
extern float cascade_roll_rate_ki;
extern float cascade_roll_rate_kd;
extern float cascade_rate_setpoint_pitch;
extern float cascade_rate_setpoint_roll;
extern float cascade_pitch_output;
extern float cascade_roll_output;
void resetCascadePIDToDefaults();
void resetCascadePID();
#endif

// ===== Print Welcome Banner =====
void printWelcomeBanner() {
  Serial.println("\n========================================");
  Serial.println("Balancing Robot v1.0 - MPU6050 Protected");
  Serial.println("========================================");
  Serial.println("\n[SAFETY] Drone cannot arm without MPU6050 initialization");
  Serial.println("         Use 'retry_mpu' command if sensor fails to connect");
  Serial.println("\nAvailable Commands:");
  Serial.println("  help           - Display this help message");
  Serial.println("  arm            - Enable motors");
  Serial.println("  disarm         - Disable motors");
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
  Serial.println("  roll_rate/rr <val>  - Set roll rate target (deg/s)");
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
  Serial.println("  reset_pid      - Reset PID to defaults");
  Serial.println("  reset_calibration - Reset calibration to defaults");
  Serial.println("  battery_reset/bat_reset - Reset low voltage warning");
  Serial.println("  i2c_scan       - Scan I2C bus for connected devices");
  Serial.println("  test_motor     - Spin each motor individually at 20% throttle for pinout scan:");
  Serial.println("                   Motor FL(33)→FR(27)→RR(26)→RL(25), 2s each, auto-stops after loop");
  Serial.println("  status         - Toggle sensor monitoring");
  Serial.println("  debug          - Toggle debug information");
  Serial.println("\nWiFi & OTA Commands:");
  Serial.println("  wifi/wifi_status - Show WiFi status");
  Serial.println("  wifi_sta/sta   - Switch to STA mode (connect to WiFi)");
  Serial.println("  wifi_ap/ap     - Switch to AP mode (create WiFi hotspot)");
  Serial.println("  (OTA ready: Arduino IDE > Tools > Port > Network Ports)");
  Serial.println("\nType a command and press Enter:");
  Serial.println("========================================\n");
}

// ===== Handle Serial Commands =====
void handleSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
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
      // Check if drone is level using hysteresis (safeToArm flag)
      else if (!safeToArm) {
        Serial.print("\n[WARN] ARM BLOCKED: Drone must be level! Pitch: "); Serial.print(filtered_pitch, 1);
        Serial.print("° Roll: "); Serial.print(filtered_roll, 1);
        Serial.println("° (Must be < 40° for 200ms to allow arming)");
      } else {
        motorsArmed = true;
        motorsActive = true;
        throttle = MIN_THROTTLE;  // Set to minimum throttle on arm
        Serial.println("\n[INFO] Balancing Robot ARMED - Motors ready");
        pidIntegral = 0;
      }
    } 
    else if (command == "disarm") {
      motorsArmed = false;
      motorsActive = false;
      throttle = 0.0f;  // Reset throttle on disarm
      pidIntegral = 0.0;  // Reset integral on disarm
      stopMotors();
      Serial.println("\n[INFO] Balancing Robot DISARMED - Motors stopped");
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
    else if (command.startsWith("set p ")) {
      float value = command.substring(6).toFloat();
      KP = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PID] KP updated to: "); Serial.println(KP);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("set i ")) {
      float value = command.substring(6).toFloat();
      KI = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PID] KI updated to: "); Serial.println(KI);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("set d ")) {
      float value = command.substring(6).toFloat();
      KD = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PID] KD updated to: "); Serial.println(KD);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // PITCH AXIS TUNING (separate)
    else if (command.startsWith("pitch_p ") || command.startsWith("pp ")) {
      float value = (command.indexOf("pitch_p") >= 0) ? command.substring(8).toFloat() : command.substring(3).toFloat();
      KP_Pitch = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PITCH PID] KP_Pitch updated to: "); Serial.println(KP_Pitch);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("pitch_i ") || command.startsWith("pi ")) {
      float value = (command.indexOf("pitch_i") >= 0) ? command.substring(8).toFloat() : command.substring(3).toFloat();
      KI_Pitch = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PITCH PID] KI_Pitch updated to: "); Serial.println(KI_Pitch);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("pitch_d ") || command.startsWith("pd ")) {
      float value = (command.indexOf("pitch_d") >= 0) ? command.substring(8).toFloat() : command.substring(3).toFloat();
      KD_Pitch = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[PITCH PID] KD_Pitch updated to: "); Serial.println(KD_Pitch);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // ROLL AXIS TUNING (separate)
    else if (command.startsWith("roll_p ") || command.startsWith("rp ")) {
      float value = (command.indexOf("roll_p") >= 0) ? command.substring(7).toFloat() : command.substring(3).toFloat();
      KP_Roll = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[ROLL PID] KP_Roll updated to: "); Serial.println(KP_Roll);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("roll_i ") || command.startsWith("ri ")) {
      float value = (command.indexOf("roll_i") >= 0) ? command.substring(7).toFloat() : command.substring(3).toFloat();
      KI_Roll = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[ROLL PID] KI_Roll updated to: "); Serial.println(KI_Roll);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("roll_d ") || command.startsWith("rd ")) {
      float value = (command.indexOf("roll_d") >= 0) ? command.substring(7).toFloat() : command.substring(3).toFloat();
      KD_Roll = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[ROLL PID] KD_Roll updated to: "); Serial.println(KD_Roll);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command == "reset_pid") {
      resetPIDToDefaults();
    }
    else if (command == "reset_calibration") {
      resetCalibrationToDefaults();
    }
    // YAW AXIS Tuning (Short form: yp, yi, yd)
    else if (command.startsWith("yaw_p ") || command.startsWith("yp ")) {
      float value = (command.indexOf("yaw_p") >= 0) ? command.substring(6).toFloat() : command.substring(3).toFloat();
      KP_Yaw = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[YAW PID] KP_Yaw updated to: "); Serial.println(KP_Yaw);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("yaw_i ") || command.startsWith("yi ")) {
      float value = (command.indexOf("yaw_i") >= 0) ? command.substring(6).toFloat() : command.substring(3).toFloat();
      KI_Yaw = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[YAW PID] KI_Yaw updated to: "); Serial.println(KI_Yaw);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("yaw_d ") || command.startsWith("yd ")) {
      float value = (command.indexOf("yaw_d") >= 0) ? command.substring(6).toFloat() : command.substring(3).toFloat();
      KD_Yaw = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[YAW PID] KD_Yaw updated to: "); Serial.println(KD_Yaw);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("yaw_rate ") || command.startsWith("yr ")) {
      float value = (command.indexOf("yaw_rate") >= 0) ? command.substring(9).toFloat() : command.substring(3).toFloat();
      yaw_rate_target = value;
      Serial.print("\n[YAW RATE] Target Rate set to: "); Serial.print(yaw_rate_target);
      Serial.println(" deg/s (Rate-Only Control)");
    }
    #if ENABLE_CASCADE_PID && !CASCADE_MODE_ANGLE_CONTROL
    // Rate mode commands (only available in cascade rate mode)
    else if (command.startsWith("pitch_rate ") || command.startsWith("pr ")) {
      float value = (command.indexOf("pitch_rate") >= 0) ? command.substring(11).toFloat() : command.substring(3).toFloat();
      pitch_rate_target = value;
      Serial.print("\n[CASCADE PITCH RATE] Target Rate set to: "); Serial.print(pitch_rate_target);
      Serial.println(" deg/s");
    }
    else if (command.startsWith("roll_rate ") || command.startsWith("rr ")) {
      float value = (command.indexOf("roll_rate") >= 0) ? command.substring(10).toFloat() : command.substring(3).toFloat();
      roll_rate_target = value;
      Serial.print("\n[CASCADE ROLL RATE] Target Rate set to: "); Serial.print(roll_rate_target);
      Serial.println(" deg/s");
    }
    #endif
    else if (command == "load") {
      lastPIDUpdateTime = millis();
      Serial.println("\n===== Current Dual-Axis PID Values =====");
      Serial.println("PITCH AXIS:");
      Serial.print("  KP_Pitch: "); Serial.println(KP_Pitch);
      Serial.print("  KI_Pitch: "); Serial.println(KI_Pitch);
      Serial.print("  KD_Pitch: "); Serial.println(KD_Pitch);
      Serial.println("ROLL AXIS:");
      Serial.print("  KP_Roll: "); Serial.println(KP_Roll);
      Serial.print("  KI_Roll: "); Serial.println(KI_Roll);
      Serial.print("  KD_Roll: "); Serial.println(KD_Roll);
      Serial.println("YAW AXIS (Rate-Only Control):");
      Serial.print("  KP_Yaw: "); Serial.println(KP_Yaw);
      Serial.print("  KI_Yaw: "); Serial.println(KI_Yaw);
      Serial.print("  KD_Yaw: "); Serial.println(KD_Yaw);
      #if ENABLE_CASCADE_PID
      Serial.println("\n*** CASCADE PID MODE  ***");
      #if CASCADE_MODE_ANGLE_CONTROL
      Serial.println("Current Mode: ANGLE CONTROL");
      #else
      Serial.println("Current Mode: RATE CONTROL");
      Serial.println("RATE TARGETS:");
      Serial.print("  pitch_rate_target: "); Serial.println(pitch_rate_target);
      Serial.print("  roll_rate_target: "); Serial.println(roll_rate_target);
      #endif
      Serial.println("PITCH AXIS - ANGLE PID (Outer Loop):");
      Serial.print("  cascade_pitch_angle_kp: "); Serial.println(cascade_pitch_angle_kp);
      Serial.print("  cascade_pitch_angle_ki: "); Serial.println(cascade_pitch_angle_ki);
      Serial.print("  cascade_pitch_angle_kd: "); Serial.println(cascade_pitch_angle_kd);
      Serial.println("PITCH AXIS - RATE PID (Inner Loop):");
      Serial.print("  cascade_pitch_rate_kp: "); Serial.println(cascade_pitch_rate_kp);
      Serial.print("  cascade_pitch_rate_ki: "); Serial.println(cascade_pitch_rate_ki);
      Serial.print("  cascade_pitch_rate_kd: "); Serial.println(cascade_pitch_rate_kd);
      Serial.println("ROLL AXIS - ANGLE PID (Outer Loop):");
      Serial.print("  cascade_roll_angle_kp: "); Serial.println(cascade_roll_angle_kp);
      Serial.print("  cascade_roll_angle_ki: "); Serial.println(cascade_roll_angle_ki);
      Serial.print("  cascade_roll_angle_kd: "); Serial.println(cascade_roll_angle_kd);
      Serial.println("ROLL AXIS - RATE PID (Inner Loop):");
      Serial.print("  cascade_roll_rate_kp: "); Serial.println(cascade_roll_rate_kp);
      Serial.print("  cascade_roll_rate_ki: "); Serial.println(cascade_roll_rate_ki);
      Serial.print("  cascade_roll_rate_kd: "); Serial.println(cascade_roll_rate_kd);
      Serial.println("(Values persisted in NVS)");
      #endif
      Serial.println("=========================================\n");
      Serial.println("Telemetry paused for 5 seconds...");
    }
    
#if ENABLE_CASCADE_PID
    // ===== CASCADE PID TUNING COMMANDS - PITCH AXIS =====
    // Cascade Angle PID (Outer Loop) - PITCH
    else if (command.startsWith("app ")) {
      String valueStr = command.substring(4);
      valueStr.trim();  // Remove any leading/trailing whitespace
      float value = valueStr.toFloat();
      if (value == 0.0f && valueStr != "0" && valueStr != "0.0") {
        Serial.println("[ERROR] Failed to parse value: " + valueStr);
        Serial.println("[HELP] Correct format: app <float_value>  (e.g., app 60.0)");
      } else {
        cascade_pitch_angle_kp = value;
        lastPIDUpdateTime = millis();
        savePIDToPreferences();
        Serial.print("[CASCADE-PITCH] Angle KP updated to: "); Serial.println(cascade_pitch_angle_kp);
      }
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("api ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_angle_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Angle KI updated to: "); Serial.println(cascade_pitch_angle_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("apd ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_angle_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Angle KD updated to: "); Serial.println(cascade_pitch_angle_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // Cascade Rate PID (Inner Loop) - PITCH
    else if (command.startsWith("rpp ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_rate_kp = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Rate KP updated to: "); Serial.println(cascade_pitch_rate_kp);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rpi ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_rate_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Rate KI updated to: "); Serial.println(cascade_pitch_rate_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rpd ")) {
      float value = command.substring(4).toFloat();
      cascade_pitch_rate_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-PITCH] Rate KD updated to: "); Serial.println(cascade_pitch_rate_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    
    // ===== CASCADE PID TUNING COMMANDS - ROLL AXIS =====
    // Cascade Angle PID (Outer Loop) - ROLL
    else if (command.startsWith("arp ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_angle_kp = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Angle KP updated to: "); Serial.println(cascade_roll_angle_kp);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("ari ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_angle_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Angle KI updated to: "); Serial.println(cascade_roll_angle_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("ard ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_angle_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Angle KD updated to: "); Serial.println(cascade_roll_angle_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    // Cascade Rate PID (Inner Loop) - ROLL
    else if (command.startsWith("rrp ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_rate_kp = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Rate KP updated to: "); Serial.println(cascade_roll_rate_kp);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rri ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_rate_ki = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Rate KI updated to: "); Serial.println(cascade_roll_rate_ki);
      Serial.println("Telemetry paused for 5 seconds...");
    }
    else if (command.startsWith("rrd ")) {
      float value = command.substring(4).toFloat();
      cascade_roll_rate_kd = value;
      lastPIDUpdateTime = millis();
      savePIDToPreferences();
      Serial.print("\n[CASCADE-ROLL] Rate KD updated to: "); Serial.println(cascade_roll_rate_kd);
      Serial.println("Telemetry paused for 5 seconds...");
    }

#endif
    
    // WiFi & OTA Commands
    else if (command == "wifi_status" || command == "wifi") {
      printWiFiStatus();
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

#endif
