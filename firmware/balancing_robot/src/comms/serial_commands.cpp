#include "../config/settings.h"  // FIRST: macros used by serial_commands.h (#if guards)
#include "serial_commands.h"
#include "../control/motor_control.h"  // stopMotors()
#include "../control/pid_controller.h"  // speed setpoints
#include "../control/cascade_pid_controller.h"  // cascade externs (when enabled)
#include "wifi_ota.h"  // switchWiFiMode(), printWiFiStatus()
#include "websocket_handler.h"  // printStateJsonSerial() — shared state JSON for WebUI
#include "../system/led.h"  // ledBootTest(), ledSet(), ledSetRGB()
#include "../config/pins_live.h"  // pins show / pin set / pins save|reset (NVS)
#include "../system/imu.h"  // mpu live-driver pointer ('debug imu') + printImuStatus()
#include "wifi_creds.h"  // wifi show / wifi set / wifi save|reset (NVS, passwords masked)
#include "comms_mode.h"  // comms show / set / save|reset (NVS link selection)

// Definitions live here (were in balancing_robot.ino); externs in serial_commands.h
float accel_z_world_mps2 = 0.0f;
bool statusMonitoring = false;
bool debugImuMonitoring = false;  // 'debug imu' raw-sensor stream (status-style toggle)
bool debugLedMonitoring = false;  // 'debug led' RGB cycle (status-style toggle)
static bool motorConfirmPending = false;
static unsigned long motorConfirmDeadline = 0;

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
  Serial.println("  pins show      - List board pins + copy-paste edit line (live values + NVS/defaults source)");
  Serial.println("  pin set <N> <gpio> - Single: pin set ENA 5 (batch: pin set ENA=5 IN1=6 ...; paste the pins-show edit line)");
  Serial.println("  pins save      - Re-persist pins to NVS (rarely needed, pin set auto-saves)");
  Serial.println("  pins reset     - Clear pin overrides, restore board defaults (reboot to apply)");
  Serial.println("  setup          - Bring-up wizard: pins->IMU->motor->LED->link->WiFi->cal (Enter skips)");
  Serial.println("  abort setup    - Cancel the setup wizard (quit/exit work too)");
  Serial.println("  comms show     - Show link selection (ws/espnow, peer MAC, sta/ap)");
  Serial.println("  comms set <mode|mac|wifimode> <value> - Stage link selection");
  Serial.println("  comms save     - Persist link selection to NVS (reboot to apply)");
  Serial.println("  comms reset    - Clear link overrides back to defaults");
  Serial.println("  reset_pid      - Reset PID to defaults");
  Serial.println("  reset_calibration - Reset calibration to defaults");
  Serial.println("  battery_reset/bat_reset - Reset low voltage warning");
  Serial.println("  i2c_scan       - Scan I2C bus for connected devices");
  Serial.println("  status         - Toggle sensor monitoring");
  Serial.println("  debug motor    - 100% wheel test (warns, needs 'debug motor yes' confirm)");
  Serial.println("  debug imu      - Toggle raw accel/gyro stream");
  Serial.println("  debug led      - Toggle RGB cycle (link LED resumes after)");
  Serial.println("  debug nvs      - Dump raw NVS namespaces (pins/wifi/calib keys)");
  Serial.println("  debug ws       - Toggle WebSocket message log (default OFF)");
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
// Raw NVS dump: shows exactly what survives reboot per namespace, so a
// stale override (forgotten pin/wifi/calib value) is visible instead of
// silently changing behavior. Passwords are never printed.
static void printNvsDebugToSerial() {
  Serial.println("--- nvs (raw) ---");
  {
    Preferences p;
    bool ok = p.begin("board_pins", true);
    Serial.printf("board_pins : %s\n", ok ? "present" : "absent (defaults)");
    if (ok) {
      const char* keys[] = {"ENA","IN1","IN2","ENB","IN3","IN4","SDA","SCL",
                            "BAT","LED","SCK","MOSI","MISO","CS"};
      for (unsigned i = 0; i < sizeof(keys)/sizeof(keys[0]); i++) {
        if (p.isKey(keys[i])) Serial.printf("  %s = %d\n", keys[i], p.getInt(keys[i]));
        else Serial.printf("  %s = <unset>\n", keys[i]);
      }
      p.end();
    }
  }
  {
    Preferences p;
    bool ok = p.begin("wifi_cfg", true);
    Serial.printf("wifi_cfg   : %s\n", ok ? "present" : "absent (defaults)");
    if (ok) {
      Serial.printf("  ssid = %s\n", p.isKey("ssid") ? p.getString("ssid").c_str() : "<unset>");
      Serial.printf("  pass = %s\n", p.isKey("pass") ? "********" : "<unset>");
      Serial.printf("  ap_ssid = %s\n", p.isKey("ap_ssid") ? p.getString("ap_ssid").c_str() : "<unset>");
      Serial.printf("  ap_pass = %s\n", p.isKey("ap_pass") ? "********" : "<unset>");
      p.end();
    }
  }
  {
    Preferences p;
    bool ok = p.begin("mpu6050", true);
    Serial.printf("mpu6050    : %s\n", ok ? "present" : "absent (defaults)");
    if (ok) {
      const char* keys[] = {"trim_pitch","trim_roll","gyroBiasX","gyroBiasY","gyroBiasZ",
                            "axBias","axScale","ayBias","ayScale","azBias","azScale",
                            "baroScale","altAzBias"};
      for (unsigned i = 0; i < sizeof(keys)/sizeof(keys[0]); i++) {
        if (p.isKey(keys[i])) Serial.printf("  %s = %.4f\n", keys[i], p.getFloat(keys[i]));
        else Serial.printf("  %s = <unset>\n", keys[i]);
      }
      p.end();
    }
  }
}

// ===== Interactive 'setup' wizard (see handleSerialCommand routing) =====
// Step-by-step bring-up: pins -> IMU check -> motor check -> LED check ->
// link selection -> WiFi creds -> calibration. Empty line (Enter) keeps the
// current value or skips the step; 'abort setup' (or quit/exit) aborts
// (staging already saved stays). Prompts are full lines with a blank line
// between steps so serial-monitor output stays readable.
enum SetupStep : uint8_t {
  ST_PINS, ST_IMU_ASK, ST_MOTOR_ASK, ST_MOTOR_WAIT, ST_MOTOR_OK,
  ST_LED_ASK, ST_COMMS_LINK, ST_COMMS_MAC, ST_COMMS_WIFI,
  ST_WIFI_SSID, ST_WIFI_PASS, ST_CAL_GYRO, ST_CAL_ACCEL, ST_CAL_RUN, ST_DONE
};
static bool setupActive = false;
static SetupStep setupStep = ST_PINS;
static String setupTmp = "";
static bool setupCalGyroDone = false;
static CalibrationState setupLastCalib = CALIB_IDLE;

// ===== 'pin set' single + batch (shared by CLI and setup wizard) =====
// Accepted: pin set ENA 5 | pin set ENA=5 | batch: pin set ENA=5 IN1=6 ...
// Paste-friendly: the whole 'pins show' block can follow 'pin set ' —
// unknown words (motors/i2c/misc/spi/source/labels) are skipped, only
// known NAME + value pairs are applied. All-or-nothing: any reject rolls
// back every pin in the line and nothing is saved.
static bool isBatchPinName(const String& t) {
  return t == "ena" || t == "in1" || t == "in2" || t == "enb" || t == "in3" ||
         t == "in4" || t == "sda" || t == "scl" || t == "bat" || t == "led" ||
         t == "sck" || t == "mosi" || t == "miso" || t == "cs";
}

static bool isBatchInt(const String& s, int& out) {
  if (s.length() == 0 || s.length() > 4) return false;
  int i = 0;
  if (s.charAt(0) == '-') { if (s.length() == 1) return false; i = 1; }
  for (; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c < '0' || c > '9') return false;
  }
  out = s.toInt();
  return true;
}

static void handlePinSetArgs(const String& args, const char* tag) {
  String norm = args;
  norm.replace("=", " ");
  norm.replace(":", " ");
  norm.replace(",", " ");
  norm.replace(";", " ");
  String toks[30];
  uint8_t tc = 0;
  int L = norm.length();
  int i = 0;
  while (i < L && tc < 30) {
    while (i < L && norm.charAt(i) == ' ') i++;
    if (i >= L) break;
    int j = i;
    while (j < L && norm.charAt(j) != ' ') j++;
    toks[tc++] = norm.substring(i, j);
    i = j;
  }
  struct PinEdit { String name; int gpio; };
  PinEdit edits[14];
  uint8_t ec = 0;
  for (uint8_t k = 0; k < tc && ec < 14; k++) {
    String t = toks[k];
    t.toLowerCase();
    if (!isBatchPinName(t)) continue;  // skip labels, source line, stray numbers
    if (k + 1 >= tc) {
      Serial.printf("[%s] rejected: %s needs a value (usage: pin set %s <gpio>)\n\n", tag, t.c_str(), t.c_str());
      return;
    }
    int gpio = 0;
    if (!isBatchInt(toks[k + 1], gpio)) {
      Serial.printf("[%s] rejected: %s has no numeric value after it\n\n", tag, t.c_str());
      return;
    }
    edits[ec++] = {t, gpio};
    k++;  // consume value
  }
  if (ec == 0) {
    Serial.println("[PINS] usage: pin set <NAME> <gpio>  (batch: pin set ENA=5 IN1=6 ...)");
    Serial.println();
    return;
  }
  int snap[] = {g_pin_ENA, g_pin_IN1, g_pin_IN2, g_pin_ENB, g_pin_IN3, g_pin_IN4,
                g_pin_SDA, g_pin_SCL, g_pin_BAT, g_pin_LED,
                g_pin_SPI_SCK, g_pin_SPI_MOSI, g_pin_SPI_MISO, g_pin_SPI_CS};
  for (uint8_t e = 0; e < ec; e++) {
    String perr;
    if (!setStagedPin(edits[e].name, edits[e].gpio, perr)) {
      g_pin_ENA = snap[0]; g_pin_IN1 = snap[1]; g_pin_IN2 = snap[2];
      g_pin_ENB = snap[3]; g_pin_IN3 = snap[4]; g_pin_IN4 = snap[5];
      g_pin_SDA = snap[6]; g_pin_SCL = snap[7]; g_pin_BAT = snap[8];
      g_pin_LED = snap[9]; g_pin_SPI_SCK = snap[10]; g_pin_SPI_MOSI = snap[11];
      g_pin_SPI_MISO = snap[12]; g_pin_SPI_CS = snap[13];
      Serial.print("[");
      Serial.print(tag);
      Serial.print("] rejected: ");
      Serial.println(perr);
      Serial.println();
      return;
    }
  }
  savePinsToNVS();
  for (uint8_t e = 0; e < ec; e++) {
    Serial.printf("[%s] %s = %d\n", tag, edits[e].name.c_str(), edits[e].gpio);
  }
  Serial.println("[PINS] saved - reboot to apply");
  Serial.println();
}

static void setupShowPins() {
  printPinsToSerial();
  Serial.println("[SETUP] edit: pin set NAME gpio  (single) or paste the edit line with new values (batch).");
  Serial.println("[SETUP] 'pins show' reprints, Enter = done, 'abort setup' cancels.");
  Serial.println();
}

// IDE 2.x serial monitor ignores ANSI clear codes, so scroll the previous
// step out of view with blank lines (scrollback kept, nothing lost).
static void setupClear() {
  for (uint8_t i = 0; i < 50; i++) Serial.println();
}

// User-facing stage header, printed after every Enter — same idea as the
// calibration pose labels (pose 1/6 ...), so the current step is always visible.
static void setupStepHeader() {
  const char* label = "";
  switch (setupStep) {
    case ST_PINS: label = "Step 1/7: Pins"; break;
    case ST_IMU_ASK: label = "Step 2/7: IMU check"; break;
    case ST_MOTOR_ASK:
    case ST_MOTOR_WAIT:
    case ST_MOTOR_OK: label = "Step 3/7: Motor test"; break;
    case ST_LED_ASK: label = "Step 4/7: LED test"; break;
    case ST_COMMS_LINK:
    case ST_COMMS_MAC:
    case ST_COMMS_WIFI: label = "Step 5/7: Link selection"; break;
    case ST_WIFI_SSID:
    case ST_WIFI_PASS: label = "Step 6/7: WiFi"; break;
    case ST_CAL_GYRO:
    case ST_CAL_ACCEL:
    case ST_CAL_RUN: label = "Step 7/7: Calibration"; break;
    case ST_DONE: label = "Step 7/7: Done"; break;
  }
  Serial.print("[SETUP] --- ");
  Serial.print(label);
  Serial.println(" ---");
  Serial.println();
}

static const char* setupCalibPose(CalibrationState s) {
  switch (s) {
    case CALIB_GYRO: return "hold STILL";
    case CALIB_ACCEL_STEP1: return "pose 1/6 LEVEL still";
    case CALIB_ACCEL_STEP2: return "pose 2/6 nose DOWN +90deg";
    case CALIB_ACCEL_STEP3: return "pose 3/6 nose UP -90deg";
    case CALIB_ACCEL_STEP4: return "pose 4/6 tilt RIGHT +90deg";
    case CALIB_ACCEL_STEP5: return "pose 5/6 tilt LEFT -90deg";
    case CALIB_ACCEL_STEP6: return "pose 6/6 UPSIDE DOWN";
    default: return "";
  }
}

static void setupAbort() {
  debugImuMonitoring = false;
  debugLedMonitoring = false;
  if (testMotorActive) toggleMotorTest();
  setupActive = false;
  Serial.println("\n[SETUP] aborted (already-saved values stay)");
}

static void setupWizardHandle(const String& command, const String& raw) {
  // 'abort setup' cancels from any step; bare 'abort' too, except mid-calibration
  // where it only cancels the calibration run (see ST_CAL_RUN).
  if (command == "quit" || command == "exit" || command == "abort setup") { setupClear(); setupAbort(); return; }
  if (command == "abort" && setupStep != ST_CAL_RUN) { setupClear(); setupAbort(); return; }
  setupClear();
  setupStepHeader();

  switch (setupStep) {
    case ST_PINS: {
      if (command.length() == 0) {
        savePinsToNVS();
        Serial.println("\n[SETUP] pins done. Checking IMU - raw stream on, wiggle the board.");
        Serial.println("[SETUP] numbers move with motion? (Enter continues)");
        debugImuMonitoring = true;
        setupStep = ST_IMU_ASK;
      } else if (command == "pins show") {
        setupShowPins();
      } else if (command.startsWith("pin set ") || command.startsWith("pins set ")) {
        String args = command.startsWith("pin set ")
            ? command.substring(8) : command.substring(9);
        handlePinSetArgs(args, "SETUP");
        Serial.println("[SETUP] more edits, 'pins show' reprints, or Enter = done.");
        Serial.println();
      } else {
        Serial.println("[SETUP] pins step: 'pin set NAME gpio', batch edit, 'pins show', or Enter = done.");
        Serial.println();
      }
      return;
    }
    case ST_IMU_ASK:
      debugImuMonitoring = false;
      Serial.println("\n[SETUP] 100% wheel test? Prop the robot UP, wheels free. (yes / Enter skips)");
      setupStep = ST_MOTOR_ASK;
      return;
    case ST_MOTOR_ASK:
      if (command == "yes" || command == "y") {
        if (motorsArmed) {
          Serial.println("[SETUP] armed - 'disarm' first, then re-run 'setup'. Aborting.");
          setupAbort();
          return;
        }
        Serial.println("[SETUP] spinning LEFT fwd/rev then RIGHT fwd/rev at 100%. Enter to check.");
        toggleMotorTest();
        setupStep = ST_MOTOR_WAIT;
      } else {
        Serial.println("\n[SETUP] motor skipped. Cycling LED now - watch it. (Enter stops + continues)");
        debugLedMonitoring = true;
        setupStep = ST_LED_ASK;
      }
      return;
    case ST_MOTOR_WAIT:
      if (testMotorActive) {
        Serial.println("[SETUP] still spinning... Enter to check.");
      } else {
        Serial.println("\n[SETUP] test done. Both wheels spun both ways? (Enter continues)");
        setupStep = ST_MOTOR_OK;
      }
      return;
    case ST_MOTOR_OK:
      Serial.println("\n[SETUP] cycling LED now - watch it. (Enter stops + continues)");
      debugLedMonitoring = true;
      setupStep = ST_LED_ASK;
      return;
    case ST_LED_ASK:
      debugLedMonitoring = false;
      Serial.println("\n[SETUP] link? (ws = WebUI, espnow = controller. Enter keeps current)");
      setupStep = ST_COMMS_LINK;
      return;
    case ST_COMMS_LINK: {
      String v = command;
      if (v.length() == 0) {
        Serial.printf("\n[SETUP] keeping %s. WiFi mode? (sta / ap. Enter keeps %s)\n",
                      g_comms_espnow ? "espnow" : "ws", g_comms_ap ? "ap" : "sta");
        setupStep = ST_COMMS_WIFI;
        return;
      }
      String cerr;
      if (!setStagedComms("mode", v, cerr)) {
        Serial.print("[SETUP] rejected: ");
        Serial.println(cerr);
        Serial.println("[SETUP] link? (ws / espnow. Enter keeps current)");
        return;
      }
      saveCommsToNVS();
      Serial.printf("[SETUP] link = %s (saved)\n\n", g_comms_espnow ? "espnow" : "ws");
      if (g_comms_espnow) {
        char mac[18];
        snprintf(mac, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                 g_peer_mac[0], g_peer_mac[1], g_peer_mac[2],
                 g_peer_mac[3], g_peer_mac[4], g_peer_mac[5]);
        Serial.printf("[SETUP] controller MAC? [%s] (Enter keeps)\n", mac);
        setupStep = ST_COMMS_MAC;
      } else {
        Serial.printf("[SETUP] WiFi mode? (sta / ap. Enter keeps %s)\n", g_comms_ap ? "ap" : "sta");
        setupStep = ST_COMMS_WIFI;
      }
      return;
    }
    case ST_COMMS_MAC: {
      if (command.length() > 0) {
        String cerr;
        if (!setStagedComms("mac", command, cerr)) {
          Serial.print("[SETUP] rejected: ");
          Serial.println(cerr);
          Serial.println("[SETUP] controller MAC? (AA:BB:CC:DD:EE:FF, Enter keeps)");
          return;
        }
        saveCommsToNVS();
        Serial.print("[SETUP] controller MAC = ");
        Serial.print(command);
        Serial.println(" (saved)\n");
      } else {
        Serial.println("[SETUP] controller MAC kept\n");
      }
      Serial.println("[SETUP] link saved. Gyro cal? Needs a STILL robot. (yes / Enter skips)");
      setupStep = ST_CAL_GYRO;
      return;
    }
    case ST_COMMS_WIFI: {
      if (command.length() > 0) {
        String cerr;
        if (!setStagedComms("wifimode", command, cerr)) {
          Serial.print("[SETUP] rejected: ");
          Serial.println(cerr);
          Serial.println("[SETUP] WiFi mode? (sta / ap. Enter keeps)");
          return;
        }
        saveCommsToNVS();
        Serial.printf("[SETUP] wifi mode = %s (saved)\n\n", g_comms_ap ? "ap" : "sta");
      } else {
        Serial.printf("[SETUP] wifi mode kept (%s)\n\n", g_comms_ap ? "ap" : "sta");
      }
      if (g_comms_ap) {
        Serial.print("[SETUP] hotspot name? [");
        Serial.print(g_ap_ssid.c_str());
        Serial.println("] (Enter keeps)");
      } else {
        Serial.println("[SETUP] WiFi name? (Enter skips WiFi creds)");
      }
      setupTmp = "";
      setupStep = ST_WIFI_SSID;
      return;
    }
    case ST_WIFI_SSID: {
      if (command.length() == 0) {
        if (!g_comms_ap) {
          Serial.println("\n[SETUP] WiFi skipped. Gyro cal? Needs a STILL robot. (yes / Enter skips)");
          setupStep = ST_CAL_GYRO;
        } else {
          Serial.println("\n[SETUP] hotspot kept. Gyro cal? Needs a STILL robot. (yes / Enter skips)");
          setupStep = ST_CAL_GYRO;
        }
        return;
      }
      setupTmp = raw;  // keep case for SSID
      Serial.print("[SETUP] name staged: ");
      Serial.println(setupTmp);
      Serial.println("[SETUP] password? (Enter keeps current)");
      setupStep = ST_WIFI_PASS;
      return;
    }
    case ST_WIFI_PASS: {
      String err;
      const char* key = g_comms_ap ? "ap_ssid" : "ssid";
      const char* pkey = g_comms_ap ? "ap_pass" : "pass";
      if (!setStagedWifi(key, setupTmp, err) ||
          (command.length() > 0 && !setStagedWifi(pkey, raw, err))) {
        Serial.print("[SETUP] rejected: ");
        Serial.println(err);
      } else {
        saveWifiToNVS();
        Serial.println("[SETUP] WiFi saved (password hidden).\n");
      }
      Serial.println("[SETUP] gyro cal? Needs a STILL robot. (yes / Enter skips)");
      setupStep = ST_CAL_GYRO;
      return;
    }
    case ST_CAL_GYRO:
      if (command == "yes" || command == "y") {
        setupCalGyroDone = false;
        setupLastCalib = CALIB_IDLE;
        startGyroCalibration();
        Serial.println("[SETUP] collecting gyro... Enter to check.");
        setupStep = ST_CAL_RUN;
      } else {
        debugImuMonitoring = false;
        debugLedMonitoring = false;
        Serial.println("\n[SETUP] done, calibration skipped. Reboot to apply all? (yes / Enter reboots)");
        setupStep = ST_DONE;
      }
      return;
    case ST_CAL_ACCEL:
      if (command == "yes" || command == "y") {
        setupLastCalib = CALIB_IDLE;
        startAccelCalibration();
        Serial.println("[SETUP] pose 1/6 LEVEL still, then type save. (abort cancels)");
        setupStep = ST_CAL_RUN;
      } else {
        debugImuMonitoring = false;
        debugLedMonitoring = false;
        Serial.println("\n[SETUP] done. Reboot to apply all? (yes / Enter reboots)");
        setupStep = ST_DONE;
      }
      return;
    case ST_CAL_RUN: {
      if (command == "abort") {
        processCalibrationStep(command);
        debugImuMonitoring = false;
        debugLedMonitoring = false;
        Serial.println("[SETUP] calibration aborted. Reboot to apply the rest? (yes / Enter reboots)");
        setupStep = ST_DONE;
        return;
      }
      if (command == "save") processCalibrationStep(command);
      if (calibrationState == CALIB_IDLE) {
        if (!setupCalGyroDone) {
          setupCalGyroDone = true;
          Serial.println("\n[SETUP] gyro done. Accel 6-pose cal? (yes / Enter skips)");
          setupStep = ST_CAL_ACCEL;
        } else {
          debugImuMonitoring = false;
          debugLedMonitoring = false;
          Serial.println("\n[SETUP] all calibrated. Reboot to apply all? (yes / Enter reboots)");
          setupStep = ST_DONE;
        }
      } else {
        if (calibrationState != setupLastCalib) {
          setupLastCalib = calibrationState;
          Serial.printf("[SETUP] %s, then type save. (abort cancels)\n", setupCalibPose(calibrationState));
        } else if (calibrationState == CALIB_GYRO) {
          Serial.println("[SETUP] still collecting... Enter to check.");
        }
      }
      return;
    }
    case ST_DONE:
      if (command == "yes" || command == "y" || command.length() == 0) {
        setupActive = false;
        stopMotors();
        Serial.println("\n[SETUP] rebooting...");
        delay(100);
        ESP.restart();
      } else {
        Serial.println("[SETUP] (yes / Enter reboots, 'abort setup' cancels)");
      }
      return;
  }
}

void handleSerialCommand() {
  if (Serial.available() > 0) {
    String raw = Serial.readStringUntil('\n');
    raw.trim();
    String command = raw;  // lowercased dispatch copy; raw keeps case for wifi creds
    command.trim();
    command.toLowerCase();

    // Interactive setup wizard consumes all lines while active
    if (setupActive) {
      setupWizardHandle(command, raw);
      return;
    }

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
    else if (command == "arm") {
      // Check MPU6050 initialization
      if (testMotorActive) {
        Serial.println("\n[WARN] ARM BLOCKED: Motor test running. Type 'debug motor' to stop.");
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
        Serial.println("\n[INFO] Status monitoring DISABLED");
      }
    }
    else if (command == "setup") {
      if (motorsArmed) {
        Serial.println("[SETUP] disarm first ('disarm'), then run 'setup'");
      } else {
        setupActive = true;
        setupStep = ST_PINS;
        setupTmp = "";
        setupCalGyroDone = false;
        setupLastCalib = CALIB_IDLE;
        setupClear();
        Serial.println("\n[SETUP] bring-up wizard: pins -> IMU -> motor -> LED -> link -> WiFi -> cal.");
        Serial.println("[SETUP] Enter = keep/skip, 'abort setup' cancels anytime.");
        Serial.println();
        setupStepHeader();
        setupShowPins();
      }
    }
    else if (command == "abort setup") {
      Serial.println("[SETUP] no wizard running (use 'setup' to start)");
    }
    else if (command == "debug motor") {
      if (testMotorActive) {
        toggleMotorTest();  // stop — no confirm needed to stop
      } else if (motorsArmed) {
        motorConfirmPending = false;
        Serial.println("\n[REFUSED] Disarm first (type 'disarm'), then 'debug motor'");
      } else {
        motorConfirmPending = true;
        motorConfirmDeadline = millis() + 15000;
        Serial.println("\n[DANGER] Wheel test drives BOTH motors at 100% PWM:");
        Serial.println("         LEFT fwd/rev then RIGHT fwd/rev, 2s each.");
        Serial.println("         Prop the robot UP so wheels spin freely, keep clear.");
        Serial.println("         Type 'debug motor yes' within 15s to proceed.");
      }
    }
    else if (command == "debug motor yes") {
      if (testMotorActive) {
        Serial.println("[MOTOR TEST] already running - 'debug motor' stops it");
      } else if (!motorConfirmPending || (long)(millis() - motorConfirmDeadline) > 0) {
        motorConfirmPending = false;
        Serial.println("[MOTOR TEST] no pending confirm - type 'debug motor' first");
      } else {
        motorConfirmPending = false;
        toggleMotorTest();
      }
    }
    else if (command == "debug imu") {
      debugImuMonitoring = !debugImuMonitoring;  // Toggle raw-sensor stream
      if (debugImuMonitoring) {
        Serial.println("\n[INFO] IMU raw stream ENABLED - Type 'debug imu' again to disable");
      } else {
        Serial.println("\n[INFO] IMU raw stream DISABLED\n");
      }
    }
    else if (command == "debug led") {
      debugLedMonitoring = !debugLedMonitoring;  // Toggle RGB cycle
      if (debugLedMonitoring) {
        Serial.println("\n[INFO] LED cycle ENABLED - Type 'debug led' again to disable (link LED resumes)");
      } else {
        Serial.println("\n[INFO] LED cycle DISABLED\n");
      }
    }
    else if (command == "debug") {
      Serial.println("\n[DEBUG] usage: debug motor | debug imu | debug led | debug ws | debug nvs");
    }
    else if (command == "debug ws") {
      wsDebugMonitoring = !wsDebugMonitoring;  // Toggle WebSocket message log
      if (wsDebugMonitoring) {
        Serial.println("\n[WS] Message log ENABLED - Type 'debug ws' again to disable");
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
    else if (command == "comms show") {
      printCommsToSerial();
    }
    else if (command == "comms save") {
      saveCommsToNVS();
      Serial.println("[COMMS] saved to NVS - reboot to apply");
    }
    else if (command == "comms reset") {
      resetCommsToDefaults();
      Serial.println("[COMMS] overrides cleared, defaults restored - reboot to apply");
    }
    else if (command.startsWith("comms set ")) {
      // Staged in RAM only; 'comms save' persists, reboot applies.
      String args = command.substring(10);
      args.trim();
      int sp = args.indexOf(' ');
      if (sp <= 0) {
        Serial.println("[COMMS] usage: comms set <mode|mac|wifimode> <value>");
      } else {
        String kname = args.substring(0, sp);
        kname.toLowerCase();
        String cval = args.substring(sp + 1);
        cval.trim();
        String cerr;
        if (setStagedComms(kname, cval, cerr)) {
          Serial.printf("[COMMS] %s staged (unsaved - 'comms save' + reboot)\n", kname.c_str());
        } else {
          Serial.print("[COMMS] rejected: ");
          Serial.println(cerr);
        }
      }
    }
    else if (command.startsWith("pin set ") || command.startsWith("pins set ")) {
      // Single + batch, auto-saved to NVS; reboot applies.
      String args = command.startsWith("pin set ")
          ? command.substring(8) : command.substring(9);
      args.trim();
      handlePinSetArgs(args, "PINS");
    }
    else if (command == "debug nvs") {
      printNvsDebugToSerial();
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

  // ===== Rate-limited [IMU] raw-sensor stream ('debug imu' toggle) =====
  // Inside setup: rolling 5-line window — every 5th line scrolls the old
  // batch out of view and relabels the step instead of flooding.
  static unsigned long lastImuDbg = 0;
  static uint8_t imuDbgLines = 0;
  if (debugImuMonitoring && (millis() - lastImuDbg >= 200)) {
    lastImuDbg = millis();
    if (setupActive && imuDbgLines >= 5) {
      imuDbgLines = 0;
      setupClear();
      setupStepHeader();
    }
    if (mpu && mpuInitialized) {
      Serial.print("[IMU] accel=");
      Serial.print(mpu->accelX); Serial.print(",");
      Serial.print(mpu->accelY); Serial.print(",");
      Serial.print(mpu->accelZ); Serial.print(" gyro=");
      Serial.print(mpu->gyroX); Serial.print(",");
      Serial.print(mpu->gyroY); Serial.print(",");
      Serial.print(mpu->gyroZ); Serial.print(" temp=");
      Serial.println(mpu->temp);
    } else {
      Serial.println("[IMU] no driver (not initialized)");
    }
    imuDbgLines++;
  }

  // ===== LED cycle ('debug led' toggle) =====
  // Low-battery blink keeps priority; otherwise cycle R/G/B/off every 300ms.
  // Battery monitor resumes the link LED on next poll after toggle-off.
  static unsigned long lastLedDbg = 0;
  static uint8_t ledDbgStep = 0;
  if (debugLedMonitoring && batteryState != BATTERY_LOW_CONFIRMED &&
      (millis() - lastLedDbg >= 300)) {
    lastLedDbg = millis();
    ledDbgStep = (ledDbgStep + 1) % 4;
    switch (ledDbgStep) {
      case 0: ledSetRGB(255, 0, 0); break;
      case 1: ledSetRGB(0, 255, 0); break;
      case 2: ledSetRGB(0, 0, 255); break;
      default: ledSet(false); break;
    }
  }
}
