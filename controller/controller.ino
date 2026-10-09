// ============================================================
// ESP-NOW RC Transmitter Controller
// Controls Quadcopter Flight Controller via ESP-NOW protocol
// Joystick Inputs: 2x Analog Joysticks + 2x Buttons
// Output: 50 Hz ESP-NOW packets with RC data + RSSI feedback
// OLED Display: 0.96" SSD1306 showing telemetry data
// CALIBRATION: Automatic joystick centering at startup
// LPF: Exponential moving average filters for joystick noise reduction
// SAFETY: Forces DISARMED state on new connection & connection loss
// SAFETY: Arm button locked when ESP-NOW connection is lost
// ============================================================

// ===== BOARD SELECTION =====
// Select your ESP32 board type (uncomment ONE only)
// #define BOARD_ESP32     // Standard ESP32 (30-pin)
#define BOARD_ESP32C3   // ESP32-C3 (13-pin)
// #define BOARD_ESP32S3   // ESP32-S3

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <U8g2lib.h>
#include <Wire.h>

// ===== HARDWARE CONFIGURATION =====
#if defined(BOARD_ESP32)
  // Joystick 1 (Left) - Throttle & Yaw
  #define JS1_X 33      // Yaw input
  #define JS1_Y 34      // Throttle input
  #define JS1_SW 16     // Arm/Disarm button

  // Joystick 2 (Right) - Pitch & Roll 
  #define JS2_X 32      // Roll input
  #define JS2_Y 33      // Pitch input
  #define JS2_SW 17     // Status toggle button

  // OLED Display pins
  #define OLED_SDA 21   // SDA pin
  #define OLED_SCL 22   // SCL pin

  // Battery monitor
  #define RC_BATTERY_PIN 34 // ADC1 pin for controller battery voltage on ESP32
#elif defined(BOARD_ESP32C3)
  // Joystick 1 (Left) - Throttle & Yaw
  #define JS1_X 4       // Yaw input
  #define JS1_Y 3       // Throttle input
  #define JS1_SW 6      // Arm/Disarm button

  // Joystick 2 (Right) - Pitch & Roll 
  #define JS2_X 1       // Roll input
  #define JS2_Y 0       // Pitch input
  #define JS2_SW 5      // Status toggle button

  // OLED Display pins
  #define OLED_SDA 7    // SDA pin
  #define OLED_SCL 10   // SCL pin

  // Battery monitor
  #define RC_BATTERY_PIN 2  // ADC pin for controller battery voltage on ESP32-C3
#elif defined(BOARD_ESP32S3)
  // Joystick 1 (Left) - Throttle & Yaw
  #define JS1_X 9       // Yaw input
  #define JS1_Y 10       // Throttle input
  #define JS1_SW 43      // Arm/Disarm button

  // Joystick 2 (Right) - Pitch & Roll 
  #define JS2_X 12      // Roll input
  #define JS2_Y 13       // Pitch input
  #define JS2_SW 44     // Status toggle button

  // OLED Display pins
  #define OLED_SDA 1    // SDA pin
  #define OLED_SCL 4    // SCL pin

  // Battery monitor
  #define RC_BATTERY_PIN 7  // ADC pin for controller battery voltage on ESP32-S3
#else
  #error "Unsupported board profile in controller configuration"
#endif

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ===== BATTERY VOLTAGE MONITORING =====
#define RC_BATTERY_DIVIDER_RATIO 1.99f  // Voltage divider ratio (91K + 91K = 1:2) * multimeter calibration ratio
#define RC_BATTERY_SAMPLE_SIZE 18  // Number of samples for averaging
#define RC_BATTERY_LPF_ALPHA 0.1f  // Low-pass filter coefficient

// ===== DYNAMIC DRONE DISCOVERY =====
// Controller discovers drones when they send feedback
// No hardcoded drone MAC needed - works with any drone listening to this controller
#define MAX_DISCOVERED_DRONES 5
struct DiscoveredDrone {
  uint8_t mac[6];
  unsigned long last_feedback_time;
  int rssi;
  float battery_voltage;
};
DiscoveredDrone discovered_drones[MAX_DISCOVERED_DRONES];
int num_discovered_drones = 0;
const unsigned long DRONE_DISCOVERY_TIMEOUT = 5000;  // Forget drone if no feedback for 5 sec

uint8_t txMAC[6];  // This controller's MAC (will be read at runtime)

// ===== DATA STRUCTURE =====
// Matches the RC_Data structure in esp_now_handler.h
typedef struct {
  int throttle;           // Throttle input (-2048 to +2048)
  int yaw;                // Yaw input (-2048 to +2048)
  int roll;               // Roll input (-2048 to +2048)
  int pitch;              // Pitch input (-2048 to +2048)
  bool b1;                // Button 1 (arm/disarm)
  bool b2;                // Button 2 (status toggle)
  unsigned long timestamp; // Timestamp for latency calculation
} RC_Data;

// ===== GLOBAL VARIABLES =====
RC_Data rcData;
bool rx_alive = false;
unsigned long lastAck = 0;
unsigned long lastSend = 0;
unsigned long lastSendTime = 0;  // Time of last sent packet (for RTT calculation)
unsigned long connectionLostTime = 0;
unsigned long lastStatusPrint = 0;
bool b1_prev = false;
bool b2_prev = false;
bool armed_state = false;  // Toggle state for arm/disarm
bool b2_state = false;     // Toggle state for button 2 (alt hold)
unsigned long last_b1_press_time = 0;  // Timestamp of last B1 button press
unsigned long last_b2_press_time = 0;  // Timestamp of last B2 button press
#define BUTTON_PRESS_DEBOUNCE_MS 200  // Don't auto-sync for 200ms after manual button press
bool just_connected = false;  // Flag for new connection establishment
int last_rssi = 0;  // Signal strength from flight controller feedback
float last_battery = 0.0f;  // Battery voltage from flight controller
bool drone_armed = false;  // Actual armed state from drone feedback
bool drone_alt_hold = false;  // Actual altitude hold state from drone feedback
float rc_battery_voltage = 0.0f;  // Controller battery voltage
float rc_battery_voltage_filtered = 0.0f;  // Low-pass filtered controller battery voltage
float rc_battery_samples[RC_BATTERY_SAMPLE_SIZE] = {0};
int rc_battery_sample_index = 0;

// ===== LOW PASS FILTER VARIABLES =====
// Exponential moving average filters for joystick noise reduction
#define LPF_ALPHA 0.3f  // Filter coefficient (0.1 = heavy filtering, 0.5 = light filtering)
// Set to 0 to disable startup auto-zero (bias calibration) for analog sticks.
#define AUTO_JOYSTICK_CENTER_CALIBRATION 0
float lpf_throttle = 0.0f;
float lpf_yaw = 0.0f;
float lpf_roll = 0.0f;
float lpf_pitch = 0.0f;
bool lpf_initialized = false;  // Flag to initialize filters on first reading
// ===== JOYSTICK TRIM/CALIBRATION VARIABLES =====
// Automatic centering calibration performed at startup
int trim_throttle = 0;
int trim_yaw = 0;
int trim_roll = 0;
int trim_pitch = 0;unsigned long last_rtt = 0;  // Round-trip latency

// ===== ADC CONVERSION FUNCTION =====
// Convert ADC value (0-4095) to ±2048 range (center at ~2048)
int axis(int raw) {
  return -(raw - 2048);  // Invert for intuitive control
}

// ===== LOW PASS FILTER FUNCTION =====
// Exponential moving average filter for noise reduction
float lowPassFilter(float current_value, float *filtered_value, float alpha) {
  if (!lpf_initialized) {
    *filtered_value = current_value;  // Initialize on first call
    lpf_initialized = true;
  } else {
    *filtered_value = alpha * current_value + (1.0f - alpha) * (*filtered_value);
  }
  return *filtered_value;
}

// ===== JOYSTICK CALIBRATION FUNCTION =====
// Automatically calibrate joystick centers at startup
void calibrateJoysticks() {
  #if !AUTO_JOYSTICK_CENTER_CALIBRATION
  trim_throttle = 0;
  trim_yaw = 0;
  trim_roll = 0;
  trim_pitch = 0;
  Serial.println("[CALIBRATION] Auto joystick center calibration DISABLED (all trims = 0)");
  return;
  #endif

  Serial.println("[CALIBRATION] Calibrating joystick centers...");
  Serial.println("[CALIBRATION] Please ensure all joysticks are centered!");
  
  // Take multiple readings and average them for better accuracy
  const int samples = 50;
  long sum_yaw = 0, sum_roll = 0, sum_pitch = 0;
  
  for (int i = 0; i < samples; i++) {
    sum_yaw += axis(analogRead(JS1_X));
    sum_roll += axis(analogRead(JS2_X));
    sum_pitch += -(axis(analogRead(JS2_Y)));
    delay(10);  // Small delay between readings
  }
  
  // Calculate average offsets
  trim_throttle = 0;  // Throttle uses raw stick value (no auto-centering)
  trim_yaw = sum_yaw / samples;
  trim_roll = sum_roll / samples;
  trim_pitch = sum_pitch / samples;
  
  // Display calibration results
  Serial.printf("[CALIBRATION] Trim offsets - Throttle: %d (raw), Yaw: %d, Roll: %d, Pitch: %d\n",
                trim_throttle, trim_yaw, trim_roll, trim_pitch);
  Serial.println("[CALIBRATION] Joystick calibration complete!");
}

// ===== UPDATE RC BATTERY VOLTAGE =====
void updateRCBatteryVoltage() {
  // Read raw ADC value
  int adc_raw = analogRead(RC_BATTERY_PIN);
  // Convert to voltage: (adc_raw / 4095.0) * 3.3 * RC_BATTERY_DIVIDER_RATIO
  float raw_voltage = (adc_raw / 4095.0f) * 3.3f * RC_BATTERY_DIVIDER_RATIO;
  
  // Store in circular buffer for averaging
  rc_battery_samples[rc_battery_sample_index] = raw_voltage;
  rc_battery_sample_index = (rc_battery_sample_index + 1) % RC_BATTERY_SAMPLE_SIZE;
  
  // Calculate average of all samples
  float voltage_sum = 0.0f;
  for (int i = 0; i < RC_BATTERY_SAMPLE_SIZE; i++) {
    voltage_sum += rc_battery_samples[i];
  }
  float voltage_average = voltage_sum / RC_BATTERY_SAMPLE_SIZE;
  
  // Apply low-pass filter: filtered = α * new_value + (1-α) * filtered_old
  rc_battery_voltage_filtered = (RC_BATTERY_LPF_ALPHA * voltage_average) + ((1.0f - RC_BATTERY_LPF_ALPHA) * rc_battery_voltage_filtered);
  
  // Use filtered value as the main battery voltage
  rc_battery_voltage = rc_battery_voltage_filtered;
}

// ===== UPDATE OLED DISPLAY =====
void updateOLED() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Top line: Arm status and B2 status
  char armText[12] = {0};
  if (!rx_alive) {
    sprintf(armText, "NO LINK");
  } else {
    sprintf(armText, armed_state ? "ARMED" : "DISARMED");
  }
  u8g2.drawStr(0, 10, armText);
  u8g2.drawStr(64, 10, b2_state ? "B2:ON" : "B2:OFF");

  // Flight data - two columns
  char thrBuffer[12] = {0};
  char yawBuffer[12] = {0};
  char rolBuffer[12] = {0};
  char pitBuffer[12] = {0};

  sprintf(thrBuffer, "THR:%4d", rcData.throttle);
  u8g2.drawStr(0, 22, thrBuffer);
  sprintf(yawBuffer, "YAW:%4d", rcData.yaw);
  u8g2.drawStr(64, 22, yawBuffer);

  sprintf(rolBuffer, "ROL:%4d", rcData.roll);
  u8g2.drawStr(0, 34, rolBuffer);
  sprintf(pitBuffer, "PIT:%4d", rcData.pitch);
  u8g2.drawStr(64, 34, pitBuffer);

  // Telemetry data
  char batBuffer[12] = {0};
  char rssiBuffer[12] = {0};
  char rcBuffer[12] = {0};

  sprintf(batBuffer, "BAT:%.1fV", last_battery);
  u8g2.drawStr(0, 46, batBuffer);
  sprintf(rssiBuffer, "RSSI:%3d", last_rssi);
  u8g2.drawStr(64, 46, rssiBuffer);

  sprintf(rcBuffer, "RC :%.1fV", rc_battery_voltage);
  u8g2.drawStr(0, 58, rcBuffer);
  u8g2.drawStr(64, 58, rx_alive ? "LINK:YES" : "LINK:NO");  // LINK:YES for connected, LINK:NO for lost

  u8g2.sendBuffer();

  // Debug: confirm OLED update
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {  // Print every second
    lastDebug = millis();
    Serial.println("[DEBUG] OLED updated");
  }
}

// ===== SEND CALLBACK =====
// Called when we send data to the flight controller
void onSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[WARNING] ESP-NOW packet send failed");
  }
}

// ===== RECEIVE CALLBACK =====
// Receive RSSI feedback from flight controller(s)
// Also discovers drones dynamically by MAC address
// Backwards compatible: handles both 4-byte (old) and 5-byte (new) packets
void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len >= 4) {  // Accept both 4-byte (old) and 5-byte (new) packets
    bool was_alive = rx_alive;
    rx_alive = true;
    lastAck = millis();
    connectionLostTime = 0;  // Reset connection lost timer

    // Detect new connection establishment
    if (!was_alive && rx_alive) {
      just_connected = true;
      Serial.println("[SAFETY] New connection established - forcing DISARMED state");
    }

    // Extract RSSI (bytes 0-1)
    int16_t rssi_raw = (int16_t)((incomingData[0] << 8) | incomingData[1]);
    last_rssi = rssi_raw;
    
    // Extract battery voltage (bytes 2-3) - encoded as uint16 in units of 0.01V
    uint16_t vbat_raw = (uint16_t)((incomingData[2] << 8) | incomingData[3]);
    last_battery = vbat_raw / 100.0f;  // Convert back to volts
    
    // Extract FLAGS byte (byte 4) if available - bit 0 = motorsArmed, bit 1 = altitudeHoldEnabled
    if (len >= 5) {
      uint8_t flags = incomingData[4];
      drone_armed = (flags & (1 << 0)) != 0;
      drone_alt_hold = (flags & (1 << 1)) != 0;
      
      // AUTO-SYNC: If drone state differs from controller state, sync controller to drone
      // But respect debounce window - don't sync for 200ms after manual button press
      unsigned long now = millis();
      bool in_b1_debounce = (now - last_b1_press_time) < BUTTON_PRESS_DEBOUNCE_MS;
      bool in_b2_debounce = (now - last_b2_press_time) < BUTTON_PRESS_DEBOUNCE_MS;
      
      if (!in_b1_debounce && drone_armed != armed_state) {
        armed_state = drone_armed;
        if (!drone_armed) {
          Serial.println("[SYNC] Drone disarmed detected - controller automatically disarmed");
        } else {
          Serial.println("[SYNC] Drone armed detected - controller automatically armed");
        }
      }
      
      if (!in_b2_debounce && drone_alt_hold != b2_state) {
        b2_state = drone_alt_hold;
        Serial.print("[SYNC] Altitude hold state synced to: ");
        Serial.println(drone_alt_hold ? "ON" : "OFF");
      }
    }
    // If packet is 4 bytes (old format), drone_armed and drone_alt_hold keep their previous state
    
    // Calculate round-trip latency
    last_rtt = millis() - lastSendTime;
    
    // ===== DYNAMIC DRONE DISCOVERY =====
    // Learn drone MAC from incoming feedback
    if (info && info->src_addr) {
      // Search for existing drone entry
      int drone_index = -1;
      for (int i = 0; i < num_discovered_drones; i++) {
        if (memcmp(discovered_drones[i].mac, info->src_addr, 6) == 0) {
          drone_index = i;
          break;
        }
      }
      
      if (drone_index == -1 && num_discovered_drones < MAX_DISCOVERED_DRONES) {
        // New drone discovered
        drone_index = num_discovered_drones++;
        memcpy(discovered_drones[drone_index].mac, info->src_addr, 6);
        Serial.printf("[DISCOVERY] Drone #%d found: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      drone_index + 1,
                      info->src_addr[0], info->src_addr[1], info->src_addr[2],
                      info->src_addr[3], info->src_addr[4], info->src_addr[5]);
      }
      
      // Update drone feedback info
      if (drone_index >= 0) {
        discovered_drones[drone_index].last_feedback_time = millis();
        discovered_drones[drone_index].rssi = rssi_raw;
        discovered_drones[drone_index].battery_voltage = last_battery;
      }
    }
  }
}

// ===== CLEANUP STALE DRONES =====
// Remove drones that haven't sent feedback recently
void cleanupStaleDrones() {
  unsigned long now = millis();
  for (int i = 0; i < num_discovered_drones; i++) {
    if (now - discovered_drones[i].last_feedback_time > DRONE_DISCOVERY_TIMEOUT) {
      Serial.printf("[DISCOVERY] Removing stale drone #%d: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    i + 1,
                    discovered_drones[i].mac[0], discovered_drones[i].mac[1], discovered_drones[i].mac[2],
                    discovered_drones[i].mac[3], discovered_drones[i].mac[4], discovered_drones[i].mac[5]);
      // Shift array down
      for (int j = i; j < num_discovered_drones - 1; j++) {
        memcpy(&discovered_drones[j], &discovered_drones[j + 1], sizeof(DiscoveredDrone));
      }
      num_discovered_drones--;
      i--;  // Re-check this index
    }
  }
}

// ===== ADD DRONE AS PEER =====
// Register a discovered drone as an ESP-NOW peer
void addDronePeer(const uint8_t *drone_mac) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, drone_mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  
  esp_err_t result = esp_now_add_peer(&peer);
  if (result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST) {
    // Peer added or already exists - no error
  } else {
    Serial.print("[WARNING] Failed to add drone peer. Error: ");
    Serial.println(result);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to stabilize
  
  Serial.println("\n===== ESP-NOW RC Controller Setup =====");
  
  // Configure button pins
  pinMode(JS1_SW, INPUT_PULLUP);
  pinMode(JS2_SW, INPUT_PULLUP);
  
  // Set WiFi mode to station (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Disable WiFi power save to avoid intermittent radio sleep on some modules
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  // Get this device's MAC address
  WiFi.macAddress(txMAC);
  Serial.printf("[INFO] Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                txMAC[0], txMAC[1], txMAC[2], txMAC[3], txMAC[4], txMAC[5]);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW initialization failed");
    return;
  }
  
  Serial.println("[INFO] ESP-NOW initialized successfully");
  Serial.printf("[INFO] WiFi channel: %d\n", WiFi.channel());
  Serial.println("[INFO] Waiting for drones to send feedback...");
  Serial.println("[INFO] Any drone listening to this controller's MAC will be auto-discovered");
  
  // Register callbacks
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);
  
  // Initialize OLED display
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "RC Controller");
  u8g2.drawStr(0, 28, "Initializing...");
  u8g2.sendBuffer();
  delay(1000);
  
  Serial.println("[INFO] OLED display initialized successfully");
  
  // Calibrate joystick centers
  calibrateJoysticks();
  
  // Initialize RC data structure
  memset(&rcData, 0, sizeof(rcData));
  
  Serial.println("[INFO] RC Controller Ready - Waiting for flight controller...\n");
  Serial.println("Controls:");
  Serial.println("  JS1 (Left):  X=Yaw, Y=Throttle, SW=Arm/Disarm");
  Serial.println("  JS2 (Right): X=Roll, Y=Pitch, SW=Status Toggle");
}

// ===== MAIN LOOP =====
void loop() {
  // Read analog joystick values, apply trim calibration, then LPF for noise reduction
  int raw_yaw = -(axis(analogRead(JS1_X)) - trim_yaw);
  int raw_throttle = axis(analogRead(JS1_Y));
  int raw_roll = axis(analogRead(JS2_X)) - trim_roll;
  int raw_pitch = (axis(analogRead(JS2_Y))) - trim_pitch;
  
  // Apply low pass filters to reduce noise
  rcData.yaw = (int)lowPassFilter((float)raw_yaw, &lpf_yaw, LPF_ALPHA);
  rcData.throttle = (int)lowPassFilter((float)raw_throttle, &lpf_throttle, LPF_ALPHA);
  rcData.roll = (int)lowPassFilter((float)raw_roll, &lpf_roll, LPF_ALPHA);
  rcData.pitch = (int)lowPassFilter((float)raw_pitch, &lpf_pitch, LPF_ALPHA);
  
  // Read button states with debouncing
  bool b1_current = digitalRead(JS1_SW) == LOW;
  bool b2_current = digitalRead(JS2_SW) == LOW;
  
  // Toggle arm state on button press (only when connected)
  if (b1_current && !b1_prev) {
    if (rx_alive) {
      // Only allow arm/disarm when connected to drone
      armed_state = !armed_state;
      last_b1_press_time = millis();  // Record button press time for debounce
      Serial.printf("[BUTTON] Arm state toggled to: %s\n", armed_state ? "ARMED" : "DISARMED");
    } else {
      // Connection required for arming
      Serial.println("[SAFETY] Cannot arm/disarm - no connection to drone!");
    }
  }
  b1_prev = b1_current;

  // Toggle B2 state on button press (only when connected)
  if (b2_current && !b2_prev) {
    if (rx_alive) {
      b2_state = !b2_state;
      last_b2_press_time = millis();  // Record button press time for debounce
      Serial.printf("[BUTTON] B2 state toggled to: %s\n", b2_state ? "ON" : "OFF");
    } else {
      Serial.println("[SAFETY] Cannot toggle B2 - no connection to drone!");
    }
  }
  b2_prev = b2_current;

  // Send latched B2 state
  rcData.b2 = b2_state;
  
  // Send data at 50 Hz (every 20 ms)
  if (millis() - lastSend > 20) {
    lastSend = millis();
    lastSendTime = millis();  // Record time for RTT calculation
    rcData.timestamp = millis();  // Record timestamp for latency calculation
    
    // Safety: Force DISARMED on new connection establishment
    if (just_connected) {
      armed_state = false;  // Force disarmed
      b2_state = false;     // Force B2 off
      just_connected = false;  // Clear the flag
      Serial.println("[SAFETY] Forced DISARMED state on new connection");
    }
    
    rcData.b1 = armed_state;  // Send arm state directly to drone
    
    // Send RC data to all discovered drones
    if (num_discovered_drones > 0) {
      for (int i = 0; i < num_discovered_drones; i++) {
        // Add drone as peer if not already
        addDronePeer(discovered_drones[i].mac);
        
        // Send RC data
        esp_err_t result = esp_now_send(discovered_drones[i].mac, (uint8_t*)&rcData, sizeof(rcData));
        if (result != ESP_OK) {
          Serial.printf("[WARNING] Failed to send RC data to drone #%d: %s (%d)\n", i + 1, esp_err_to_name(result), result);
        }
      }
      
      // Debug: show what arm state is being sent
      static bool last_sent_arm_state = false;
      if (rcData.b1 != last_sent_arm_state) {
        Serial.printf("[ESP-NOW] Sent arm state to %d drone(s): %s\n", num_discovered_drones, rcData.b1 ? "ARMED" : "DISARMED");
        last_sent_arm_state = rcData.b1;
      }
    } else {
      // No drones discovered yet
      if (millis() % 5000 == 0) {  // Print every 5 seconds
        Serial.println("[DISCOVERY] Waiting for drone feedback to auto-discover...");
      }
    }
  }
  
  // Periodically cleanup stale drones (every 1 second)
  static unsigned long last_cleanup = 0;
  if (millis() - last_cleanup > 1000) {
    last_cleanup = millis();
    cleanupStaleDrones();
  }
  
  // Update RC battery voltage
  updateRCBatteryVoltage();
  
  // Print telemetry and update OLED every 100ms
  if (millis() - lastStatusPrint > 100) {
    lastStatusPrint = millis();
    
    Serial.printf("[TX] Throttle: %5d | Pitch: %5d | Roll: %5d | Yaw: %5d | B1: %s | B2: %s | RSSI: %4d dBm | RTT: %4lums | Vbat(FC): %.2fV | RC_BAT: %.2fV | %s\n",
                  rcData.throttle,
                  rcData.pitch,
                  rcData.roll,
                  rcData.yaw,
                  rx_alive ? (armed_state ? "ARMED" : "DISARMED") : "LOCKED",
            b2_state ? "ON" : "OFF",
                  last_rssi,
                  last_rtt,
                  last_battery,
                  rc_battery_voltage,
                  rx_alive ? "LINK:YES" : "LINK:NO");
    
    // Update OLED display (only when telemetry is printed)
    updateOLED();
  }
  
  // Check connection status
  if (rx_alive && (millis() - lastAck > 2000)) {
    // Connection was alive but now lost
    if (connectionLostTime == 0) {
      connectionLostTime = millis();
      Serial.println("[WARNING] Lost connection to flight controller!");
      Serial.println("[INFO] Attempting to reconnect...");
      
      // SAFETY: Automatically disarm when connection is lost
      if (armed_state || b2_state) {
        armed_state = false;
        b2_state = false;
        Serial.println("[SAFETY] Automatically DISARMED due to connection loss!");
        // OLED will update on next telemetry cycle (within 100ms)
      }
    }
    rx_alive = false;
  }
  
  // Attempt to reconnect if offline for too long
  if (!rx_alive && (millis() - connectionLostTime > 10000)) {
    connectionLostTime = millis();
    Serial.println("[INFO] Waiting for drones to reconnect via auto-discovery...");
    // Drones will send discovery packets automatically
  }
  
  // Small delay to prevent watchdog timeout
  delay(1);
}