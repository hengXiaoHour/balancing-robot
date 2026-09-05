#include <Arduino.h>
#include "esp_now_handler.h"

static portMUX_TYPE espnow_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile RC_Data espnow_data;
static volatile uint32_t espnow_seq = 0;
volatile bool espnow_connected = false;
static volatile unsigned long last_espnow_update = 0;

// Throttle gating state defined here (were in balancing_robot.ino); externs in esp_now_handler.h
bool throttle_gate_ready = false;
float last_throttle = 0.0f;
static volatile int last_rssi = 0;
static volatile unsigned long last_rssi_update = 0;
static unsigned long connection_lost_time = 0;
static volatile uint8_t last_sender_mac[6] = {0};  // Store controller MAC for feedback
static volatile bool need_send_feedback = false;   // Flag to send feedback without blocking
static unsigned long last_discovery_sent = 0;  // For discovery packet intervals

static inline float mapESPNOWThrottlePercent(float throttle_input) {
  // Shaping values live here (were ESPNOW_THROTTLE_* in settings.h)
  const float THR_MAX = 100.0f, THR_MIN = 0.0f;
  float stick = constrain(throttle_input, -1.0f, 1.0f);
  float mid = constrain(30.0f, THR_MIN, THR_MAX);

  if (stick <= 0.0f) {
    float t = stick + 1.0f;  // [-1..0] -> [0..1]
    float shaped = powf(t, 1.8f);  // >1.0 = gentler near low stick
    return THR_MIN + shaped * (mid - THR_MIN);
  }

  float t = stick;  // [0..1]
  float shaped = powf(t, 0.7f);  // <1.0 = stronger above center
  return mid + shaped * (THR_MAX - mid);
}

// ===== INITIALIZE ESP-NOW =====
void initESPNOW() {
  Serial.println("\n===== ESP-NOW Receiver Setup =====");

  // Set WiFi mode to station (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] Failed to initialize ESP-NOW");
    return;
  }

  Serial.println("[INFO] ESP-NOW initialized successfully");

  // Register callbacks
  esp_now_register_recv_cb(onESPNOWReceive);
  esp_now_register_send_cb(onESPNOWSend);

  // Get this device's MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("[INFO] Flight Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("[INFO] Listening for controller with MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                CONTROLLER_MAC_0, CONTROLLER_MAC_1, CONTROLLER_MAC_2, CONTROLLER_MAC_3, CONTROLLER_MAC_4, CONTROLLER_MAC_5);

  Serial.println("[INFO] Will auto-announce to controller every 2 seconds until connection...");

  // Initialize data structure
  memset((void*)&espnow_data, 0, sizeof(espnow_data));

  // Initialize timestamp to prevent integer overflow on first timeout check
  portENTER_CRITICAL(&espnow_mux);
  last_espnow_update = millis();
  portEXIT_CRITICAL(&espnow_mux);
}

// ===== SEND DISCOVERY PACKET =====
// Send periodic discovery packet to controller so it learns our MAC
void sendDiscoveryPacket() {
  // Create controller peer if not already added
  static bool controller_peer_added = false;
  if (!controller_peer_added) {
    uint8_t controller_mac[6] = {CONTROLLER_MAC_0, CONTROLLER_MAC_1, CONTROLLER_MAC_2, CONTROLLER_MAC_3, CONTROLLER_MAC_4, CONTROLLER_MAC_5};

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, controller_mac, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) == ESP_OK) {
      controller_peer_added = true;
    }
  }

  // Send a dummy feedback packet (4 bytes) to announce ourselves
  uint8_t controller_mac[6] = {CONTROLLER_MAC_0, CONTROLLER_MAC_1, CONTROLLER_MAC_2, CONTROLLER_MAC_3, CONTROLLER_MAC_4, CONTROLLER_MAC_5};
  uint8_t dummy_feedback[4] = {0xFF, 0xFF, 0x00, 0x00};  // Indicate discovery packet
  esp_now_send(controller_mac, dummy_feedback, sizeof(dummy_feedback));
}

// ===== RECEIVE CALLBACK =====
// Called when data is received from the controller
void onESPNOWReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(RC_Data)) {
    bool just_connected = false;
    uint8_t mac_copy[6] = {0};

    portENTER_CRITICAL_ISR(&espnow_mux);
    memcpy((void*)&espnow_data, incomingData, sizeof(RC_Data));
    espnow_seq++;
    last_espnow_update = millis();

    if (!espnow_connected) {
      espnow_connected = true;
      just_connected = true;
      memcpy(mac_copy, info->src_addr, 6);
    }

    if (info) {
      last_rssi = info->rx_ctrl->rssi;
      last_rssi_update = millis();
    }

    memcpy((void*)last_sender_mac, info->src_addr, 6);
    need_send_feedback = true;
    portEXIT_CRITICAL_ISR(&espnow_mux);

    if (just_connected) {
      Serial.println("[SUCCESS] ✓ ESP-NOW Controller CONNECTED!");
      Serial.printf("[DEBUG] Received from: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac_copy[0], mac_copy[1], mac_copy[2],
                    mac_copy[3], mac_copy[4], mac_copy[5]);
    }
  }
}

// ===== SEND CALLBACK =====
// Called when we send data (ACK)
void onESPNOWSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Optional: Can be used for debugging
  // if (status != ESP_NOW_SEND_SUCCESS) {
  //   Serial.println("[WARNING] ESP-NOW send failed");
  // }
}

// ===== SEND RSSI FEEDBACK =====
// Send RSSI, battery, armed state, and altitude hold state back to controller
void sendRSSIFeedback(const uint8_t *sender_mac) {
  // Add controller as peer if not already added
  static bool peer_added = false;
  if (!peer_added) {
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, sender_mac, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) == ESP_OK) {
      peer_added = true;
    }
  }

  // Create feedback packet: [RSSI_HIGH, RSSI_LOW, VBAT_HIGH, VBAT_LOW, FLAGS]
  // Encode battery voltage as uint16 (in units of 0.01V, so 415 = 4.15V)
  uint16_t vbat_encoded = (uint16_t)(battery_voltage * 100.0f);

  uint8_t feedback[5];
  feedback[0] = (last_rssi >> 8) & 0xFF;
  feedback[1] = last_rssi & 0xFF;
  feedback[2] = (vbat_encoded >> 8) & 0xFF;
  feedback[3] = vbat_encoded & 0xFF;

  // FLAGS byte: bit 0 = motorsArmed, bit 1 = reserved
  feedback[4] = 0;
  if (motorsArmed) feedback[4] |= (1 << 0);

  esp_now_send(sender_mac, feedback, sizeof(feedback));
}

// ===== FAST PATH: Update RC data (lightweight, safe for 1kHz) =====
// Only copies mutex-protected data - no blocking calls
void updateESPNOWDataOnly() {
  bool connected = false;
  RC_Data local_data;

  portENTER_CRITICAL(&espnow_mux);
  connected = espnow_connected;
  if (connected) {
    memcpy(&local_data, (const void*)&espnow_data, sizeof(RC_Data));
  }
  portEXIT_CRITICAL(&espnow_mux);

  if (!connected) {
    return;
  }

  // ===== PROCESS RC DATA (no lock needed - all local) =====
  const float scale_factor = ESPNOW_MAX_PITCH / 2048.0f;

  // Apply deadzone to raw input values before conversion
  int16_t pitch_raw = local_data.pitch;
  int16_t roll_raw = local_data.roll;
  int16_t yaw_raw = local_data.yaw;

  #if JOYSTICK_SWAP_ROLL_PITCH_INPUT
  int16_t tmp = pitch_raw;
  pitch_raw = roll_raw;
  roll_raw = tmp;
  #endif

  pitch_raw = (int16_t)(pitch_raw * CONTROLLER_PITCH_SIGN);
  roll_raw = (int16_t)(roll_raw * CONTROLLER_ROLL_SIGN);
  yaw_raw = (int16_t)(yaw_raw * CONTROLLER_YAW_SIGN);

  if (abs(pitch_raw) < ESPNOW_PITCH_DEADZONE) pitch_raw = 0;
  if (abs(roll_raw) < ESPNOW_ROLL_DEADZONE) roll_raw = 0;
  if (abs(yaw_raw) < ESPNOW_YAW_DEADZONE) yaw_raw = 0;

  // Balancing robot control mapping:
  // - Pitch stick: forward/backward lean target (angle or rate depending on mode)
  // - Roll stick : steering lean target (angle or rate depending on mode)
  // - Yaw stick  : turn-rate (always rate mode)

  #if ENABLE_CASCADE_PID && !CASCADE_MODE_ANGLE_CONTROL
  // ===== CASCADE RATE MODE: Map joystick to rate targets =====
  pitch_rate_target = constrain(pitch_raw * scale_factor, -ESPNOW_MAX_PITCH, ESPNOW_MAX_PITCH);
  roll_rate_target = constrain(roll_raw * scale_factor, -ESPNOW_MAX_ROLL, ESPNOW_MAX_ROLL);
  #else
  // ===== ANGLE MODE: Map joystick to angle targets =====
  pitch_setpoint = constrain(pitch_raw * scale_factor, -ESPNOW_MAX_PITCH, ESPNOW_MAX_PITCH);
  roll_setpoint = constrain(roll_raw * scale_factor, -ESPNOW_MAX_ROLL, ESPNOW_MAX_ROLL);
  #endif

  // ===== YAW CONTROL (Always Rate Mode) =====
  #if JOYSTICK_YAW_INPUT_ENABLED
  float yaw_input = constrain(yaw_raw / 2048.0f, -1.0f, 1.0f);
  yaw_input *= YAW_AXIS_SIGN;  // Apply yaw inversion if configured
  yaw_rate_target = constrain(yaw_input * ESPNOW_MAX_YAW_RATE, -ESPNOW_MAX_YAW_RATE, ESPNOW_MAX_YAW_RATE);
  yaw_setpoint = yaw_rate_target;
  #else
  yaw_rate_target = 0.0f;
  yaw_setpoint = 0.0f;
  static bool throttle_gate_ready = false;
  static float last_throttle = 0.0f;
  #endif

  float throttle_input = local_data.throttle / 2048.0f;
  throttle_input = constrain(throttle_input, -1.0f, 1.0f);
  throttle_input_normalized = throttle_input;

  if (!motorsArmed) {
    throttle = 0.0f;
    motorsActive = false;
    throttle_gate_ready = false;
    last_throttle = 0.0f;
  } else {
    motorsActive = true;
    float cmd_magnitude = max(fabs(pitch_setpoint) / max(ESPNOW_MAX_PITCH, 0.001f), fabs(roll_setpoint) / max(ESPNOW_MAX_ROLL, 0.001f));
    float virtual_throttle = (LOW_THROTTLE_THRESHOLD + 5.0f) + (cmd_magnitude * (100.0f - (LOW_THROTTLE_THRESHOLD + 5.0f)));
    throttle = constrain(virtual_throttle, LOW_THROTTLE_THRESHOLD + 5.0f, 100.0f);
    last_throttle = throttle;
  }

  // Handle arm/disarm button
  static bool b1_prev_level = false;
  if (local_data.b1 != b1_prev_level) {
    if (local_data.b1) {
      if (!mpuInitialized) {
        Serial.println("[ERROR] ARM BLOCKED via ESP-NOW: MPU6050 not initialized!");
      } else if (!filterInitialized) {
        Serial.println("[WARN] ARM BLOCKED via ESP-NOW: Filter still warming up, please wait...");
      } else if (!motorsArmed) {
        motorsArmed = true;
        motorsActive = false;
        throttle_gate_ready = false;
        last_throttle = 0.0f;
        throttle = 0.0f;
        Serial.println("[INFO] Balancing robot ARMED via ESP-NOW");
      }
    } else {
      if (motorsArmed) {
        motorsArmed = false;
        motorsActive = false;
        throttle_gate_ready = false;
        last_throttle = 0.0f;
        throttle = 0.0f;
        Serial.println("[INFO] Balancing robot DISARMED via ESP-NOW");
      }
    }
  }
  b1_prev_level = local_data.b1;

  // Handle altitude hold button
  static bool b2_prev = false;
  if (local_data.b2 && !b2_prev) {
    Serial.println("[INFO] Button B2 reserved for future use");
  }
  b2_prev = local_data.b2;
}

// ===== SLOW PATH: Heavy ESP-NOW operations (run at ~10Hz from main loop) =====
void updateESPNOWService() {
  bool conn = false;
  unsigned long last_update = 0;
  bool send_feedback = false;
  uint8_t sender_mac_copy[6] = {0};

  portENTER_CRITICAL(&espnow_mux);
  conn = espnow_connected;
  last_update = last_espnow_update;
  if (need_send_feedback) {
    send_feedback = true;
    need_send_feedback = false;
    memcpy(sender_mac_copy, (const void*)last_sender_mac, 6);
  }
  portEXIT_CRITICAL(&espnow_mux);

  // Sample time after pulling shared state to avoid race where last_update > now.
  unsigned long now = millis();
  unsigned long elapsed = now - last_update;

  if (conn && (elapsed > ESPNOW_TIMEOUT)) {
    portENTER_CRITICAL(&espnow_mux);
    espnow_connected = false;
    portEXIT_CRITICAL(&espnow_mux);
    motorsArmed = false;
    motorsActive = false;
    throttle = 0.0f;
    Serial.printf("[WARNING] ESP-NOW connection lost! Gap: %lums > Timeout: %dms\n", elapsed, ESPNOW_TIMEOUT);
  }

  // Send discovery if not connected (every 2 seconds)
  static unsigned long last_discovery = 0;
  if (!conn && (now - last_discovery > 2000)) {
    last_discovery = now;
    sendDiscoveryPacket();
  }

  // Send RSSI feedback if pending
  if (send_feedback) {
    sendRSSIFeedback(sender_mac_copy);
  }
}

// ===== OLD FUNCTION (DEPRECATED - kept for reference) =====
// Now split into updateESPNOWDataOnly() + updateESPNOWService()
// Call updateESPNOWDataOnly() at 1kHz in control loop
// Call updateESPNOWService() at ~10Hz in main loop
void updateESPNOWControl() {
  // This is now deprecated - use updateESPNOWDataOnly() + updateESPNOWService()
  updateESPNOWDataOnly();   // Fast path (no blocking)
}

// ===== GET CONNECTION STATUS =====
bool isESPNOWConnected() {
  bool conn = false;
  unsigned long last_update = 0;
  portENTER_CRITICAL(&espnow_mux);
  conn = espnow_connected;
  last_update = last_espnow_update;
  portEXIT_CRITICAL(&espnow_mux);
  return conn && (millis() - last_update < ESPNOW_TIMEOUT);
}

// ===== GET LATENCY =====
unsigned long getESPNOWLatency() {
  bool conn = false;
  unsigned long ts = 0;
  portENTER_CRITICAL(&espnow_mux);
  conn = espnow_connected;
  ts = espnow_data.timestamp;
  portEXIT_CRITICAL(&espnow_mux);
  if (!conn) return 0;
  return millis() - ts;
}

// ===== GET RSSI =====
int getESPNOWRSSI() {
  int rssi = 0;
  portENTER_CRITICAL(&espnow_mux);
  rssi = last_rssi;
  portEXIT_CRITICAL(&espnow_mux);
  return rssi;
}
