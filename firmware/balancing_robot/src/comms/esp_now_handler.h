#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include "../config/settings.h"  // ESPNOW_*, CONTROLLER_MAC_*, sign macros (body-free chain)

// ===== EXTERNAL VARIABLES =====
extern float pitch_setpoint;
extern float roll_setpoint;
extern float yaw_setpoint;
extern float pitch_rate_target;  // CASCADE RATE MODE: pitch rate target
extern float roll_rate_target;   // CASCADE RATE MODE: roll rate target
extern float yaw_rate_target;
extern float throttle;
extern float throttle_input_normalized;
extern bool motorsArmed;
extern bool motorsActive;
extern bool statusMonitoring;
extern bool mpuInitialized;  // MPU6050 initialization status
extern bool filterInitialized;  // Filter warm-up complete
extern float battery_voltage;
extern bool throttle_gate_ready;  // Throttle gating for ESP-NOW (defined in .ino)
extern float last_throttle;       // Previous throttle value for ESP-NOW gating
extern float LOW_THROTTLE_THRESHOLD;  // (defined in balancing_robot.ino)


// ===== DATA STRUCTURE =====
// Must match the RC_Data structure in controller.ino
typedef struct {
  int throttle;           // Throttle input (-2048 to +2048)
  int yaw;                // Yaw input (-2048 to +2048)
  int roll;               // Roll input (-2048 to +2048)
  int pitch;              // Pitch input (-2048 to +2048)
  bool b1;                // Button 1 (arm/disarm)
  bool b2;                // Button 2 (status toggle)
  unsigned long timestamp; // Timestamp for latency calculation
} RC_Data;

// ===== GLOBAL ESPNOW VARIABLES =====
extern volatile bool espnow_connected;  // (definition in esp_now_handler.cpp)

// ===== ESP-NOW API (see esp_now_handler.cpp) =====
void onESPNOWReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
void onESPNOWSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void sendRSSIFeedback(const uint8_t *sender_mac);
void initESPNOW();
void sendDiscoveryPacket();
void updateESPNOWDataOnly();  // FAST PATH: lightweight, safe for 1kHz
void updateESPNOWService();   // SLOW PATH: heavy ops, run at ~10Hz from main loop
void updateESPNOWControl();   // DEPRECATED: calls updateESPNOWDataOnly()
bool isESPNOWConnected();
unsigned long getESPNOWLatency();
int getESPNOWRSSI();

#endif
