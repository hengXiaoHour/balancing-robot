#ifndef WIFI_OTA_H
#define WIFI_OTA_H

#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== WiFi Configuration =====
// NOTE: real credentials live only in the local working copy, never in git.
#define WIFI_SSID "YOUR_WIFI_SSID"          // Change to your WiFi network name
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"  // Change to your WiFi password
#define AP_SSID "ESP32_QUAD"           // Access Point name when in AP mode
#define AP_PASSWORD "12345678"         // Access Point password

// WiFi mode (definitions in wifi_ota.cpp)
extern bool useAPMode;  // false = STA mode (default), true = AP mode
extern unsigned long wifiLastConnectionAttempt;

// Written by OTA start handler (definitions live elsewhere)
extern bool motorsArmed;
extern bool motorsActive;

// ===== WiFi/OTA API (see wifi_ota.cpp) =====
void initWiFi();
void handleWiFiConnection();
void setupOTA();
void printWiFiStatus();
void switchWiFiMode();  // Usage: Call switchWiFiMode() from serial commands

#endif
