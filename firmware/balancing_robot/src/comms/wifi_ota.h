#ifndef WIFI_OTA_H
#define WIFI_OTA_H

#include <WiFi.h>
#include <ArduinoOTA.h>
#include "../config/settings.h"  // WIFI_*/AP_* defaults (live values in wifi_creds.h, NVS overrides)

// Fallbacks only if settings.h is ever bypassed — normal builds take values from settings.h.
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif
#ifndef AP_SSID
#define AP_SSID "ESP32_BALANCING"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "12345678"
#endif

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
