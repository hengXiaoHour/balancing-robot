#ifndef WIFI_CREDS_H
#define WIFI_CREDS_H

#include <Arduino.h>

// ===== Runtime WiFi credentials (NVS "wifi_cfg") =====
// Defaults come from settings.h (WIFI_SSID / WIFI_PASSWORD / AP_SSID /
// AP_PASSWORD). loadWifiFromNVS() overlays NVS values at boot — call it
// before initWiFi(). Staged via CLI ('wifi set ...'), persisted with
// 'wifi save'. Passwords are never printed; 'wifi show' masks them.
extern String g_wifi_ssid;
extern String g_wifi_pass;
extern String g_ap_ssid;
extern String g_ap_pass;

void loadWifiFromNVS();
void saveWifiToNVS();
void resetWifiToDefaults();
bool wifiHasNvsOverrides();

// Stage one value in RAM (validated). Name already lowercased by dispatcher,
// value keeps original case. Returns false + err on reject.
bool setStagedWifi(const String& name, const String& value, String& err);
void printWifiConfigMasked();

#endif
