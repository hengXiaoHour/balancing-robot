#ifndef COMMS_MODE_H
#define COMMS_MODE_H

#include <Arduino.h>

// ===== Runtime comms selection (NVS "comms_cfg") =====
// Picks the control link at boot: onboard WebSocket UI (STA/AP WiFi) or an
// external ESP-NOW controller. settings.h stays the seed/defaults source:
// ENABLE_ESPNOW seeds mode, CONTROLLER_MAC_* seed the peer MAC, STA seeds
// the WiFi side. loadCommsFromNVS() overlays NVS values — call it in
// setup() before initWiFi()/initESPNOW(). Reboot applies.
extern bool g_comms_espnow;      // true = ESP-NOW link, false = WebSocket UI
extern uint8_t g_peer_mac[6];    // ESP-NOW controller MAC (discovery peer)
extern bool g_comms_ap;          // WS side: true = AP hotspot, false = STA join

void loadCommsFromNVS();
void saveCommsToNVS();
void resetCommsToDefaults();
bool commsHasNvsOverrides();

// Convenience for setup()/loop() branches (replaces #if ENABLE_ESPNOW).
inline bool commsUseEspNow() { return g_comms_espnow; }

// Stage one value in RAM (validated). Name already lowercased by dispatcher.
// mode: ws|espnow, mac: AA:BB:CC:DD:EE:FF, wifimode: sta|ap.
// Returns false + err on reject.
bool setStagedComms(const String& name, const String& value, String& err);
void printCommsToSerial();  // 'comms show' listing

#endif
