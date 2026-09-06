#include "comms_mode.h"
#include <Preferences.h>
#include "../config/settings.h"  // ENABLE_ESPNOW + CONTROLLER_MAC_* seeds

// Live values seed from compile-time defaults, NVS overrides after.
bool g_comms_espnow = (ENABLE_ESPNOW != 0);
uint8_t g_peer_mac[6] = {CONTROLLER_MAC_0, CONTROLLER_MAC_1, CONTROLLER_MAC_2,
                         CONTROLLER_MAC_3, CONTROLLER_MAC_4, CONTROLLER_MAC_5};
bool g_comms_ap = false;  // STA join by default

static void seedDefaults() {
  g_comms_espnow = (ENABLE_ESPNOW != 0);
  const uint8_t seed[6] = {CONTROLLER_MAC_0, CONTROLLER_MAC_1, CONTROLLER_MAC_2,
                           CONTROLLER_MAC_3, CONTROLLER_MAC_4, CONTROLLER_MAC_5};
  memcpy(g_peer_mac, seed, 6);
  g_comms_ap = false;
}

static void macToStr(const uint8_t m[6], char out[18]) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           m[0], m[1], m[2], m[3], m[4], m[5]);
}

static bool strToMac(const String& s, uint8_t m[6]) {
  unsigned v[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",
             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) {
    if (v[i] > 0xFF) return false;
    m[i] = (uint8_t)v[i];
  }
  return true;
}

bool commsHasNvsOverrides() {
  Preferences p;
  if (!p.begin("comms_cfg", true)) return false;
  bool has = p.isKey("mode");
  p.end();
  return has;
}

void loadCommsFromNVS() {
  seedDefaults();
  Preferences p;
  if (!p.begin("comms_cfg", true)) return;  // no namespace yet -> defaults
  String mode = p.getString("mode", g_comms_espnow ? "espnow" : "ws");
  mode.toLowerCase();
  g_comms_espnow = (mode == "espnow");
  uint8_t m[6];
  if (strToMac(p.getString("mac", ""), m)) memcpy(g_peer_mac, m, 6);
  String wm = p.getString("wifimode", "sta");
  wm.toLowerCase();
  g_comms_ap = (wm == "ap");
  p.end();
}

void saveCommsToNVS() {
  Preferences p;
  p.begin("comms_cfg", false);
  p.putString("mode", g_comms_espnow ? "espnow" : "ws");
  char mac[18];
  macToStr(g_peer_mac, mac);
  p.putString("mac", mac);
  p.putString("wifimode", g_comms_ap ? "ap" : "sta");
  p.end();
  delay(100);  // NVS flush on ESP32-C3
}

void resetCommsToDefaults() {
  Preferences p;
  p.begin("comms_cfg", false);
  p.clear();
  p.end();
  delay(50);
  seedDefaults();
}

bool setStagedComms(const String& name, const String& value, String& err) {
  String v = value;
  v.trim();
  v.toLowerCase();
  if (name == "mode") {
    if (v == "ws" || v == "websocket") { g_comms_espnow = false; return true; }
    if (v == "espnow" || v == "esp-now") { g_comms_espnow = true; return true; }
    err = "mode must be ws or espnow";
    return false;
  }
  if (name == "mac" || name == "peer") {
    uint8_t m[6];
    if (!strToMac(value, m)) { err = "mac must be AA:BB:CC:DD:EE:FF"; return false; }
    memcpy(g_peer_mac, m, 6);
    return true;
  }
  if (name == "wifimode" || name == "wifi") {
    if (v == "sta") { g_comms_ap = false; return true; }
    if (v == "ap") { g_comms_ap = true; return true; }
    err = "wifimode must be sta or ap (sta by default)";
    return false;
  }
  err = "unknown key. keys: mode mac wifimode";
  return false;
}

void printCommsToSerial() {
  char mac[18];
  macToStr(g_peer_mac, mac);
  Serial.println("--- comms (live) ---");
  Serial.printf("link    : %s\n", g_comms_espnow ? "espnow (external controller)" : "ws (WebUI :80/:81)");
  Serial.printf("peer    : %s\n", mac);
  Serial.printf("wifimode: %s\n", g_comms_ap ? "ap (hotspot)" : "sta (join, default)");
  Serial.printf("source  : %s\n", commsHasNvsOverrides() ? "NVS overrides" : "defaults");
}
