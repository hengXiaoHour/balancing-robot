#include "wifi_creds.h"
#include <Preferences.h>
#include "../config/settings.h"  // WIFI_SSID / WIFI_PASSWORD / AP_SSID / AP_PASSWORD defaults

String g_wifi_ssid = WIFI_SSID;
String g_wifi_pass = WIFI_PASSWORD;
String g_ap_ssid = AP_SSID;
String g_ap_pass = AP_PASSWORD;

static void seedDefaults() {
  g_wifi_ssid = WIFI_SSID;
  g_wifi_pass = WIFI_PASSWORD;
  g_ap_ssid = AP_SSID;
  g_ap_pass = AP_PASSWORD;
}

bool wifiHasNvsOverrides() {
  Preferences p;
  if (!p.begin("wifi_cfg", true)) return false;
  bool has = p.isKey("sta_ssid");
  p.end();
  return has;
}

void loadWifiFromNVS() {
  seedDefaults();
  Preferences p;
  if (!p.begin("wifi_cfg", true)) return;
  g_wifi_ssid = p.getString("sta_ssid", g_wifi_ssid);
  g_wifi_pass = p.getString("sta_pass", g_wifi_pass);
  g_ap_ssid = p.getString("ap_ssid", g_ap_ssid);
  g_ap_pass = p.getString("ap_pass", g_ap_pass);
  p.end();
}

void saveWifiToNVS() {
  Preferences p;
  p.begin("wifi_cfg", false);
  p.putString("sta_ssid", g_wifi_ssid);
  p.putString("sta_pass", g_wifi_pass);
  p.putString("ap_ssid", g_ap_ssid);
  p.putString("ap_pass", g_ap_pass);
  p.end();
  delay(100);  // NVS flush on ESP32-C3
}

void resetWifiToDefaults() {
  Preferences p;
  p.begin("wifi_cfg", false);
  p.clear();
  p.end();
  delay(50);
  seedDefaults();
}

bool setStagedWifi(const String& name, const String& value, String& err) {
  if (name == "ssid" || name == "sta_ssid" || name == "sta") {
    if (value.length() < 1 || value.length() > 32) { err = "SSID must be 1-32 chars"; return false; }
    g_wifi_ssid = value;
    return true;
  }
  if (name == "pass" || name == "password" || name == "sta_pass") {
    if (value.length() > 64) { err = "password max 64 chars"; return false; }
    if (value.length() > 0 && value.length() < 8) { err = "WPA password needs 8+ chars (or empty for open)"; return false; }
    g_wifi_pass = value;
    return true;
  }
  if (name == "ap_ssid" || name == "apssid") {
    if (value.length() < 1 || value.length() > 32) { err = "AP SSID must be 1-32 chars"; return false; }
    g_ap_ssid = value;
    return true;
  }
  if (name == "ap_pass" || name == "appass" || name == "ap_password") {
    if (value.length() > 0 && value.length() < 8) { err = "AP password needs 8+ chars (or empty for open AP)"; return false; }
    if (value.length() > 64) { err = "password max 64 chars"; return false; }
    g_ap_pass = value;
    return true;
  }
  err = "unknown field. use: ssid | pass | ap_ssid | ap_pass";
  return false;
}

static void printMaskedPass(const String& pass) {
  if (pass.length() == 0) Serial.print("<open>");
  else Serial.print("********");
}

void printWifiConfigMasked() {
  Serial.println("--- wifi (live) ---");
  Serial.print("sta : ssid=\""); Serial.print(g_wifi_ssid);
  Serial.print("\" pass="); printMaskedPass(g_wifi_pass); Serial.println();
  Serial.print("ap  : ssid=\""); Serial.print(g_ap_ssid);
  Serial.print("\" pass="); printMaskedPass(g_ap_pass); Serial.println();
  Serial.printf("source : %s\n", wifiHasNvsOverrides() ? "NVS overrides" : "defaults (settings.h)");
}
