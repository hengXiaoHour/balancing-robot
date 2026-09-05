#include <Arduino.h>
#include "wifi_ota.h"
#include "../control/motor_control.h"  // stopMotors()

// WiFi mode
bool useAPMode = false;  // false = STA mode (default), true = AP mode
unsigned long wifiLastConnectionAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 5000;  // Try to reconnect every 5 seconds

// ===== Initialize WiFi (STA Mode by Default) =====
void initWiFi() {
  if (useAPMode) {
    // AP Mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[WIFI] AP ");
    Serial.print(AP_SSID);
    Serial.print(" at ");
    Serial.println(WiFi.softAPIP());
  } else {
    // STA Mode (default)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WIFI] connecting to ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
    wifiLastConnectionAttempt = millis();
  }

  delay(500);
  setupOTA();
}

// ===== Handle WiFi Connection (STA Mode) =====
void handleWiFiConnection() {
  if (useAPMode) {
    // In AP mode, always ready
    return;
  }

  // In STA mode, announce the IP once per connection; retry silently
  static bool wifiAnnounced = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiAnnounced) {
      wifiAnnounced = true;
      Serial.print("[WIFI] connected at ");
      Serial.println(WiFi.localIP());
    }
    return;
  }
  wifiAnnounced = false;

  // Not connected, try to reconnect periodically
  if (millis() - wifiLastConnectionAttempt > WIFI_RECONNECT_INTERVAL) {
    WiFi.reconnect();
    wifiLastConnectionAttempt = millis();
  }
}

// ===== Setup OTA Updates =====
void setupOTA() {
  ArduinoOTA.setHostname("ESP32_BalancingRobot");

  // No password for local network OTA (more reliable than dynamic password)
  // If you need security, use firewall or VPN instead
  // ArduinoOTA.setPassword("admin");  // Optional: uncomment and set fixed password if needed

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "FLASH" : "SPIFFS";
    Serial.println("\n[OTA] Start updating " + type);
    motorsArmed = false;
    motorsActive = false;
    stopMotors();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.print(".");
    if (progress % 40 == 0) Serial.println();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print("[OTA ERROR] ");
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed - wrong password?");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] ready");
}

// ===== Print WiFi Status =====
void printWiFiStatus() {
  Serial.println("\n===== WiFi Status =====");

  if (useAPMode) {
    Serial.println("Mode: AP (Access Point)");
    Serial.print("SSID: "); Serial.println(AP_SSID);
    Serial.print("IP: "); Serial.println(WiFi.softAPIP());
    Serial.print("Clients: "); Serial.println(WiFi.softAPgetStationNum());
    Serial.print("OTA Password: "); Serial.println(AP_PASSWORD);
  } else {
    Serial.println("Mode: STA (Station)");
    Serial.print("SSID: "); Serial.println(WIFI_SSID);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Status: CONNECTED");
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    } else {
      Serial.println("Status: DISCONNECTED");
      Serial.print("Attempting to reconnect...");
    }
    Serial.print("OTA Password: "); Serial.println(WIFI_PASSWORD);
  }

  Serial.println("OTA: Ready for updates");
  Serial.println("Use Arduino IDE Tools > Port > Network Ports to flash");
  Serial.println("========================\n");
}

// ===== Switch WiFi Mode (STA <-> AP) =====
// Usage: Call switchWiFiMode() from serial commands
void switchWiFiMode() {
  useAPMode = !useAPMode;

  WiFi.disconnect(true);  // Disconnect and turn off WiFi
  delay(500);

  initWiFi();
  printWiFiStatus();

  if (useAPMode) {
    Serial.println("[WiFi] Switched to AP mode");
  } else {
    Serial.println("[WiFi] Switched to STA mode");
  }
}
