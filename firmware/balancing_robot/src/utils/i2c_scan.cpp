#include <Arduino.h>
#include <Wire.h>
#include "i2c_scan.h"
#include "../config/settings.h"  // (kept for I2C_SPEED etc)
#include "../config/pins_live.h"  // g_pin_SDA/SCL live pins (NVS overrides)

// Device name lookup helper
const char* getI2CDeviceName(uint8_t address) {
  switch(address) {
    case 0x68: return "MPU6050 (IMU)";
    case 0x69: return "MPU6050 (IMU, ALT ADDR)";
    case 0x76: return "BMP280/BME280 (Barometer)";
    case 0x77: return "BMP180 (Barometer)";
    default: return "Unknown device";
  }
}

// Scan I2C bus and report all active addresses
void scanI2CBus() {
  Serial.println("\n========================================");
  Serial.println("        I2C Bus Scan - Active Devices");
  Serial.println("========================================");
  Serial.printf("I2C Config - SDA: GPIO %d, SCL: GPIO %d\n\n", g_pin_SDA, g_pin_SCL);

  byte error;
  uint8_t address;
  int nDevices = 0;

  Serial.println("Address | Device Name");
  Serial.println("--------|------------------------------");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("  0x%02X  | %s\n", address, getI2CDeviceName(address));
      nDevices++;
    } else if (error == 4) {
      Serial.printf("  0x%02X  | [ERROR: Unknown issue]\n", address);
    }
  }

  Serial.println("--------|------------------------------");
  if (nDevices == 0) {
    Serial.println("\n[WARN] NO I2C DEVICES FOUND!");
    Serial.println("Check circuit, pull-ups, SDA/SCL connections, and power.");
  } else {
    Serial.printf("\n[OK] Found %d I2C device(s)\n", nDevices);
  }
  Serial.println("========================================\n");
}
