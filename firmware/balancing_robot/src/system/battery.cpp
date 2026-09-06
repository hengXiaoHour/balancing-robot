#include "battery.h"
#include "led.h"  // ledSet / ledPollBlink
#include "../config/pins_live.h"  // g_pin_BAT/LED live pins (NVS overrides)

// Definitions live here (were in balancing_robot.ino); externs in battery.h
float battery_voltage = 0.0f;
float battery_voltage_filtered = 0.0f;
float battery_samples[BATTERY_SAMPLE_SIZE] = {0};
int battery_sample_index = 0;
bool batteryFilterPrimed = false;
float lastValidBatteryRawVoltage = 0.0f;
BatteryState batteryState = BATTERY_NORMAL;

// ===== BATTERY VOLTAGE MANAGEMENT FUNCTIONS =====
void updateBatteryVoltage() {
  if (g_pin_BAT < 0) {
    // No battery sense pin: assume the supply is fine so the link LED
    // and arming logic treat power as healthy (typical for USB bench use).
    if (!batteryFilterPrimed) {
      for (int i = 0; i < BATTERY_SAMPLE_SIZE; i++) battery_samples[i] = 4.0f;
      battery_voltage_filtered = 4.0f;
      battery_voltage = 4.0f;
      lastValidBatteryRawVoltage = 4.0f;
      batteryFilterPrimed = true;
    }
    return;
  }
  int adc_raw = analogRead(g_pin_BAT);
  float raw_voltage = (adc_raw / (float)ADC_MAX) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;

  if (!batteryFilterPrimed) {
    for (int i = 0; i < BATTERY_SAMPLE_SIZE; i++) {
      battery_samples[i] = raw_voltage;
    }
    battery_voltage_filtered = raw_voltage;
    battery_voltage = raw_voltage;
    lastValidBatteryRawVoltage = raw_voltage;
    batteryFilterPrimed = true;
    return;
  }

  float raw_delta = raw_voltage - lastValidBatteryRawVoltage;
  float max_step_down = BATTERY_GLITCH_REJECT_V;
  float max_step_up = BATTERY_GLITCH_REJECT_V * 2.0f;
  if (raw_delta > max_step_up) {
    raw_voltage = lastValidBatteryRawVoltage + max_step_up;
  } else if (raw_delta < -max_step_down) {
    raw_voltage = lastValidBatteryRawVoltage - max_step_down;
  }
  lastValidBatteryRawVoltage = raw_voltage;

  battery_samples[battery_sample_index] = raw_voltage;
  battery_sample_index = (battery_sample_index + 1) % BATTERY_SAMPLE_SIZE;

  float voltage_sum = 0.0f;
  for (int i = 0; i < BATTERY_SAMPLE_SIZE; i++) {
    voltage_sum += battery_samples[i];
  }
  float voltage_average = voltage_sum / BATTERY_SAMPLE_SIZE;

  battery_voltage_filtered = (BATTERY_LPF_ALPHA * voltage_average) + ((1.0f - BATTERY_LPF_ALPHA) * battery_voltage_filtered);
  battery_voltage = battery_voltage_filtered;

  if (debugMonitoring) {
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 1000 && millis() < 10000) {
      Serial.print("[BATTERY DEBUG] Raw ADC: ");
      Serial.print(adc_raw);
      Serial.print(", Raw Voltage: ");
      Serial.print(raw_voltage, 3);
      Serial.print("V, Filtered: ");
      Serial.print(battery_voltage, 3);
      Serial.println("V");
      lastDebugTime = millis();
    }
  }
}

void updateBatteryMonitoring() {
  static unsigned long initStartTime = 0;
  if (initStartTime == 0) initStartTime = millis();

  if (battery_voltage < 3.0f || (millis() - initStartTime) < 2000) {
    ledSet(webSocketConnected);  // link LED works on USB power too (no battery)
    return;
  }

  static int lowVoltageCount = 0;

  if (debugMonitoring) {
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 2000 && millis() < 30000) {
      Serial.print("[LED DEBUG] ESP-NOW Connected: ");
      Serial.print(espnow_connected ? "YES" : "NO");
      Serial.print(", Battery State: ");
      Serial.print(batteryState == BATTERY_NORMAL ? "NORMAL" : "LOW");
      Serial.print(", LED should be: ");
      if (batteryState == BATTERY_LOW_CONFIRMED) {
        Serial.println("BLINKING");
      } else if (espnow_connected) {
        Serial.println("ON (connected)");
      } else {
        Serial.println("OFF (disconnected)");
      }
      lastDebugTime = millis();
    }
  }

  switch (batteryState) {
    case BATTERY_NORMAL:
      if (g_pin_LED >= 0) pinMode(g_pin_LED, OUTPUT);

      if (espnow_connected || webSocketConnected) {
        ledSet(true);
      } else {
        ledSet(false);
      }

      if (battery_voltage <= LOW_VOLTAGE_THRESHOLD) {
        lowVoltageCount++;
        if (lowVoltageCount >= 5) {
          batteryState = BATTERY_LOW_CONFIRMED;
          Serial.println("[BATTERY] LOW BATTERY DETECTED! Voltage has been 3.3V or below for 100ms. LED will blink until battery changed.");
          if (g_pin_LED >= 0) pinMode(g_pin_LED, OUTPUT);
          lowVoltageCount = 0;
        }
      } else {
        lowVoltageCount = 0;
      }
      break;

    case BATTERY_LOW_CONFIRMED:
      ledPollBlink();
      break;
  }
}
