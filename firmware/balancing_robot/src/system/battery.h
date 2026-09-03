#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "../config/config.h"  // BATTERY_PIN, ADC_*, thresholds, LED pins
#include "../sensors/battery_state.h"  // BatteryState enum

// Battery globals (definitions in balancing_robot.ino)
extern float battery_voltage;
extern float battery_voltage_filtered;
extern float battery_samples[BATTERY_SAMPLE_SIZE];
extern int battery_sample_index;
extern bool batteryFilterPrimed;
extern float lastValidBatteryRawVoltage;
extern BatteryState batteryState;
extern unsigned long lastLEDBlink;
extern bool ledState;

// Used by battery monitoring (defined elsewhere)
extern bool debugMonitoring;
extern volatile bool espnow_connected;

// ===== Battery API (see battery.cpp) =====
void updateBatteryVoltage();
void updateBatteryMonitoring();

#endif
