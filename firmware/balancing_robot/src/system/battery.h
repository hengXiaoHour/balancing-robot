#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "../config/settings.h"  // BATTERY_PIN, ADC_*, thresholds, LED pins
#include "../sensors/battery_state.h"  // BatteryState enum

// Battery globals (definitions in battery.cpp)
extern float battery_voltage;
extern float battery_voltage_filtered;
extern float battery_samples[BATTERY_SAMPLE_SIZE];
extern int battery_sample_index;
extern bool batteryFilterPrimed;
extern float lastValidBatteryRawVoltage;
extern BatteryState batteryState;
// (LED blink timing moved into led.cpp as function statics)

// Used by battery monitoring (defined elsewhere)
extern bool debugMonitoring;
extern volatile bool espnow_connected;
extern bool webSocketConnected;  // defined in comms/websocket_handler.cpp

// ===== Battery API (see battery.cpp) =====
void updateBatteryVoltage();
void updateBatteryMonitoring();

#endif
