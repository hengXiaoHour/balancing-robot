#ifndef BATTERY_STATE_H
#define BATTERY_STATE_H

// ===== Battery Low Voltage Monitoring =====
#define LOW_VOLTAGE_THRESHOLD 3.5f  // Voltage threshold for low battery detection
enum BatteryState {
  BATTERY_NORMAL,        // Normal operation
  BATTERY_LOW_CONFIRMED  // Low battery detected - LED blinking
};

#endif
