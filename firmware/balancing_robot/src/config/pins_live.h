#ifndef PINS_LIVE_H
#define PINS_LIVE_H

#include <Arduino.h>

// ===== Runtime pin overrides (NVS "board_pins") =====
// Defaults come from board_pins.h via ACTIVE_BOARD in settings.h.
// loadPinsFromNVS() copies NVS overrides over these at boot — call it
// FIRST in setup(), before any peripheral init. settings.h stays the
// sole place for compile-time defaults; NVS only stores deltas.
extern int g_pin_ENA;
extern int g_pin_IN1;
extern int g_pin_IN2;
extern int g_pin_ENB;
extern int g_pin_IN3;
extern int g_pin_IN4;
extern int g_pin_SDA;
extern int g_pin_SCL;
extern int g_pin_BAT;
extern int g_pin_LED;
extern int g_pin_SPI_SCK;
extern int g_pin_SPI_MOSI;
extern int g_pin_SPI_MISO;
extern int g_pin_SPI_CS;

void loadPinsFromNVS();    // boot: defaults + NVS overrides
void savePinsToNVS();      // persist staged RAM values
void resetPinsToDefaults();  // clear NVS, restore RAM to board_pins.h
bool pinsHaveNvsOverrides();  // true if NVS holds at least one pin key

// Stage one pin in RAM (validated). Returns false + err on reject.
bool setStagedPin(const String& name, int gpio, String& err);
void printPinsToSerial();  // 'pins show' listing

#endif
