#ifndef LED_H
#define LED_H

#include <Arduino.h>

// ===== Status LED helpers (see led.cpp) =====
// Board-specific pin + levels come from config/board.h.

// Boot self-test: blinks the LED, blocks ~1.5s. Call once from setup().
void ledBootTest();

// Steady output: true = ON, false = OFF.
void ledSet(bool on);

// Non-blocking 500ms blink poll for the low-battery state.
// Call every loop while BATTERY_LOW_CONFIRMED; owns its timing internally.
void ledPollBlink();

#endif  // LED_H
