#include "led.h"
#include "../config/config.h"  // LOW_VOLTAGE_LED_PIN, LED_*_LEVEL, STATUS_LED_RGB, LED_* colors

#ifdef STATUS_LED_RGB
// Addressable RGB (WS2812): driven by the core's built-in neopixelWrite,
// no extra library needed. Note GRB wire order is handled by neopixelWrite.
static void ledWriteRGB(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(LOW_VOLTAGE_LED_PIN, r, g, b);
}
#else
static void ledWriteRGB(uint8_t, uint8_t, uint8_t) {}  // no-op on plain-LED boards
#endif

void ledBootTest() {
  Serial.printf("[LED TEST] Testing LED on pin %d...\n", LOW_VOLTAGE_LED_PIN);
#ifdef STATUS_LED_RGB
  ledWriteRGB(LED_ON_R, LED_ON_G, LED_ON_B);  // ON (green)
  delay(500);
  ledWriteRGB(0, 0, 0);  // OFF
  delay(500);
  ledWriteRGB(LED_ON_R, LED_ON_G, LED_ON_B);  // ON again
  delay(500);
  ledWriteRGB(0, 0, 0);  // OFF
#else
  pinMode(LOW_VOLTAGE_LED_PIN, OUTPUT);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);  // LED ON
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL);  // LED OFF
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);  // LED ON again
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL);  // LED OFF
#endif
  Serial.println("[LED TEST] LED test complete");
}

void ledSet(bool on) {
#ifdef STATUS_LED_RGB
  if (on) ledWriteRGB(LED_ON_R, LED_ON_G, LED_ON_B);
  else ledWriteRGB(0, 0, 0);
#else
  digitalWrite(LOW_VOLTAGE_LED_PIN, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
#endif
}

void ledSetRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledWriteRGB(r, g, b);
}

void ledPollBlink() {
  static unsigned long lastLEDBlink = 0;
  static bool ledState = false;
  unsigned long now = millis();
  if (now - lastLEDBlink >= 500) {
    ledState = !ledState;
#ifdef STATUS_LED_RGB
    if (ledState) ledWriteRGB(LED_BLINK_R, LED_BLINK_G, LED_BLINK_B);  // red
    else ledWriteRGB(0, 0, 0);
#else
    ledSet(ledState);
#endif
    lastLEDBlink = now;
  }
}
