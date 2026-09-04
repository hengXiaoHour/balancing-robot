#include "led.h"
#include "../config/config.h"  // LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL, LED_OFF_LEVEL

void ledBootTest() {
  Serial.printf("[LED TEST] Testing LED on pin %d...\n", LOW_VOLTAGE_LED_PIN);
  pinMode(LOW_VOLTAGE_LED_PIN, OUTPUT);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);  // LED ON
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL);  // LED OFF
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_ON_LEVEL);  // LED ON again
  delay(500);
  digitalWrite(LOW_VOLTAGE_LED_PIN, LED_OFF_LEVEL);  // LED OFF
  Serial.println("[LED TEST] LED test complete");
}

void ledSet(bool on) {
  digitalWrite(LOW_VOLTAGE_LED_PIN, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

void ledPollBlink() {
  static unsigned long lastLEDBlink = 0;
  static bool ledState = false;
  unsigned long now = millis();
  if (now - lastLEDBlink >= 500) {
    ledState = !ledState;
    ledSet(ledState);
    lastLEDBlink = now;
  }
}
