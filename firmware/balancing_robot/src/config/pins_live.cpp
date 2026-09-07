#include "pins_live.h"
#include <Preferences.h>
#include "settings.h"  // ENA/INx/I2C_*/BATTERY_PIN/LOW_VOLTAGE_LED_PIN/IMU_SPI_* defaults

// Live values seed from compile-time board profile, NVS overrides after.
int g_pin_ENA = ENA;
int g_pin_IN1 = IN1;
int g_pin_IN2 = IN2;
int g_pin_ENB = ENB;
int g_pin_IN3 = IN3;
int g_pin_IN4 = IN4;
int g_pin_SDA = I2C_SDA;
int g_pin_SCL = I2C_SCL;
int g_pin_BAT = BATTERY_PIN;
int g_pin_LED = LOW_VOLTAGE_LED_PIN;
int g_pin_SPI_SCK = IMU_SPI_SCK_PIN;
int g_pin_SPI_MOSI = IMU_SPI_MOSI_PIN;
int g_pin_SPI_MISO = IMU_SPI_MISO_PIN;
int g_pin_SPI_CS = IMU_SPI_CS_PIN;

static void seedDefaults() {
  g_pin_ENA = ENA;
  g_pin_IN1 = IN1;
  g_pin_IN2 = IN2;
  g_pin_ENB = ENB;
  g_pin_IN3 = IN3;
  g_pin_IN4 = IN4;
  g_pin_SDA = I2C_SDA;
  g_pin_SCL = I2C_SCL;
  g_pin_BAT = BATTERY_PIN;
  g_pin_LED = LOW_VOLTAGE_LED_PIN;
  g_pin_SPI_SCK = IMU_SPI_SCK_PIN;
  g_pin_SPI_MOSI = IMU_SPI_MOSI_PIN;
  g_pin_SPI_MISO = IMU_SPI_MISO_PIN;
  g_pin_SPI_CS = IMU_SPI_CS_PIN;
}

bool pinsHaveNvsOverrides() {
  Preferences p;
  if (!p.begin("board_pins", true)) return false;
  bool has = p.isKey("ENA");
  p.end();
  return has;
}

void loadPinsFromNVS() {
  seedDefaults();
  Preferences p;
  if (!p.begin("board_pins", true)) return;  // no namespace yet -> defaults
  g_pin_ENA = p.getInt("ENA", g_pin_ENA);
  g_pin_IN1 = p.getInt("IN1", g_pin_IN1);
  g_pin_IN2 = p.getInt("IN2", g_pin_IN2);
  g_pin_ENB = p.getInt("ENB", g_pin_ENB);
  g_pin_IN3 = p.getInt("IN3", g_pin_IN3);
  g_pin_IN4 = p.getInt("IN4", g_pin_IN4);
  g_pin_SDA = p.getInt("SDA", g_pin_SDA);
  g_pin_SCL = p.getInt("SCL", g_pin_SCL);
  g_pin_BAT = p.getInt("BAT", g_pin_BAT);
  g_pin_LED = p.getInt("LED", g_pin_LED);
  g_pin_SPI_SCK = p.getInt("SCK", g_pin_SPI_SCK);
  g_pin_SPI_MOSI = p.getInt("MOSI", g_pin_SPI_MOSI);
  g_pin_SPI_MISO = p.getInt("MISO", g_pin_SPI_MISO);
  g_pin_SPI_CS = p.getInt("CS", g_pin_SPI_CS);
  p.end();
}

void savePinsToNVS() {
  Preferences p;
  p.begin("board_pins", false);
  p.putInt("ENA", g_pin_ENA);
  p.putInt("IN1", g_pin_IN1);
  p.putInt("IN2", g_pin_IN2);
  p.putInt("ENB", g_pin_ENB);
  p.putInt("IN3", g_pin_IN3);
  p.putInt("IN4", g_pin_IN4);
  p.putInt("SDA", g_pin_SDA);
  p.putInt("SCL", g_pin_SCL);
  p.putInt("BAT", g_pin_BAT);
  p.putInt("LED", g_pin_LED);
  p.putInt("SCK", g_pin_SPI_SCK);
  p.putInt("MOSI", g_pin_SPI_MOSI);
  p.putInt("MISO", g_pin_SPI_MISO);
  p.putInt("CS", g_pin_SPI_CS);
  p.end();
  delay(100);  // NVS flush on ESP32-C3
}

void resetPinsToDefaults() {
  Preferences p;
  p.begin("board_pins", false);
  p.clear();
  p.end();
  delay(50);
  seedDefaults();
}

// Resolve CLI name (already lowercased by dispatcher) to a live pin slot.
static int* slotFor(const String& n) {
  if (n == "ena") return &g_pin_ENA;
  if (n == "in1") return &g_pin_IN1;
  if (n == "in2") return &g_pin_IN2;
  if (n == "enb") return &g_pin_ENB;
  if (n == "in3") return &g_pin_IN3;
  if (n == "in4") return &g_pin_IN4;
  if (n == "sda" || n == "i2c_sda") return &g_pin_SDA;
  if (n == "scl" || n == "i2c_scl") return &g_pin_SCL;
  if (n == "bat" || n == "battery" || n == "battery_pin") return &g_pin_BAT;
  if (n == "led") return &g_pin_LED;
  if (n == "sck") return &g_pin_SPI_SCK;
  if (n == "mosi") return &g_pin_SPI_MOSI;
  if (n == "miso") return &g_pin_SPI_MISO;
  if (n == "cs") return &g_pin_SPI_CS;
  return nullptr;
}

static bool isMotorPin(const int* s) {
  return s == &g_pin_ENA || s == &g_pin_IN1 || s == &g_pin_IN2 ||
         s == &g_pin_ENB || s == &g_pin_IN3 || s == &g_pin_IN4;
}

static bool isSpiPin(const int* s) {
  return s == &g_pin_SPI_SCK || s == &g_pin_SPI_MOSI ||
         s == &g_pin_SPI_MISO || s == &g_pin_SPI_CS;
}

bool setStagedPin(const String& name, int gpio, String& err) {
  int* slot = slotFor(name);
  if (!slot) {
    err = "unknown pin. names: ENA IN1 IN2 ENB IN3 IN4 SDA SCL BAT LED SCK MOSI MISO CS";
    return false;
  }
  // -1 = unused: allowed for optional pins (SPI block, BAT, LED).
  // Motors and I2C are required, so they reject -1.
  bool optional = isSpiPin(slot) || slot == &g_pin_BAT || slot == &g_pin_LED;
  if (gpio == -1 && !optional) { err = "that pin is required (only SPI/BAT/LED accept -1 = unused)"; return false; }
  if (gpio < -1 || gpio > 48) { err = "gpio out of range (-1 = unused, 0-48)"; return false; }
#if ACTIVE_BOARD == BOARD_PROFILE_ESP32C3
  // C3 has no GPIO 11-19; only 0-10, 20, 21 exist.
  if ((gpio >= 11 && gpio <= 19) || (gpio > 21 && gpio != 48)) {
    // 48 kept legal as a no-op guard value some code uses; real C3 max is 21.
    if (gpio != 48) { err = "ESP32-C3 has no such GPIO (valid: 0-10, 20, 21)"; return false; }
  }
#elif ACTIVE_BOARD == BOARD_PROFILE_ESP32
  if (gpio > 39) { err = "ESP32 GPIOs are 0-39"; return false; }
  // 34-39 are input-only: fine for BAT, fatal for outputs.
  if (gpio >= 34 && slot != &g_pin_BAT) { err = "GPIO 34-39 are input-only on ESP32"; return false; }
#endif
  // SDA/SCL must differ.
  if ((slot == &g_pin_SDA && gpio == g_pin_SCL) ||
      (slot == &g_pin_SCL && gpio == g_pin_SDA)) {
    err = "SDA and SCL must differ";
    return false;
  }
  // Motor block must stay collision-free.
  if (isMotorPin(slot)) {
    const int* m[] = {&g_pin_ENA, &g_pin_IN1, &g_pin_IN2, &g_pin_ENB, &g_pin_IN3, &g_pin_IN4};
    for (auto p : m) {
      if (p != slot && gpio != -1 && *p == gpio) { err = "motor pins must be distinct"; return false; }
    }
  }
  // SPI block must stay collision-free (-1 = unused, never collides).
  if (isSpiPin(slot)) {
    const int* s[] = {&g_pin_SPI_SCK, &g_pin_SPI_MOSI, &g_pin_SPI_MISO, &g_pin_SPI_CS};
    for (auto p : s) {
      if (p != slot && gpio != -1 && *p == gpio) { err = "SPI pins must be distinct"; return false; }
    }
  }
  *slot = gpio;
  return true;
}

void printPinsToSerial() {
  Serial.println("--- pins (live) ---");
  Serial.printf("motors : ENA=%d IN1=%d IN2=%d ENB=%d IN3=%d IN4=%d\n",
                g_pin_ENA, g_pin_IN1, g_pin_IN2, g_pin_ENB, g_pin_IN3, g_pin_IN4);
  Serial.printf("i2c    : SDA=%d SCL=%d\n", g_pin_SDA, g_pin_SCL);
  Serial.printf("misc   : BAT=%d LED=%d\n", g_pin_BAT, g_pin_LED);
  Serial.printf("spi    : SCK=%d MOSI=%d MISO=%d CS=%d\n",
                g_pin_SPI_SCK, g_pin_SPI_MOSI, g_pin_SPI_MISO, g_pin_SPI_CS);
  Serial.printf("edit   : pin set ENA=%d IN1=%d IN2=%d ENB=%d IN3=%d IN4=%d SDA=%d SCL=%d BAT=%d LED=%d SCK=%d MOSI=%d MISO=%d CS=%d\n",
                g_pin_ENA, g_pin_IN1, g_pin_IN2, g_pin_ENB, g_pin_IN3, g_pin_IN4,
                g_pin_SDA, g_pin_SCL, g_pin_BAT, g_pin_LED,
                g_pin_SPI_SCK, g_pin_SPI_MOSI, g_pin_SPI_MISO, g_pin_SPI_CS);
  Serial.printf("source : %s (board profile %d)\n",
                pinsHaveNvsOverrides() ? "NVS overrides" : "defaults",
                ACTIVE_BOARD);
}
