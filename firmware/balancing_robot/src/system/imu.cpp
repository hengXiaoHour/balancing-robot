#include "imu.h"
#include "../config/pins_live.h"  // g_pin_SDA/SCL live pins (NVS overrides)

// One static object per driver; mpu points at the winner.
static MPU6500_SPI_Custom imuSpi;
static MPU6050_Custom imu6050;
static MPU6500_I2C_Custom imu6500i2c;

IMU_Driver* mpu = nullptr;

// MPU6050 initialization status
bool mpuInitialized = false;

const char* activeImuName() {
  return mpu ? mpu->driverName() : "none";
}

// Probe order: SPI first (fast bus, exact 0x70 match), then I2C by WHO_AM_I
// (0x68/0x69 = MPU6050, 0x70 = MPU6500 — ACK alone is not trusted since 0x68
// is shared with other chip families). Each initialize() is side-effect free
// on mismatch: it returns false before configuring anything.
static IMU_Driver* probeImu() {
#if ACTIVE_IMU == IMU_SENSOR_AUTO || ACTIVE_IMU == IMU_SENSOR_MPU6500_SPI
  if (imuSpi.initialize()) return &imuSpi;
#if ACTIVE_IMU == IMU_SENSOR_MPU6500_SPI
  return nullptr;  // forced SPI, no fallthrough
#endif
#endif
#if ACTIVE_IMU == IMU_SENSOR_AUTO || ACTIVE_IMU == IMU_SENSOR_MPU6050
  if (imu6050.initialize()) return &imu6050;
#if ACTIVE_IMU == IMU_SENSOR_MPU6050
  return nullptr;  // forced MPU6050, no fallthrough
#endif
#endif
#if ACTIVE_IMU == IMU_SENSOR_AUTO || ACTIVE_IMU == IMU_SENSOR_MPU6500_I2C
  if (imu6500i2c.initialize()) return &imu6500i2c;
#endif
  return nullptr;
}

// ===== IMU INITIALIZATION (moved from setup() verbatim) =====
void initIMU() {
  // Initialize I2C bus (used by MPU6050 path and optional barometer)
  Wire.begin(g_pin_SDA, g_pin_SCL);  // SDA, SCL from config
  Wire.setClock(I2C_SPEED);      // I2C speed from config

  // Initialize IMU
  delay(100);
  mpu = probeImu();
  if (!mpu) {
    Serial.println("[ERROR] No IMU found! Robot cannot be armed until sensor is properly connected and initialized.");
#if ACTIVE_IMU == IMU_SENSOR_AUTO
    Serial.println("[ERROR] Auto-detect probed SPI (WHO_AM_I=0x70) then I2C 0x68/0x69 by WHO_AM_I");
#else
    Serial.printf("[ERROR] Forced driver ACTIVE_IMU=%d did not answer\n", ACTIVE_IMU);
#endif
    Serial.printf("[ERROR] I2C config -> SDA: GPIO %d, SCL: GPIO %d, Speed: %d Hz\n", g_pin_SDA, g_pin_SCL, I2C_SPEED);
    Serial.printf("[ERROR] SPI config -> SCK: GPIO %d, MOSI: GPIO %d, MISO: GPIO %d, CS: GPIO %d\n",
                  g_pin_SPI_SCK, g_pin_SPI_MOSI, g_pin_SPI_MISO, g_pin_SPI_CS);
    Serial.println("[ERROR] Check wiring + pull-ups, and confirm MPU address (0x68/0x69)");
    mpuInitialized = false;
  } else {
#if ACTIVE_IMU == IMU_SENSOR_AUTO
    Serial.printf("[IMU] init ok (%s, auto)\n", mpu->driverName());
#else
    Serial.printf("[IMU] init ok (%s, forced)\n", mpu->driverName());
#endif
    mpuInitialized = true;
  }
  delay(500);
}

// ===== MPU INITIALIZATION RETRY (moved from balancing_robot.ino verbatim) =====
void retryMPUInitialization() {
  Serial.println("[INFO] Attempting to retry MPU initialization...");

  // Re-initialize I2C bus
  Wire.begin(g_pin_SDA, g_pin_SCL);
  delay(100);

  // Re-probe (auto) or re-init (forced)
  mpu = probeImu();
  if (mpu) {
    mpuInitialized = true;
    Serial.printf("[OK] IMU re-initialized (%s)\n", mpu->driverName());
    filterInitialized = false;  // Reset filter to warm up from next reading
    Serial.println("[INFO] Filter state reset - will warm up from next reading");
  } else {
    Serial.println("[ERROR] MPU re-initialization failed - check sensor connection");
    mpuInitialized = false;
  }
}

// ===== Periodic retry poll: call from loop() (moved from .ino verbatim) =====
void pollMpuRetry() {
  static unsigned long lastMPURetry = 0;
  if (!mpuInitialized && (millis() - lastMPURetry > 30000)) {
    lastMPURetry = millis();
    retryMPUInitialization();
  }
}

// ===== IMU status for CLI 'imu show' =====
void printImuStatus() {
  Serial.println("--- imu (live) ---");
#if ACTIVE_IMU == IMU_SENSOR_AUTO
  Serial.println("mode   : auto-detect (SPI first, then I2C by WHO_AM_I)");
#else
  Serial.printf("mode   : forced ACTIVE_IMU=%d\n", ACTIVE_IMU);
#endif
  Serial.printf("active : %s\n", activeImuName());
  Serial.printf("i2c    : SDA=%d SCL=%d\n", g_pin_SDA, g_pin_SCL);
  Serial.printf("spi    : SCK=%d MOSI=%d MISO=%d CS=%d\n",
                g_pin_SPI_SCK, g_pin_SPI_MOSI, g_pin_SPI_MISO, g_pin_SPI_CS);
}
