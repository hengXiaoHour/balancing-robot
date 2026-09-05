#include "imu.h"

// Global IMU object
IMU_Custom mpu;

// MPU6050 initialization status
bool mpuInitialized = false;

// ===== IMU INITIALIZATION (moved from setup() verbatim) =====
void initIMU() {
  // Initialize I2C bus (used by MPU6050 path and optional barometer)
  Wire.begin(I2C_SDA, I2C_SCL);  // SDA, SCL from config
  Wire.setClock(I2C_SPEED);      // I2C speed from config

  // Initialize IMU
  delay(100);
  if (!mpu.initialize()) {
    #if ACTIVE_IMU == IMU_SENSOR_MPU6500_SPI
    Serial.println("[ERROR] MPU6500 (SPI) not found! Robot cannot be armed until sensor is properly connected and initialized.");
    #elif ACTIVE_IMU == IMU_SENSOR_MPU6500_I2C
    Serial.println("[ERROR] MPU6500 (I2C) not found! Robot cannot be armed until sensor is properly connected and initialized.");
    Serial.printf("[ERROR] I2C config -> SDA: GPIO %d, SCL: GPIO %d, Speed: %d Hz\n", I2C_SDA, I2C_SCL, I2C_SPEED);
    Serial.println("[ERROR] Check wiring + pull-ups, and confirm MPU address (0x68/0x69)");
    #else
    Serial.println("[ERROR] MPU6050 not found! Robot cannot be armed until sensor is properly connected and initialized.");
    Serial.printf("[ERROR] I2C config -> SDA: GPIO %d, SCL: GPIO %d, Speed: %d Hz\n", I2C_SDA, I2C_SCL, I2C_SPEED);
    Serial.println("[ERROR] Check wiring + pull-ups, and confirm MPU address (0x68/0x69)");
    #endif
    mpuInitialized = false;
  } else {
    #if ACTIVE_IMU == IMU_SENSOR_MPU6500_SPI
    Serial.println("[IMU] init ok (MPU6500 SPI)");
    #elif ACTIVE_IMU == IMU_SENSOR_MPU6500_I2C
    Serial.println("[IMU] init ok (MPU6500 I2C)");
    #else
    Serial.println("[IMU] init ok (MPU6050)");
    #endif
    mpuInitialized = true;
  }
  delay(500);
}

// ===== MPU INITIALIZATION RETRY (moved from balancing_robot.ino verbatim) =====
void retryMPUInitialization() {
  Serial.println("[INFO] Attempting to retry MPU initialization...");

  // Re-initialize I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // Attempt to re-initialize MPU object
  if (mpu.initialize()) {
    mpuInitialized = true;
    Serial.println("[OK] MPU successfully re-initialized");
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
