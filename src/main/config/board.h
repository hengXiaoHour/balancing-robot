#ifndef BOARD_H
#define BOARD_H

// ===== BOARD SELECTION =====
// Change ONLY this line:
#define BOARD_PROFILE 2   // 1=ESP32, 2=ESP32-C3, 3=ESP32-S3

// Internal board IDs
#define BOARD_PROFILE_ESP32 1
#define BOARD_PROFILE_ESP32C3 2
#define BOARD_PROFILE_ESP32S3 3

// Compatibility board macros
#if BOARD_PROFILE == BOARD_PROFILE_ESP32
  #define BOARD_ESP32
#elif BOARD_PROFILE == BOARD_PROFILE_ESP32C3
  #define BOARD_ESP32C3
#elif BOARD_PROFILE == BOARD_PROFILE_ESP32S3
  #define BOARD_ESP32S3
#else
  #error "Invalid BOARD_PROFILE. Use 1, 2, or 3."
#endif

#if defined(BOARD_ESP32)
  #define PWM_FREQ 19000    //   ESP32 PWM frequency in Hz
  #define PWM_RES 12        // PWM resolution (12-bit = 0-4095)
  #define PWM_MAX 4095      // Maximum PWM value
  #define MOTOR_MIN_PWM 2000 // Minimum motor PWM duty (0-4095) - prevents motor from stalling

  // Status LED pin
  #define LOW_VOLTAGE_LED_PIN 22
  #define LED_ON_LEVEL LOW
  #define LED_OFF_LEVEL HIGH

  // Balancing robot motor driver pins (L298N-style)
  #define ENA 25
  #define IN1 26
  #define IN2 27
  #define ENB 14
  #define IN3 12
  #define IN4 13

  // I2C pins
  
  #define I2C_SDA 23
  #define I2C_SCL 19
  

  // I2C pins FOR BALANCING_ROBOT
  
  // #define I2C_SDA 21
  // #define I2C_SCL 22

  // ADC battery pin
  #define BATTERY_PIN 34
  #define BATTERY_R_UPPER_OHM 100000.0f
  #define BATTERY_R_LOWER_OHM 82000.0f
  #define BATTERY_CALIBRATION_SCALE 0.9957f
  #define BATTERY_DIVIDER_RATIO ((((BATTERY_R_UPPER_OHM + BATTERY_R_LOWER_OHM) / BATTERY_R_LOWER_OHM)) * BATTERY_CALIBRATION_SCALE)
  #define ADC_MAX 4095
  #define ADC_REF_VOLTAGE 3.3f
  #define BATTERY_SAMPLE_SIZE 8
  #define BATTERY_LPF_ALPHA 0.1f
  #define BATTERY_GLITCH_REJECT_V 0.25f

  // SPI pins (if SPI sensor selected)
  #define IMU_SPI_SCK_PIN 18
  #define IMU_SPI_MOSI_PIN 23
  #define IMU_SPI_MISO_PIN 19
  #define IMU_SPI_CS_PIN 5

#elif defined(BOARD_ESP32C3)
  #define PWM_FREQ 9000     // ESP32-C3 PWM frequency in Hz
  #define PWM_RES 12        // PWM resolution (12-bit = 0-4095)
  #define PWM_MAX 4095      // Maximum PWM value
  #define MOTOR_MIN_PWM 2000 // Minimum motor PWM duty (0-4095) - prevents motor from stalling


  // Status LED pin
  #define LOW_VOLTAGE_LED_PIN 8
  #define LED_ON_LEVEL HIGH
  #define LED_OFF_LEVEL LOW

  // Balancing robot motor driver pins (adjust to your wiring)
  #define ENA 3
  #define IN1 4
  #define IN2 5
  #define ENB 10
  #define IN3 6
  #define IN4 7

  // I2C pins
  #define I2C_SDA 0
  #define I2C_SCL 1

  // ADC battery pin
  #define BATTERY_PIN 9
  #define BATTERY_R_UPPER_OHM 100000.0f
  #define BATTERY_R_LOWER_OHM 82000.0f
  #define BATTERY_CALIBRATION_SCALE 0.9957f
  #define BATTERY_DIVIDER_RATIO ((((BATTERY_R_UPPER_OHM + BATTERY_R_LOWER_OHM) / BATTERY_R_LOWER_OHM)) * BATTERY_CALIBRATION_SCALE)
  #define ADC_MAX 4095
  #define ADC_REF_VOLTAGE 3.3f
  #define BATTERY_SAMPLE_SIZE 8
  #define BATTERY_LPF_ALPHA 0.1f
  #define BATTERY_GLITCH_REJECT_V 0.25f

  // SPI pins (if SPI sensor selected)
  #define IMU_SPI_SCK_PIN 2
  #define IMU_SPI_MOSI_PIN 3
  #define IMU_SPI_MISO_PIN 10
  #define IMU_SPI_CS_PIN 8

#elif defined(BOARD_ESP32S3)
  #define PWM_FREQ 9000     // ESP32-S3 PWM frequency in Hz
  #define PWM_RES 12        // PWM resolution (12-bit = 0-4095)
  #define PWM_MAX 4095      // Maximum PWM value
  #define MOTOR_MIN_PWM 2000 // Minimum motor PWM duty (0-4095) - prevents motor from stalling


  // Status LED pin
  #define LOW_VOLTAGE_LED_PIN 48
  #define LED_ON_LEVEL HIGH
  #define LED_OFF_LEVEL LOW

  // Balancing robot motor driver pins (adjust to your wiring)
  #define ENA 0
  #define IN1 2
  #define IN2 3
  #define ENB 4
  #define IN3 5
  #define IN4 6

  // I2C pins (available for barometer/other sensors)
  #define I2C_SDA 1
  #define I2C_SCL 2

  // ADC battery pin
  #define BATTERY_PIN 9
  #define BATTERY_R_UPPER_OHM 100000.0f
  #define BATTERY_R_LOWER_OHM 100000.0f
  #define BATTERY_CALIBRATION_SCALE 1.075f
  #define BATTERY_DIVIDER_RATIO ((((BATTERY_R_UPPER_OHM + BATTERY_R_LOWER_OHM) / BATTERY_R_LOWER_OHM)) * BATTERY_CALIBRATION_SCALE)
  #define ADC_MAX 4095
  #define ADC_REF_VOLTAGE 3.3f
  #define BATTERY_SAMPLE_SIZE 32
  #define BATTERY_LPF_ALPHA 0.01f
  #define BATTERY_GLITCH_REJECT_V 0.18f

  // SPI pins (your MPU6500 wiring)
  #define IMU_SPI_SCK_PIN 7
  #define IMU_SPI_MOSI_PIN 6
  #define IMU_SPI_MISO_PIN 5
  #define IMU_SPI_CS_PIN 4

#endif

#endif
