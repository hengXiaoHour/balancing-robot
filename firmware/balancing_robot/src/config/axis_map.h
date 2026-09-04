#ifndef AXIS_MAP_H
#define AXIS_MAP_H

// ===== AXIS MAPPING MACHINERY =====
// No user-editable behaviour here — only the INVERT flags above are tuned,
// and those live in settings.h. Everything below derives from them.

// ----- Axis inversion flags (set in settings.h) -----
#if PITCH_AXIS_INVERT
  #define PITCH_AXIS_SIGN -1.0f
#else
  #define PITCH_AXIS_SIGN 1.0f
#endif

#if ROLL_AXIS_INVERT
  #define ROLL_AXIS_SIGN -1.0f
#else
  #define ROLL_AXIS_SIGN 1.0f
#endif

#if YAW_AXIS_INVERT
  #define YAW_AXIS_SIGN -1.0f
#else
  #define YAW_AXIS_SIGN 1.0f
#endif

#if GYRO_X_INVERT
  #define GYRO_X_SIGN -1.0f
#else
  #define GYRO_X_SIGN 1.0f
#endif

#if GYRO_Y_INVERT
  #define GYRO_Y_SIGN -1.0f
#else
  #define GYRO_Y_SIGN 1.0f
#endif

#if GYRO_Z_INVERT
  #define GYRO_Z_SIGN -1.0f
#else
  #define GYRO_Z_SIGN 1.0f
#endif

// ----- Derived gyro readings -----
#define GYRO_X_USED (GYRO_X_SIGN * gyroX)
#define GYRO_Y_USED (GYRO_Y_SIGN * gyroY)
#define GYRO_Z_USED (GYRO_Z_SIGN * gyroZ)
#define GYRO_BIAS_X_USED (GYRO_X_SIGN * gyroBiasX)
#define GYRO_BIAS_Y_USED (GYRO_Y_SIGN * gyroBiasY)
#define GYRO_BIAS_Z_USED (GYRO_Z_SIGN * gyroBiasZ)

// ----- Balancing robot axis mapping (pitch-only control, no roll) -----
#define PITCH_ANGLE_RAW pitch
#define PITCH_ANGLE_FINAL pitch_final
#define ROLL_ANGLE_RAW roll
#define ROLL_ANGLE_FINAL roll_final

#define GYRO_PITCH_RATE_RAW GYRO_Y_USED
#define GYRO_PITCH_BIAS_RAW GYRO_BIAS_Y_USED

#define PITCH_ANGLE_RAW_USED (PITCH_AXIS_SIGN * PITCH_ANGLE_RAW)
#define PITCH_ANGLE_FINAL_USED (PITCH_AXIS_SIGN * PITCH_ANGLE_FINAL)
#define GYRO_PITCH_RATE_USED (PITCH_AXIS_SIGN * GYRO_PITCH_RATE_RAW)
#define GYRO_PITCH_BIAS_USED (PITCH_AXIS_SIGN * GYRO_PITCH_BIAS_RAW)
#define GYRO_YAW_RATE_USED GYRO_Z_USED
#define GYRO_YAW_BIAS_USED GYRO_BIAS_Z_USED

// ----- Roll axis stubs (balancing robot has no roll control) -----
#define GYRO_ROLL_RATE_USED 0.0f
#define GYRO_ROLL_BIAS_USED 0.0f
#define ROLL_ANGLE_RAW_USED 0.0f
#define ROLL_ANGLE_FINAL_USED 0.0f

// ----- Gyro sensitivity (LSB per deg/s) -----
#define GYRO_SENSITIVITY 16.4  // ±2000°/s range (±500: 65.5, ±250: 131.0)

// ----- Accelerometer sensitivity (LSB per g, derives from ACCEL_RANGE_G) -----
#if ACCEL_RANGE_G == 2
  #define ACCEL_SENSITIVITY 16384.0f
#elif ACCEL_RANGE_G == 4
  #define ACCEL_SENSITIVITY 8192.0f
#elif ACCEL_RANGE_G == 8
  #define ACCEL_SENSITIVITY 4096.0f
#elif ACCEL_RANGE_G == 16
  #define ACCEL_SENSITIVITY 2048.0f
#else
  #define ACCEL_SENSITIVITY 16384.0f  // Default to ±2G
#endif

#endif  // AXIS_MAP_H
