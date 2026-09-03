#ifndef FILTER_SELECTOR_H
#define FILTER_SELECTOR_H

/* ========================================
   FILTER SELECTOR - Choose ONE filter below
   Comment out the other 4 to disable them
   ======================================== */

// Uncomment ONE of these:
// #define USE_KALMAN_FILTER
// #define USE_MAHONY_FILTER
#define USE_MADGWICK_FILTER
// #define USE_COMPLEMENTARY_FILTER
// #define USE_COMPLEMENTARY_QUATERNION_FILTER
// #define USE_EKF_FILTER

// ===== COMMON SENSOR PRE-FILTERING =====
// Accelerometer Low-Pass Filter
#ifndef ACCEL_LPF_ALPHA
#define ACCEL_LPF_ALPHA 0.7f  // LPF coefficient (0.0-1.0)
#endif

// Gyroscope Low-Pass Filter (optional, currently not used)
#ifndef GYRO_LPF_ALPHA
#define GYRO_LPF_ALPHA 0.7f    // LPF coefficient for gyro data
#endif

// ===== FILTER STATUS STRING =====
const char* getActiveFilterName() {
  #ifdef USE_KALMAN_FILTER
    return "Kalman Filter";
  #elif defined(USE_MAHONY_FILTER)
    return "Mahony Filter";
  #elif defined(USE_MADGWICK_FILTER)
    return "Madgwick Filter";
  #elif defined(USE_COMPLEMENTARY_FILTER)
    return "Complementary Filter (2D)";
  #elif defined(USE_COMPLEMENTARY_QUATERNION_FILTER)
    return "Complementary Filter (3D/Quaternion)";
  #elif defined(USE_EKF_FILTER)
    return "Extended Kalman Filter (EKF)";
  #else
    return "NO FILTER SELECTED!";
  #endif
}

#endif
