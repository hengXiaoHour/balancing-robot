#include "control_task.h"

// ===== CORE 0: Control Loop (IMU + Kalman + PID) =====
void controlLoopTask(void *pvParameters) {
  unsigned long lastLoopTime = millis();
  unsigned long warmupStartTime = millis();
  const unsigned long WARMUP_TIME = 2000;  // 2 seconds for sensors to stabilize

  while (1) {
    // Maintain 1kHz control loop
    if (millis() - lastLoopTime >= LOOP_INTERVAL) {
      unsigned long loopStart = millis();
      dt = (loopStart - lastLoopTime) / 1000.0;
      lastLoopTime = loopStart;

      // Fast ESP-NOW path (1kHz) for joystick input
      #if ENABLE_ESPNOW
      updateESPNOWDataOnly();
      #endif

      // Barometer disabled - no updateBarometer() call

      // Read sensor data (only if MPU is initialized)
      if (mpuInitialized) {
        mpu.readAll();

        // Store raw sensor values
        mpu_accelX = mpu.accelX;
        mpu_accelY = mpu.accelY;
        mpu_accelZ = mpu.accelZ;
        mpu_gyroX = mpu.gyroX;
        mpu_gyroY = mpu.gyroY;
        mpu_gyroZ = mpu.gyroZ;
      } else {
        // Set default values if MPU not available
        mpu_accelX = mpu_accelY = mpu_accelZ = 0;
        mpu_gyroX = mpu_gyroY = mpu_gyroZ = 0;
      }

      // Apply low-pass filter to accelerometer data
      filtered_accelX = ACCEL_LPF_ALPHA * mpu_accelX + (1.0 - ACCEL_LPF_ALPHA) * filtered_accelX;
      filtered_accelY = ACCEL_LPF_ALPHA * mpu_accelY + (1.0 - ACCEL_LPF_ALPHA) * filtered_accelY;
      filtered_accelZ = ACCEL_LPF_ALPHA * mpu_accelZ + (1.0 - ACCEL_LPF_ALPHA) * filtered_accelZ;

      // Apply low-pass filter to gyroscope data
      filtered_gyroX = GYRO_LPF_ALPHA * mpu_gyroX + (1.0 - GYRO_LPF_ALPHA) * filtered_gyroX;
      filtered_gyroY = GYRO_LPF_ALPHA * mpu_gyroY + (1.0 - GYRO_LPF_ALPHA) * filtered_gyroY;
      filtered_gyroZ = GYRO_LPF_ALPHA * mpu_gyroZ + (1.0 - GYRO_LPF_ALPHA) * filtered_gyroZ;

      // Apply calibration bias to raw data
      accelX = (mpu_accelX - axBias) * axScale;
      accelY = (mpu_accelY - ayBias) * ayScale;
      accelZ = (mpu_accelZ - azBias) * azScale;
      // Keep gyro in filtered raw LSB here; each filter converts to deg/s and applies stored deg/s bias.
      gyroX = filtered_gyroX;
      gyroY = filtered_gyroY;
      gyroZ = filtered_gyroZ;

      // Initialize filter after warmup period with stable accel data (only if MPU initialized)
      if (!filterInitialized && mpuInitialized && (loopStart - warmupStartTime) >= WARMUP_TIME) {
        #ifdef USE_KALMAN_FILTER
          initKalmanFilter();
        #elif defined(USE_MAHONY_FILTER)
          initMahonyFilter();
        #elif defined(USE_MADGWICK_FILTER)
          initMadgwickFilter();
        #elif defined(USE_COMPLEMENTARY_QUATERNION_FILTER)
          initComplementaryQuaternionFilter();
        #elif defined(USE_EKF_FILTER)
          initEKFFilter();
        #endif
        filterInitialized = true;
        if (debugMonitoring) {
          Serial.println("[OK] Filter initialized with stable accel data");
        }
      }

      // ===== UPDATE FILTER (Choose ONE - comment out other 4) =====
      // Measure filter execution time
      unsigned long filterStart = micros();

      #ifdef USE_KALMAN_FILTER
        updateKalmanFilter();
      #elif defined(USE_MAHONY_FILTER)
        if (filterInitialized) updateMahonyFilter();
      #elif defined(USE_MADGWICK_FILTER)
        if (filterInitialized) updateMadgwickFilter();
      #elif defined(USE_COMPLEMENTARY_FILTER)
        updateComplementaryFilter();
      #elif defined(USE_COMPLEMENTARY_QUATERNION_FILTER)
        if (filterInitialized) updateComplementaryQuaternionFilter();
      #elif defined(USE_EKF_FILTER)
        if (filterInitialized) updateEKFFilter();
      #else
        #error "NO FILTER SELECTED! Enable one filter in filter_selector.h"
      #endif

      filterExecutionTime = micros() - filterStart;
      avgFilterTime = TIME_ALPHA * filterExecutionTime + (1.0 - TIME_ALPHA) * avgFilterTime;

      // Apply trim on final angle output so it follows axis swap + inversion logic
      pitch_final = PITCH_ANGLE_RAW + trim_pitch;
      roll_final = ROLL_ANGLE_RAW + trim_roll;

      // Update ToF sensor (non-blocking state machine)
      #if TOF_ENABLED
      tof_update();
      #endif

      // Altitude fusion disabled

      freeHeapMemory = ESP.getFreeHeap();
      if (freeHeapMemory < minFreeHeap) {
        minFreeHeap = freeHeapMemory;
      }

      // Apply low-pass filter to FINAL trimmed angles for failsafe (smooth out spikes)
      filtered_pitch = FAILSAFE_LPF_ALPHA * PITCH_ANGLE_FINAL_USED + (1.0 - FAILSAFE_LPF_ALPHA) * filtered_pitch;
      filtered_roll = FAILSAFE_LPF_ALPHA * ROLL_ANGLE_FINAL_USED + (1.0 - FAILSAFE_LPF_ALPHA) * filtered_roll;

      // ===== UPDATE ARM HYSTERESIS =====
      // Check if drone is level enough to allow arming (only if MPU is initialized)
      if (mpuInitialized && abs(filtered_pitch) < SAFE_ANGLE_THRESHOLD && abs(filtered_roll) < SAFE_ANGLE_THRESHOLD) {
        safeAngleCounter++;
        if (safeAngleCounter >= SAFE_ANGLE_HYSTERESIS_CHECKS) {
          safeToArm = true;  // Drone has been level for required duration
          safeAngleCounter = SAFE_ANGLE_HYSTERESIS_CHECKS;  // Cap the counter
        }
      } else {
        // Angle exceeded safe threshold - reset counter and disallow arming
        safeAngleCounter = 0;
        safeToArm = false;
      }

      if (!motorsArmed) {
        motorsActive = false;
      }

      if (testMotorActive) {
        // Motor test runs on Core 1; skip control loop motor output.
      }
      else if (motorsArmed && motorsActive) {
        // Failsafe: Disarm if MPU6050 becomes disconnected
        if (!mpuInitialized) {
          motorsArmed = false;
          motorsActive = false;
          safeToArm = false;
          safeAngleCounter = 0;
          statusMonitoring = false;
          debugMonitoring = false;
          stopMotors();

          // Reset all control variables
          throttle = 0.0f;
          pidIntegral_Pitch = 0.0f;
          pidIntegral_Roll = 0.0f;
          pidIntegral_Yaw = 0.0f;
          #if ENABLE_CASCADE_PID
          resetCascadePID();
          #endif

          Serial.println("\n[FAILSAFE] MPU6050 disconnected during flight - Motors DISARMED!");

          // Force immediate broadcast to WebSocket UI
          broadcastTelemetryImmediate();
        }
        // Failsafe: Disarm if filtered pitch or roll exceeds ±45° angle limit
        else if (abs(filtered_pitch) > 45.0f || abs(filtered_roll) > 45.0f) {
          motorsArmed = false;
          motorsActive = false;
          safeToArm = false;  // Also reset arm hysteresis on failsafe
          safeAngleCounter = 0;
          statusMonitoring = false;  // Stop status printing on failsafe
          debugMonitoring = false;   // Stop debug printing on failsafe
          stopMotors();

          // Reset all control variables to prevent motor twitching/spin-up
          throttle = 0.0f;
          pidIntegral_Pitch = 0.0f;
          pidIntegral_Roll = 0.0f;
          pidIntegral_Yaw = 0.0f;
          #if ENABLE_CASCADE_PID
          resetCascadePID();
          #endif

          Serial.print("\n[FAILSAFE] Pitch: "); Serial.print(filtered_pitch, 2);
          Serial.print("° | Roll: "); Serial.print(filtered_roll, 2);
          Serial.println("° exceeded ±45° - Motors DISARMED! (All control vars reset)");

          // FORCE immediate broadcast to WebSocket UI so button syncs immediately
          // Don't wait for rate-limited broadcastTelemetry()
          broadcastTelemetryImmediate();
        } else {
          // ===== SIMPLE BRAKING SYSTEM =====
          // Apply braking when joystick is released for faster stopping
          applyBraking();

          // ===== VEHICLE INPUT SAFETY LIMITS =====
          // Balancing mode clamps joystick-driven tilt setpoints.
          applyVehicleInputLimits();

          // ===== SELECT PID CONTROLLER =====
          #if ENABLE_CASCADE_PID
          // Cascade PID: outer angle loop + inner rate loop
          updateCascadePID();
          #else
          // Standard dual-axis PID (pitch + roll + yaw control)
          updateDualPID();
          #endif

          updateVehicleMotorControl();
        }
      } else {
        stopMotors();
      }

      loopTimeMs = millis() - loopStart;
      loopTimeUs = loopTimeMs * 1000UL;
    }

    // Prevent watchdog timeout
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
