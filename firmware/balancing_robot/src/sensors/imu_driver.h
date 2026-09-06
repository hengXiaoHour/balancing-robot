#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>

// ===== Runtime IMU driver interface (auto-detect) =====
// All three custom drivers inherit this so initIMU() can probe each bus
// (WHO_AM_I, not just ACK — 0x68 is shared across MPU6050/6500/9250/DS1307)
// and hand the winner to the control loop via a single pointer.
class IMU_Driver {
public:
  // Shared sample fields (drivers write into these; control_task reads them)
  int16_t accelX = 0, accelY = 0, accelZ = 0;
  int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
  int16_t temp = 0;

  // Shared calibration offsets
  int16_t accelXOffset = 0;
  int16_t accelYOffset = 0;
  int16_t accelZOffset = 0;
  int16_t gyroXOffset = 0;
  int16_t gyroYOffset = 0;
  int16_t gyroZOffset = 0;

  virtual ~IMU_Driver() {}
  virtual bool initialize() = 0;
  virtual void readAccel() = 0;
  virtual void readGyro() = 0;
  virtual void readTemp() = 0;
  virtual void readAll() = 0;
  virtual const char* driverName() const = 0;
};

#endif
