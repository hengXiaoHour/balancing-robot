#ifndef I2C_SCAN_H
#define I2C_SCAN_H

#include <stdint.h>

// Device name lookup helper (see i2c_scan.cpp)
const char* getI2CDeviceName(uint8_t address);

// Scan I2C bus and report all active addresses (see i2c_scan.cpp)
void scanI2CBus();

#endif
