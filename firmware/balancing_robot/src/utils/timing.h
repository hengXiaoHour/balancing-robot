#ifndef TIMING_H
#define TIMING_H

// ===== GLOBAL LOOP TIMING VARIABLES (declarations) =====
// Definitions live in timing.cpp. Included once via balancing_robot.ino.
extern volatile unsigned long loopTimeUs;
extern volatile unsigned long loopTimeMinUs;
extern volatile unsigned long loopTimeMaxUs;
extern volatile unsigned long loopOverrunCount;

#endif
