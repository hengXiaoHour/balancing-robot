#ifndef TIMING_H
#define TIMING_H

// ===== GLOBAL LOOP TIMING VARIABLES =====
volatile unsigned long loopTimeUs = 0;
volatile unsigned long loopTimeMinUs = 1000000;
volatile unsigned long loopTimeMaxUs = 0;
volatile unsigned long loopOverrunCount = 0;

#endif
