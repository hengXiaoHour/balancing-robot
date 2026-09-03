#include "timing.h"

// ===== GLOBAL LOOP TIMING VARIABLES (definitions) =====
volatile unsigned long loopTimeUs = 0;
volatile unsigned long loopTimeMinUs = 1000000;
volatile unsigned long loopTimeMaxUs = 0;
volatile unsigned long loopOverrunCount = 0;
