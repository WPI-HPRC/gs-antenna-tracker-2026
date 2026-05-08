#include <Arduino.h>
#include "tracker.h"

// Manual fix for TeensyStep generic timer compatibility on Teensy 4/MicroMod
#ifndef dwt_getCycles
#define dwt_getCycles() (ARM_DWT_CYCCNT)
#endif

Tracker tracker;

void setup() {
  Serial.begin(115200);
  
  while (!Serial) {} // Wait for serial to be ready

  tracker.initializeTracker();
}

void loop() {
  tracker.trackerLoop();
}