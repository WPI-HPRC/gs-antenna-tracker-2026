#include <Arduino.h>
#include "tracker.h"

Tracker tracker;

void setup() {
  Serial.begin(115200);
  
  while (!Serial) {} // Wait for serial to be ready

  tracker.initializeTracker();
}

void loop() {
  tracker.trackerLoop();
}