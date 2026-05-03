#include "tracker.h"
#include <Arduino.h>

Tracker tracker;

void setup() {
  Serial.begin(115200);
  tracker.initializeTracker();
}

void loop() {
  tracker.trackerLoop();
}