#include <Arduino.h>
#include "tracker.h"

Tracker tracker;

void setup() {
  Serial.begin(115200);

  tracker.InitializeTracker();
}

void loop() {
  tracker.TrackerLoop();
}