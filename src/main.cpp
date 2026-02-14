#include <Arduino.h>
#include "tracker.h"

Tracker tracker;

void setup() {
  Serial.begin(115200);

  tracker.InitializeTracker();
}

void loop() {
  tracker.TrackerLoop();

  float targetAz = 45.0; // more random angles
  float targetEl = 10.0;
  runTrackerControl(targetAz, targetEl);

  stepperAz.runSpeed(); // tell motors to go 
  stepperEl.runSpeed();
}