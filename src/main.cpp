/**
 * The tracker stuff isn't implemented it, but this should be mostly what main includes.
 * The purpose of main is legit just to setup serial communication and run tracker in a loop.
 */


#include <Arduino.h>
#include "tracker.h"

Tracker tracker;

void setup() {
  Serial.begin(115200);

  tracker.InitializeTracker();
}

void loop() {
  tracker.trackerLoop();
}