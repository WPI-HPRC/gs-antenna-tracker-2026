/**
 * The tracker stuff isn't implemented it, but this should be mostly what main includes.
 * The purpose of main is legit just to setup serial communication and run tracker in a loop.
 */


#include <Arduino.h>
#include <AccelStepper.h>
#include "tracker.h"
#include "tracker-control.h"

const int AZ_STEP_PIN = 1;
const int AZ_DIR_PIN  = 2;

const int EL_STEP_PIN = 3;
const int EL_DIR_PIN  = 4;

// arduino accel stepper library which steps the motors for us and is very convenient
// docs: https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
AccelStepper stepperAz(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper stepperEl(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  stepperAz.setMaxSpeed(6700); // more temporary numbers
  stepperEl.setMaxSpeed(6700);
  initTracker();
}

void loop() {
  float targetAz = 45.0; // more random angles
  float targetEl = 10.0;
  runTrackerControl(targetAz, targetEl);

  stepperAz.runSpeed(); // tell motors to go 
  stepperEl.runSpeed();
}