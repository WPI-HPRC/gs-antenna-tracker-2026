
#include <AccelStepper.h>
#include "stepperDriver.h"

void StepperDriver::StepperDriver(int stepPin, int dirPin, int enablePin) {
    this->STEP_PIN = stepPin;
    this->DIR_PIN = dirPin;
    this->ENABLE_PIN = enablePin;
}

void StepperDriver::initializeStepperDriver(void) {
    this->stepper = AccelStepper(AccelStepper::DRIVER, thisSTEP_PIN, DIR_PIN);
}

void StepperDriver::setSpeed(float speed) {
    this->speed = speed;
}

void StepperDriver::setMaxSpeed(float maxSpeed) {
    this->maxSpeed = maxSpeed;
}

void StepperDriver::stop(void) {
    this->speed = 0.0;
}

void StepperDriver::freeMotor(void) {
    // Code to disable motor power and allow free movement
}