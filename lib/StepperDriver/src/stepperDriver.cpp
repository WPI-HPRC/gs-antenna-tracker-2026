/**
 * Will take higher level control from chassis.cpp to direclty drive
 * the stepper motors. It'll mostly convert input variables, such as
 * RPM or rad/s into a pulse frequency for use by the stepper drivers.
 */

#include "stepperDriver.h"

StepperDriver::StepperDriver(int stepPin, int dirPin)
    : stepPin(stepPin),
      dirPin(dirPin),
      stepper(AccelStepper::DRIVER, stepPin, dirPin)
{
}

void StepperDriver::InitializeStepperDriver()
{
    stepper.setMaxSpeed(1000);
}

void StepperDriver::SetSpeed(float speed)
{
    stepper.setSpeed(speed * stepsPerDegree);
}

void StepperDriver::SetPID(float Kp_, float Ki_, float Kd_)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}