
#include "chassis.h"
#include "stepperDriver.h"
#include <AccelStepper.h>

IntervalTimer chassisTimer;
stepperDriver azimuthStepper;
stepperDriver elevationStepper;


Chassis::Chassis(void) {
    // Constructor code here
}

void Chassis::initializeChassis(void) {
    chassisTimer.begin(chassisLoop, 10000); // Call chassisLoop every 10ms
}

void Chassis::chassisLoop(void) {
    Chassis::updateChassis();
}

void Chassis::updateChassis(void) {
    // Code to update the chassis state
}

void Chassis::setChassisSpeed(float azSpeed, float elSpeed) {
    // Code to set the speed of the chassis based on input speeds
}

void Chassis::stopChassis(void) {
    Chassis::setChassisSpeed(0.0, 0.0);
}

void Chassis::chassisESTOP(void) {
    // Code to immediately stop the chassis in case of an emergency
    Chassis::stopChassis();

    // TODO: Add additional code to handle alarm signal and disabling motor power
}