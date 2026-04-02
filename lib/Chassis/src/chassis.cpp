
#include "chassis.h"
#include <AccelStepper.h>

IntervalTimer chassisTimer;
stepperDriver azimuthStepper;
stepperDriver elevationStepper;

AccelStepper azStepper(AccelStepper::DRIVER, this->AZ_PULSE_PIN, this->AZ_DIR_PIN);
AccelStepper elStepper(AccelStepper::DRIVER, this->EL_PULSE_PIN, this->EL_DIR_PIN);


Chassis::Chassis(void) {
    // Constructor code here
}

void Chassis::initializeChassis(void) {
    chassisTimer.begin(chassisLoop, 10000); // Call chassisLoop every 10ms

    azStepper.setEnablePin(this->AZ_ENABLE_PIN);
    elStepper.setEnablePin(this->EL_ENABLE_PIN);

    azStepper.setMaxSpeed(this->azimuthMaxSpeed);
    elStepper.setMaxSpeed(this->elevationMaxSpeed);

    azStepper.setAcceleration(this->azimuthAcceleration);
    elStepper.setAcceleration(this->elevationAcceleration);

    azStepper.setMinPulseWidth(5); // microseconds
    elStepper.setMinPulseWidth(5); // microseconds

    // TODO: Update this
    azStepper.setPinsInverted(false, false, true);
    elStepper.setPinsInverted(false, false, true);
}

void Chassis::chassisLoop(void) {
    Chassis::updateChassis();

    // Add additional code per update cycle, such as reading encoder values and handling alarms
    // Encoder::readEncoders();
}

void Chassis::updateChassis(void) {
    azStepper.runSpeed();
    elStepper.runSpeed();
}

void Chassis::setChassisSpeed(float azSpeed, float elSpeed) {
    azStepper.setSpeed(azSpeed);
    elStepper.setSpeed(elSpeed);
}

void Chassis::stopChassis(void) {
    azStepper.stop();
    elStepper.stop();
}

void Chassis::freeChassis(void) {
    azStepper.disableOutputs();
    elStepper.disableOutputs();
}

void Chassis::calibrateChassis(void) {
    // TODO: Update this
    elStepper.setCurrentPosition(0);
}