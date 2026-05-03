#include "chassis.h"
#include <AccelStepper.h>
#include <math.h>

// Initialize the stepper(s)
// Interface type 1 means an external driver with Step and Direction pins
AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper elevation(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

/// @brief Initializes chassis motors and drivers
void Chassis::initializeChassis() {
    // Setup enable pins and set to low (Enabled)
    pinMode(AZ_EN_PIN, OUTPUT);
    pinMode(EL_EN_PIN, OUTPUT);
    digitalWrite(AZ_EN_PIN, LOW);
    digitalWrite(EL_EN_PIN, LOW);

    // Configure Azimuth
    azimuth.setMaxSpeed(STEPS_PER_REV * MAX_RPM);
    azimuth.setAcceleration(ACCELERATION * AZ_GEAR_REDUCTION);
    azimuth.setMinPulseWidth(PULSE_WIDTH);

    // Configure Elevation
    elevation.setMaxSpeed(STEPS_PER_REV * MAX_RPM);
    elevation.setAcceleration(ACCELERATION * EL_GEAR_REDUCTION);
    elevation.setMinPulseWidth(PULSE_WIDTH);
}

/// @brief Updates and runs chassis functions
/// @return Returns true if loop was successful
bool Chassis::chassisLoop() {
    azimuth.runSpeed();
    elevation.runSpeed();

    return true;
}

/// @brief Sets motor run speeds for next chassis update
/// @param azSpeed Azimuth rotation speed [rad/s]
/// @param elSpeed Elevation rotation speed [rad/s]
void Chassis::setSpeed(float azSpeed, float elSpeed) {
    azimuth.setSpeed(azRadToStep(azSpeed));
    elevation.setSpeed(elRadToStep(elSpeed));
}

/// @brief Stops motors
/// @param az Stops azimuth motor if true
/// @param el Stops elevation motor if true
void Chassis::stop(bool az, bool el) {
    if (az) { azimuth.setSpeed(0); azimuth.runSpeed(); };
    if (el) { elevation.setSpeed(0); elevation.runSpeed(); };
}

/// @brief Enables or frees motors
/// @param az Enables azimuth if boolean is true, free otherwise
/// @param el Enables elevation if boolean is true, free otherwise
void Chassis::enable(bool az, bool el) {
    digitalWrite(AZ_EN_PIN, az ? LOW : HIGH);
    digitalWrite(EL_EN_PIN, el ? LOW : HIGH);
}

/// @brief Sets max speed of motors
/// @param azSpeed Azimuth axis max speed [rad/s]
/// @param elSpeed Elevation axis max speed [rad/s]
void Chassis::setMaxSpeed(float azSpeed, float elSpeed) {
    azimuth.setMaxSpeed(azRadToStep(azSpeed));
    elevation.setMaxSpeed(elRadToStep(elSpeed));
}

/// @brief Sets motor acceleration
/// @param azAccel Azimuth axis max acceleration [rad/s^2]
/// @param elAccel Elevation axis max acceleration [rad/s^2]
void Chassis::setAccel(float azAccel, float elAccel) {
    azimuth.setAcceleration(azRadToStep(azAccel));
    elevation.setAcceleration(elRadToStep(elAccel));
}