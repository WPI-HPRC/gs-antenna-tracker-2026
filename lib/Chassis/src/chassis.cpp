#include "chassis.h"

Chassis::Chassis() {}

/// @brief Updates and runs chassis functions
void Chassis::chassisLoop() {
    if (azEnabled)azMotor->runSpeed();
    if (elEnabled)elMotor->runSpeed();

    // Checkers and Handlers
    if (checkFrontHall()) handleFrontHall();
    if (checkRearHall()) handleRearHall();
    // if (checkAzAlarm()) handleAzAlarm();
    // if (checkElAlarm()) handleElAlarm();
}

/// @brief Initializes chassis motors and drivers
void Chassis::initializeChassis() {
    // Define motors, encoders, and hall effect sensors
    azMotor = new AccelStepper(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
    elMotor = new AccelStepper(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);
    frontHall = new HallEffect(FRONT_HALL_PIN);
    rearHall = new HallEffect(REAR_HALL_PIN);

    // Setup enable pins and set to high (Disabled)
    pinMode(AZ_EN_PIN, OUTPUT);
    pinMode(EL_EN_PIN, OUTPUT);
    digitalWrite(AZ_EN_PIN, HIGH);
    digitalWrite(EL_EN_PIN, HIGH);

    // Setup alarm pins for read
    pinMode(AZ_ALRM_PIN, INPUT);
    pinMode(EL_ALRM_PIN, INPUT);

    // Initialize Hall Effect Sensors
    frontHall->initializeHallEffect();
    rearHall->initializeHallEffect();

    // Configure Azimuth Motor
    azMotor->setMaxSpeed(STEPS_PER_REV * MAX_RPM);
    azMotor->setAcceleration(ACCELERATION * AZ_GEAR_REDUCTION);
    azMotor->setMinPulseWidth(PULSE_WIDTH);

    // Configure Elevation
    elMotor->setMaxSpeed(STEPS_PER_REV * MAX_RPM);
    elMotor->setAcceleration(ACCELERATION * EL_GEAR_REDUCTION);
    elMotor->setMinPulseWidth(PULSE_WIDTH);
}

/// @brief Sets motor run speeds for next chassis update
/// @param azSpeed Azimuth rotation speed [rad/s]
/// @param elSpeed Elevation rotation speed [rad/s]
void Chassis::setSpeed(float azSpeed, float elSpeed) {
    azMotor->setSpeed(azRadToStep(azSpeed));
    elMotor->setSpeed(elRadToStep(elSpeed));
}

/// @brief Stops motors
/// @param az Stops azimuth motor if true
/// @param el Stops elevation motor if true
void Chassis::stop(bool az, bool el) {
    if (az) azMotor->setSpeed(0);
    if (el) elMotor->setSpeed(0);
}

/// @brief Enables or frees motors
/// @param az Enables azimuth motor if boolean is true, free otherwise
/// @param el Enables elevation motor if boolean is true, free otherwise
void Chassis::enable(bool az, bool el) {
    digitalWrite(AZ_EN_PIN, az ? LOW : HIGH);
    digitalWrite(EL_EN_PIN, el ? LOW : HIGH);

    azEnabled = az;
    elEnabled = el;

    prevAzEnState = az;
    prevElEnState = el;
}

/// @brief Sets max speed of motors
/// @param azSpeed Azimuth axis max speed [rad/s]
/// @param elSpeed Elevation axis max speed [rad/s]
void Chassis::setMaxSpeed(float azSpeed, float elSpeed) {
    azMotor->setMaxSpeed(azRadToStep(azSpeed));
    elMotor->setMaxSpeed(elRadToStep(elSpeed));
}

/// @brief Sets motor acceleration
/// @param azAccel Azimuth axis max acceleration [rad/s^2]
/// @param elAccel Elevation axis max acceleration [rad/s^2]
void Chassis::setAccel(float azAccel, float elAccel) {
    azMotor->setAcceleration(azRadToStep(azAccel));
    elMotor->setAcceleration(elRadToStep(elAccel));
}

float Chassis::getAzPose() {
    return azStepToRad(azMotor->currentPosition());
}

float Chassis::getElPose() {
    return elStepToRad(elMotor->currentPosition());
}

/// @brief Spins in a direction until either hall effect sensor is triggered
/// @param positiveDir Spin in the positive direction if true
void Chassis::calibrateEl(bool positiveDir) {
    elCalibrating = true;

    if (positiveDir) {
        elMotor->setSpeed(elRadToStep(0.25));
    } else {
        elMotor->setSpeed(elRadToStep(-0.25));
    }
}

void Chassis::calibrateAz(bool accurate) {
    azCalibrating = true;

    if (!accurate) {
        azMotor->setSpeed(azRadToStep(2*PI/15)); // 1 revolution every 15 seconds
        azMotor->runSpeed();
    } else {
        azMotor->setSpeed(0);
        azMotor->runSpeed();
    }
}

/// @brief Checks if the front hall effect sensor was triggered
/// @return Returns true once if detected
bool Chassis::checkFrontHall() {
    if (frontHall->readSensor() && !frontPrevDetected) {
        frontPrevDetected = true;
        return true;
    }

    if (!frontHall->readSensor()) {
        frontPrevDetected = false;
    }

    return false;
}

/// @brief Updates encoder count to match front hall
void Chassis::handleFrontHall() {
    float speed = elMotor->speed();
    if (speed > 0) {
        elMotor->setCurrentPosition(elRadToStep(0.7));
    } else {
        elMotor->setCurrentPosition(elRadToStep(0.89));
    }

    if (elCalibrating) {
        // Ensures elevation stops when calibration is complete
        Serial.println("Elevation Calibrated!");
        elCalibrating = false;

        elMotor->setSpeed(0);
        elMotor->runSpeed();
    } else {
        elMotor->setSpeed(speed);
        elMotor->runSpeed();
    }

}

/// @brief Checks if the front hall effect sensor was triggered
/// @return Returns true once if detected
bool Chassis::checkRearHall() {
    if (rearHall->readSensor() && !rearPrevDetected) {
        rearPrevDetected = true;
        return true;
    }

    if (!rearHall->readSensor()) {
        rearPrevDetected = false;
    }

    return false;
}

/// @brief Updates encoder count to match rear hall
void Chassis::handleRearHall() {
    float speed = elMotor->speed();
    if (speed > 0) {
        elMotor->setCurrentPosition(elRadToStep(2.26));
    } else {
        elMotor->setCurrentPosition(elRadToStep(2.51));
    }

    if (elCalibrating) {
        // Ensures elevation stops when calibration is complete
        Serial.println("Elevation Calibrated!");
        elCalibrating = false;

        elMotor->setSpeed(0);
        elMotor->runSpeed();
    } else {
        elMotor->setSpeed(speed);
        elMotor->runSpeed();
    }
}

/// @brief Checks if the azimuth alarm is on
/// @return Returns true whenever there is a change in azimuth alarm state
bool Chassis::checkAzAlarm() {
    if (digitalRead(AZ_ALRM_PIN) && !prevAzAlarm) {
        prevAzAlarm = true;
        prevAzEnState = azEnabled;
        return true;
    }

    if (!digitalRead(AZ_ALRM_PIN) && prevAzAlarm) {
        prevAzAlarm = false;
        return true;
    }

    return false;
}

/// @brief Disables azimuth motor to handle alarm
void Chassis::handleAzAlarm() {
    if (prevAzAlarm) {
        digitalWrite(AZ_EN_PIN, HIGH);
        azEnabled = false;
    } else {
        digitalWrite(AZ_EN_PIN, prevAzEnState ? LOW : HIGH);
        azEnabled = prevAzEnState;
    }
}

/// @brief Checks if the elevation alarm is on
/// @return Returns true whenever there is a change in elevation alarm state
bool Chassis::checkElAlarm() {
    if (digitalRead(EL_ALRM_PIN) && !prevElAlarm) {
        prevElAlarm = true;
        prevElEnState = elEnabled;
        return true;
    }

    if (!digitalRead(EL_ALRM_PIN) && prevElAlarm) {
        prevElAlarm = false;
        return true;
    }

    return false;
}

/// @brief Disables elevation motor to handle alarm
void Chassis::handleElAlarm() {
    if (prevElAlarm) {
        digitalWrite(EL_EN_PIN, HIGH);
        elEnabled = false;
    } else {
        digitalWrite(EL_EN_PIN, prevElEnState ? LOW : HIGH);
        elEnabled = prevElEnState;
    }
}