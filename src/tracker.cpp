#include "tracker.h"
#include "serial_comm.h"

void Tracker::trackerLoop() {
    
    if (checkSerialInput()) parseSerialInput();

    if (trackerState == TRACKER_CALIBRATING) {
        if (!elCalibrated) {
            chassis->calibrateEl(false);
            elCalibrated = true;
        }

        chassis->calibrateAz(false);
    }

    if (trackerState == TRACKER_TRACKING) {    
        targetAz = azInput;
        targetEl = elInput;

        // Azimuth Control
        float targetAzPose = targetAz;
        float azError = targetAzPose - chassis->getAzPose();

        // Elevation Control
        float targetElPose = targetEl;
        if (targetElPose > maxLimit) targetElPose = maxLimit;
        if (targetElPose < minLimit) targetElPose = minLimit;
        float elError = targetElPose - chassis->getElPose();


        // Drive motors with proportional control
        // chassis->setSpeed(KpAz * azError, KpEl * elError);
        chassis->setSpeed(KpAz * azError, KpEl * elError);

        // Debugging (remove this later)
        // if (counter % 1000 == 0) {
        //     Serial.print("Azimuth Target: ");
        //     Serial.print(targetAzPose);

        //     Serial.print("  |  Elevation Target: ");
        //     Serial.print(targetElPose);

        //     // Serial.print("Elevation Error: ");
        //     // Serial.print(elError);

        //     // Serial.print("  |  Target Elevation: ");
        //     // Serial.print(targetElPose);

        //     // Serial.print("  |  Current Elevation: ");
        //     // Serial.println(chassis->getElPose());
        // }

        // counter++;
    }

    if (trackerState == TRACKER_REMOTE) {
        // chassis->setSpeed(az, el);
        chassis->setSpeed(azInput, elInput);
        // Serial.println("Current Elevation: " + String(chassis->getElPose()));
    }

    chassis->chassisLoop();
}


void Tracker::initializeTracker() {
    chassis = new Chassis();
    chassis->initializeChassis();
    
    chassis->setMaxSpeed(1.571, 0.523);
    chassis->setAccel(0.005, 0.005);
}

void Tracker::enterIdleState() {
    Serial.println("Entering Idle State");
    chassis->setSpeed(0, 0);
    chassis->enable(false, false);
    trackerState = TRACKER_IDLE;
}

void Tracker::enterTrackingState() {
    Serial.println("Entering Tracking State");
    chassis->setSpeed(0, 0);
    chassis->enable(true, true);
    trackerState = TRACKER_TRACKING;
}

void Tracker::enterRemoteState() {
    Serial.println("Entering Remote State");
    chassis->setSpeed(0, 0);
    chassis->enable(true, true);
    trackerState = TRACKER_REMOTE;
}

void Tracker::enterCalibratingState() {
    Serial.println("Entering Calibrating State");
    chassis->setSpeed(0, 0);
    chassis->enable(true, true);
    trackerState = TRACKER_CALIBRATING;
}