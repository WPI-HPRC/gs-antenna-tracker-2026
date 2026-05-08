#include "tracker.h"
#include "serial_comm.h"

void Tracker::trackerLoop() {
    
    if (checkSerialInput()) parseSerialInput();

    if (trackerState == TRACKER_CALIBRATING) {
        if (!elCalibrated) {
            chassis->calibrateEl(false);
            elCalibrated = true;
        }
    }

    if (trackerState == TRACKER_TRACKING) {
        // float azError = targetAz - chassis->getAzPose();

        float targetElPose = targetEl + PI/2;

        if (targetElPose > PI) targetElPose = PI;
        if (targetElPose < 0) targetElPose = 0;

        float elError = targetElPose - chassis->getElPose();
        chassis->setSpeed(0, KpEl * elError);

        if (counter % 1000 == 0) {
            Serial.print(">Elevation Error: ");
            Serial.print(elError);

            Serial.print("   >Target Elevation: ");
            Serial.print(targetElPose);

            Serial.print("   >Current Elevation: ");
            Serial.println(chassis->getElPose());
        }

        counter++;
    }

    chassis->chassisLoop();
}


void Tracker::initializeTracker() {
    chassis = new Chassis();
    chassis->initializeChassis();
    
    chassis->setMaxSpeed(0.524, 0.524);
}

void Tracker::enterIdleState() {
    if (trackerState == TRACKER_CALIBRATING) {
        chassis->calibrateAz(true);
    }

    Serial.println("Entering Idle State");
    chassis->setSpeed(0, 0);
    chassis->enable(false, false);
    trackerState = TRACKER_IDLE;
}

void Tracker::enterTrackingState() {
    Serial.println("Entering Tracking State");
    chassis->enable(true, true);
    trackerState = TRACKER_TRACKING;
}

void Tracker::enterRemoteState() {
    Serial.println("Entering Remote State");
    chassis->enable(true, true);
    trackerState = TRACKER_REMOTE;
}

void Tracker::enterCalibratingState() {
    Serial.println("Entering Calibrating State");
    chassis->enable(true, true);
    trackerState = TRACKER_CALIBRATING;
}