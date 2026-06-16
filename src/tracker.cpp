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
        targetAz = azInput * PI;
        targetEl = elInput * PI + PI/2;

        // Azimuth Control
        float targetAzPose = targetAz;
        float azError = targetAzPose - chassis->getAzPose();

        // Elevation Control
        float targetElPose = targetEl;
        if (targetElPose > PI) targetElPose = PI;
        if (targetElPose < 0) targetElPose = 0;
        float elError = targetElPose - chassis->getElPose();


        // Drive motors with proportional control
        // chassis->setSpeed(KpAz * azError, KpEl * elError);
        chassis->setSpeed(KpAz * azError, KpEl * elError);

        // // Debugging (remove this later)
        // if (counter % 1000 == 0) {
        //     Serial.print("Elevation Error: ");
        //     Serial.print(elError);

        //     Serial.print("  |  Target Elevation: ");
        //     Serial.print(targetElPose);

        //     Serial.print("  |  Current Elevation: ");
        //     Serial.println(chassis->getElPose());
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
    
    chassis->setMaxSpeed(0.785, 0.262);
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