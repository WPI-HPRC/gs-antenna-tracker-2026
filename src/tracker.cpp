#include "tracker.h"
#include "serial_comm.h"

void Tracker::initializeTracker() {
    chassis.initializeChassis();
    Serial.println("Tracker Initialized");
}

void Tracker::trackerLoop()
{
    if (checkSerialInput() ) { parseSerialInput(); }

    chassis.chassisLoop();

    enterIdleState();
    
    if(trackerState == TRACKER_IDLE) {
        
    }

    if (trackerState == TRACKER_CALIBRATING) {

    }

    if (trackerState == TRACKER_TRACKING) {

    }

    if (trackerState == TRACKER_MANUAL) {

    }
}

void Tracker::enterIdleState() {
    trackerState = TRACKER_IDLE;
    Serial.println("ENTERING IDLE STATE");

    chassis.stop(true, true);
    chassis.enable(false, false);
}

void Tracker::enterCalibratingState() {
    trackerState = TRACKER_CALIBRATING;
    Serial.println("ENTERING CALIBRATING STATE");

    chassis.enable(true, true);
}

void Tracker::enterTrackingState() {
    trackerState = TRACKER_TRACKING;
    Serial.println("ENTERING TRACKING STATE");

    chassis.enable(true, true);
}

void Tracker::enterManualState() {
    trackerState = TRACKER_MANUAL;
    Serial.println("ENTERING MANUAL STATE");

    chassis.enable(true, true);
}

