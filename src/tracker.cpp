#include "tracker.h"
#include "serial_comm.h"

void Tracker::initializeTracker(void) {
    chassis.initializeChassis();
}

void Tracker::trackerLoop(void)
{
    if (checkSerialInput() ) { parseSerialInput(); }

    chassis.chassisLoop();

    
    if(trackerState == TRACKER_IDLE) {
        
    }

    if (trackerState == TRACKER_CALIBRATING) {

    }

    if (trackerState == TRACKER_TRACKING) {

    }

    if (trackerState == TRACKER_MANUAL) {

    }
}

void Tracker::enterIdleState(void) {
    trackerState = TRACKER_IDLE;
    Serial.print("ENTERING IDLE STATE");

    chassis.stop(true, true);
    chassis.enable(false, false);
}

void Tracker::enterCalibratingState(void) {
    trackerState = TRACKER_CALIBRATING;
    Serial.println("ENTERIND CALIBRATING STATE");

    chassis.enable(true, true);
}

void Tracker::enterTrackingState(void) {
    trackerState = TRACKER_TRACKING;
    Serial.println("ENTERIND TRACKING STATE");

    chassis.enable(true, true);
}

void Tracker::enterManualState(void) {
    trackerState = TRACKER_MANUAL;
    Serial.println("ENTERIND MANUAL STATE");

    chassis.enable(true, true);
}

