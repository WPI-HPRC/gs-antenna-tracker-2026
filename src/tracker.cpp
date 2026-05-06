#include "tracker.h"
#include "serial_comm.h"

void Tracker::trackerLoop() {
    
    if (checkSerialInput()) parseSerialInput();

    chassis->chassisLoop();
}

void Tracker::initializeTracker() {
    chassis = new Chassis();
    chassis->initializeChassis();
}

void Tracker::enterIdleState() {
    chassis->setSpeed(0, 0);
    chassis->enable(false, false);
    trackerState = TRACKER_IDLE;
}

void Tracker::enterTrackingState() {
    chassis->enable(true, true);
    trackerState = TRACKER_TRACKING;
}

void Tracker::enterRemoteState() {
    chassis->enable(true, true);
    trackerState = TRACKER_REMOTE;
}