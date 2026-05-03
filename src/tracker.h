#pragma once

#include "chassis.h"
#include <Arduino.h>

class Tracker
{
public:
    void initializeTracker(void);
    void trackerLoop(void);

protected:
    Chassis chassis;

    void enterIdleState(void);
    void enterCalibratingState(void);
    void enterTrackingState(void);
    void enterManualState(void);

    bool checkSerialInput(void);
    void parseSerialInput(void);
    void parseSerialCommand(String cmd);

    // Enumerate tracker states
    enum TRACKER_STATE
    {
        TRACKER_IDLE,
        TRACKER_CALIBRATING,
        TRACKER_TRACKING,
        TRACKER_MANUAL
    };

    TRACKER_STATE trackerState = TRACKER_IDLE;
};