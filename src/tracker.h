#pragma once

#include <AccelStepper.h>
#include "chassis.h"

class Tracker
{
public:
    void initializeTracker(void);
    void trackerLoop(void);

protected:
    Chassis* chassis;

    void enterIdleState(void);
    void enterTrackingState(void);
    void enterRemoteState(void);

    bool checkSerialInput(void);
    void parseSerialInput(void);

    // Serial communication variables
    char serialBuffer[64];
    int bufIdx = 0;
    String serialInput;
    String data;

    // Enumerate tracker states
    enum TRACKER_STATE
    {
        TRACKER_IDLE,
        TRACKER_CALIBRATING,
        TRACKER_TRACKING,
        TRACKER_REMOTE
    };

    TRACKER_STATE trackerState = TRACKER_IDLE;
};