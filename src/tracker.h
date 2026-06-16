#pragma once

#include <AccelStepper.h>
#include "chassis.h"

class Tracker
{
protected:
    // Configuration Parameters
    float KpAz = 15.0;
    float KpEl = 15.0;

public:
    void initializeTracker(void);
    void trackerLoop(void);

protected:
    Chassis* chassis;

    void enterIdleState(void);
    void enterTrackingState(void);
    void enterRemoteState(void);
    void enterCalibratingState(void);

    bool checkSerialInput(void);
    void parseSerialInput(void);

    // Calibration Variables
    bool elCalibrated = false;
    bool azCalibrated = false;
    long int counter = 0;

    // Remote Control Variables
    float targetAz = 0;
    float targetEl = 0;
    float maxLimit = 1.92; // 90 degrees in radians
    float minLimit = 0;    // 0 degrees in radians

    // Serial communication variables
    char serialBuffer[64];
    int bufIdx = 0;
    String serialInput;
    String data;
    bool updateFlag = false;
    float azInput = 0;
    float elInput = 0;

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