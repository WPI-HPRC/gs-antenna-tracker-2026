/**
 * Holds all the function prototypes and variables related to tracker control and the state machine.
 * There should be no function bodies defined here, only variables and prototypes.
 */

#pragma once // This replaces #ifndef...

#include "chassis.h"

class Tracker
{
public:
    Tracker(void);
    void InitializeTracker(void);
    void TrackerLoop(void);

protected:
    void EnterIdleState(void);
    void EnterCalibratingState(void);
    void EnterTrackingState(void);

    bool CheckSerialInput(void);
    void ParseSerialInput(void);

    // Enumerate tracker states
    enum TRACKER_STATE
    {
        TRACKER_IDLE,
        TRACKER_CALIBRATING,
        TRACKER_TRACKING
    };

    TRACKER_STATE trackerState = TRACKER_IDLE;
};