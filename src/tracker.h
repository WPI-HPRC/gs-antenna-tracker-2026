#pragma once

#include <AccelStepper.h>

class Tracker
{
protected:
    // --- Configuration ---
    const int STEPS_PER_REV = 1600; // Standard 1.8 degree motor
    const float MAX_RPM = 60.0;
    const float GEAR_REDUCTION = 10.0;
    const float MAX_STEPS_PER_SEC = (MAX_RPM * STEPS_PER_REV) * GEAR_REDUCTION / 60;

    // --- Azimuth Pin Definitions ---
    const int AZ_STEP_PIN = 12;
    const int AZ_DIR_PIN  = 9;
    const int AZ_EN_PIN   = 6; // "Free motor" pin (Enable)

    bool motorsEnabled = false; // Start with motors disabled for safety

    AccelStepper* azimuth;

    String serialInput;
    String data;

public:
    void initializeTracker(void);
    void trackerLoop(void);

protected:
    void enterIdleState(void);
    void enterTrackingState(void);

    bool checkSerialInput(void);
    void parseSerialInput(void);

    // Enumerate tracker states
    enum TRACKER_STATE
    {
        TRACKER_IDLE,
        TRACKER_CALIBRATING,
        TRACKER_TRACKING
    };

    TRACKER_STATE trackerState = TRACKER_IDLE;
};