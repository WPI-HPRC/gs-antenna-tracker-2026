#pragma once

#include "stepperDriver.h"
#include "encoder.h"
#include "utils.h"

class Chassis
{
protected: 
    // Have multiple protected sections so the easy to reach sections are up top
    const int AZ_STEP_PIN = 1;
    const int AZ_DIR_PIN  = 2;
    
    const int EL_STEP_PIN = 3;
    const int EL_DIR_PIN  = 4;

    const int AZ_A_PIN = 5;
    const int AZ_B_PIN = 6;

    const int EL_A_PIN = 7;
    const int EL_B_PIN = 8;

public:
    Chassis();

    void InitializeChassis(void);
    bool ChassisLoop(Pose&);
    void SetSpeed(Twist&);

    float GetAzimuthError(float targetAngle);
    float GetElevationError(float targetAngle);

protected:
    StepperDriver azimuthDriver;
    StepperDriver elevationDriver;

    Encoder azimuthEncoder;
    Encoder elevationEncoder;
};