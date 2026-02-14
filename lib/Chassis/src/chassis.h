#pragma once

#include "stepperDriver.h"
#include "encoder.h"
#include "utils.h"

class Chassis
{
protected: 
    // Have multiple protected sections so the easy to reach sections are up top
    const int AZ_STEP_PIN = 1, AZ_DIR_PIN  = 2;
    const int AZ_A_PIN = 5, AZ_B_PIN = 6;

    const int EL_STEP_PIN = 3, EL_DIR_PIN  = 4;
    const int EL_A_PIN = 7, EL_B_PIN = 8;

    float azKp = 5.0, azKi = 0.0, azKd = 0.1;
    float elKp = 5.0, elKi = 0.0, elKd = 0.1;

    float elMinAngle = -5, elMaxAngle = 185;

public:
    Chassis();

    void InitializeChassis(void);
    bool ChassisLoop(Pose&);
    
    void Stop(void);
    void SetSpeed(Twist&);
    Pose GetError(Pose targetPose);
    float CalculatePID(TrackerAxis &axis, float currentAngle, float targetAngle, float dt);


protected:
    StepperDriver azimuthDriver;
    StepperDriver elevationDriver;

    Encoder azimuthEncoder;
    Encoder elevationEncoder;

    // Memory (Internal State)
    float lastError;
    float integralSum; // integral sum which will only be used if we use the integral term
    unsigned long lastTime = 0;
};