#pragma once

#include "stepperDriver.h"
#include "encoder.h"
#include "utils.h"

class Chassis
{
protected:

    /**
     * You can change the control loop period, but you should use multiples of 4 ms to 
     * avoid rounding errors.
     */
    const uint16_t CONTROL_LOOP_PERIOD_MS = 20;

    /**
     * loopFlag is used to tell the program when to update. It is set when Timer4
     * overflows (see InitializeMotorControlTimer). Some of the calculations are too
     * lengthy for an ISR, so we set a flag and use that to key the calculations.
     * 
     * Note that we use in integer so we can see if we've missed a loop. If loopFlag is
     * more than 1, then we missed a cycle.
     */
    static uint8_t loopFlag;



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
    bool ChassisLoop(Pose& chassisPose, Twist& chassisTwist);
    
    void Stop(void);
    void SetSpeed(Twist&);
    Pose GetError(Pose targetPose);
    Twist CalculatePID(Pose targetAngle, float dt);
    void UpdatePose(Pose& chassisPose);
    void UpdateTwist(Twist& chassisTwist);



    /* Needed for managing motors. */
    static void Timer4OverflowISRHandler(void);

protected:
    StepperDriver azimuthDriver;
    StepperDriver elevationDriver;

    Encoder azimuthEncoder;
    Encoder elevationEncoder;

    //comment bool loopFlag = false;

    // Memory (Internal State)
    float azLastError;
    float elLastError;
    float integralSum; // integral sum which will only be used if we use the integral term
    unsigned long lastTime = 0;
};