#ifndef TRACKER_CONTROL_H
#define TRACKER_CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>

struct TrackerAxis {
    AccelStepper* stepper;
    float stepsPerDegree;
    
    // Limits
    float minAngle;
    float maxAngle;
    bool wrapsAround;

    // PID Tuning
    float Kp; 
    float Ki; // integral term which we may or may not use
    float Kd;

    // Memory (Internal State)
    float lastError;
    float integralSum; // integral sum which will only be used if we use the integral term
};

// Functions
void initTracker();
float getShortestError(float currentAngle, float targetAngle);
void runTrackerControl(float azTargetDeg, float elTargetDeg);

#endif