/**
 * This is where most of the higher level control loop stuff will go, mostly
 * PID or whatever other control is gonna be used to drive the tracker to
 * the input angles/coordinates through USB communication.
 */
#include <AccelStepper.h>
#include <Arduino.h>
#include "tracker-control.h"

TrackerAxis azAxis, elAxis;
const float MAX_ACCEL = 500.0; //random numbers
const float MAX_SPEED = 4000.0;

extern AccelStepper stepperAz;
extern AccelStepper stepperEl;

float getShortestError(float currentAngle, float targetAngle) {
    float error = targetAngle - currentAngle;

    if (error > 180.0) {
        error -= 360.0;
    } else if (error < -180.0) {
        error += 360.0;
    }

    return error; // azimuth error is shortened between -180 and 180 to prevent the tracker from going the long way around
}

// main pid loop, loosely based on https://ascendrobotics.gitbook.io/ascend/vex-robotics/coding/vexcode-pro/advanced/coding-pids/drive-pid-tutorial
float calculatePID(TrackerAxis &axis, float currentAngle, float targetAngle, float dt) {
    float error;

    if(axis.wrapsAround) {
        error = getShortestError(currentAngle, targetAngle);
    } else {
        error = targetAngle - currentAngle;
    }

    float P = axis.Kp * error;
    float derivative = error - axis.lastError;
    float D = axis.Kd * derivative;
    float velocity = P + D;

    axis.lastError = error;

    return velocity * axis.stepsPerDegree;
}

void initTracker() {
    // azimuth setup
    azAxis.stepper = &stepperAz;
    azAxis.stepsPerDegree = 67; 
    azAxis.wrapsAround = true;
    
    azAxis.Kp = 5.0;  
    azAxis.Ki = 0.0;
    azAxis.Kd = 0.1;   

    // elevation setup
    elAxis.stepper = &stepperEl;
    elAxis.stepsPerDegree = 67;
    elAxis.wrapsAround = false;
    elAxis.minAngle = -5.0; 
    elAxis.maxAngle = 90.0;

    elAxis.Kp = 5.0;
    elAxis.Ki = 0.0;
    elAxis.Kd = 0.1;
}

unsigned long lastTime = 0;

void runTrackerControl(float azTargetDeg, float elTargetDeg) {
    unsigned long now = millis();
    
    // PID interval is 20ms
    if (now - lastTime < 20) return; 

    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // get current degree position
    float currentAz = azAxis.stepper->currentPosition() / azAxis.stepsPerDegree;
    // normalize degrees
    while (currentAz >= 360.0) currentAz -= 360.0;
    while (currentAz < 0.0)    currentAz += 360.0;

    // get speed from PID
    float speedAz = calculatePID(azAxis, currentAz, azTargetDeg, dt);
    
    // set the stepper to the calculated speed
    azAxis.stepper->setSpeed(speedAz);


    // elevation should be limited so it doesn't go to any weird positions or collide with objects
    if (elTargetDeg > elAxis.maxAngle) elTargetDeg = elAxis.maxAngle;
    if (elTargetDeg < elAxis.minAngle) elTargetDeg = elAxis.minAngle;

    float currentEl = elAxis.stepper->currentPosition() / elAxis.stepsPerDegree;

    float speedEl = calculatePID(elAxis, currentEl, elTargetDeg, dt);

    elAxis.stepper->setSpeed(speedEl);
}