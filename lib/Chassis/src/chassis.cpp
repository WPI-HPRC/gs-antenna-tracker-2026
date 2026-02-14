/**
 * This is mostly gonna be to combine all the sensors and stepper driver controls
 * into one comprehensive system. This wont have any state machine control, but will
 * be directly controlled by tracker-control.cpp. This'll mostly just ensure desired
 * positioning variables are met using all the sensor data on the tracker's chassis.
 */

#include "chassis.h"


Chassis::Chassis()
    : azimuthDriver(AZ_STEP_PIN, AZ_DIR_PIN),
      elevationDriver(EL_STEP_PIN, EL_DIR_PIN),
      azimuthEncoder(AZ_A_PIN, AZ_B_PIN),
      elevationEncoder(EL_A_PIN, EL_B_PIN)
{
}

void Chassis::InitializeChassis()
{
    azimuthDriver.SetPID(azKp, azKi, azKd);
    elevationDriver.SetPID(elKp, elKi, elKd);
}

void Chassis::Stop()
{
    Twist zero = {0};
    SetSpeed(zero);
}

void Chassis::SetSpeed(Twist& twist)
{
    azimuthDriver.SetSpeed(twist.azVel);
    elevationDriver.SetSpeed(twist.elVel);
}

Pose Chassis::GetError(Pose targetPose) {
    Pose error;

    error.az = targetPose.az - azimuthEncoder.ReadPosition();
    error.el = targetPose.el - elevationEncoder.ReadPosition();

    if (error.az > 180.0) { error.az -= 360.0; }
    else if (error.az < -180.0) { error.az += 360.0; }

    return error; // Azimuth error is shortened between -180 and 180 to prevent the tracker from going the long way around
}

bool Chassis::ChassisLoop(Pose&)
{
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

// main pid loop, loosely based on https://ascendrobotics.gitbook.io/ascend/vex-robotics/coding/vexcode-pro/advanced/coding-pids/drive-pid-tutorial
float Chassis::CalculatePID(TrackerAxis &axis, float currentAngle, float targetAngle, float dt) {
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