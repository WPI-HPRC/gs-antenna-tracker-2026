/**
 * This is mostly gonna be to combine all the sensors and stepper driver controls
 * into one comprehensive system. This wont have any state machine control, but will
 * be directly controlled by tracker-control.cpp. This'll mostly just ensure desired
 * positioning variables are met using all the sensor data on the tracker's chassis.
 */

#include "chassis.h"
#include "encoder.h"

Chassis::Chassis()
    : azimuthDriver(AZ_STEP_PIN, AZ_DIR_PIN),
      elevationDriver(EL_STEP_PIN, EL_DIR_PIN),
      azimuthEncoder(AZ_A_PIN, AZ_B_PIN),
      elevationEncoder(EL_A_PIN, EL_B_PIN)
{
}

void Chassis::SetSpeed(Twist& twist)
{
    azimuthDriver.SetSpeed(twist.azVel);
    elevationDriver.SetSpeed(twist.elVel);
}



// Combine both error functions to one using a pose struct?

float Chassis::GetAzimuthError(float targetAngle) {
    float error = targetAngle - azimuthEncoder.ReadPosition();

    if (error > 180.0) {
        error -= 360.0;
    } else if (error < -180.0) {
        error += 360.0;
    }

    return error; // azimuth error is shortened between -180 and 180 to prevent the tracker from going the long way around
}

float Chassis::GetElevationError(float targetAngle) {
    float error = targetAngle - elevationEncoder.ReadPosition();

    if (error > 180.0) {
        error -= 360.0;
    } else if (error < -180.0) {
        error += 360.0;
    }

    return error;
}