#pragma once

#include <AccelStepper.h>
#include <Arduino.h>
#include <math.h>
#include <Encoder.h>
#include "hallEffect.h"

class Chassis
{
protected:
    // --- Configuration ---
    const int STEPS_PER_REV = 1600;
    const int AZ_GEAR_REDUCTION = 40;
    const int EL_GEAR_REDUCTION = 40;
    const int MAX_RPM = 1000; // Physical limit = 1500RPM, DONT APPROACH
    const int ACCELERATION = 50;
    const int PULSE_WIDTH = 10;

    // --- Azimuth Pin Definitions ---
    const uint8_t AZ_STEP_PIN = 12;
    const uint8_t AZ_DIR_PIN  = 9;
    const uint8_t AZ_EN_PIN   = 6;

    const uint8_t AZ_ALRM_PIN = 28;
    const uint8_t AZ_ENCA_PIN = 33;
    const uint8_t AZ_ENCB_PIN = 32;

    // --- Elevation Pin Definitions ---
    const uint8_t EL_STEP_PIN = 13;
    const uint8_t EL_DIR_PIN  = 10;
    const uint8_t EL_EN_PIN   = 11;
    
    const uint8_t EL_ALRM_PIN = 22;
    const uint8_t EL_ENCA_PIN = 30;
    const uint8_t EL_ENCB_PIN = 31;

    const uint8_t FRONT_HALL_PIN = 18; // TODO: DOUBLE CHECK THESE
    const uint8_t REAR_HALL_PIN  = 19;

    // --- Hall Effect / Encoder Params ---
    const int32_t FRONT_ENCODER_COUNT = -999; // TODO: Update these
    const int32_t REAR_ENCODER_COUNT = -999;

    // Stepper Drivers
    AccelStepper* azMotor = nullptr;
    AccelStepper* elMotor = nullptr;
    
    // Encoders
    Encoder* azEncoder = nullptr;
    Encoder* elEncoder = nullptr;

    // Hall Effect Sensors
    HallEffect* frontHall = nullptr;
    HallEffect* rearHall = nullptr;

    // Calibration Parameters
    bool azCalibrated = false;
    bool elCalibrated = false;

    // Hall Effect Parameters
    bool frontPrevDetected = false;
    bool rearPrevDetected = false;

    // Alarm Parameters
    bool prevAzAlarm = false;
    bool prevElAlarm = false;

    bool prevAzEnState = false;
    bool prevElEnState = false;

    // Stepper Enable Parameters
    bool azEnabled = true;
    bool elEnabled = true;

public:
    void initializeChassis(void);
    void chassisLoop(void);

    void setSpeed(float azSpeed, float elSpeed);
    void stop(bool az, bool el);
    void enable(bool az, bool el);
    
    void setMaxSpeed(float azSpeed, float elSpeed);
    void setAccel(float azAccel, float elAccel);

protected:
    // Checkers and Handlers

    bool checkFrontHall(void);
    void handleFrontHall(void);

    bool checkRearHall(void);
    void handleRearHall(void);

    bool checkAzAlarm(void);
    void handleAzAlarm(void);

    bool checkElAlarm(void);
    void handleElAlarm(void);


    // Helper Functions

    /// @brief Converts motor steps to output radians for azimuth axis
    /// @param steps Number of motor steps
    /// @return Azimuth radians
    float azStepToRad(float steps) {
        float rads = steps / (STEPS_PER_REV * AZ_GEAR_REDUCTION) * (2*PI);
        return rads;
    }

    /// @brief Converts motor steps to output radians for elevation axis
    /// @param steps Number of motor steps
    /// @return Elevation radians
    float elStepToRad(float steps) {
        float rads = steps / (STEPS_PER_REV * EL_GEAR_REDUCTION) * (2*PI);
        return rads;
    }
    
    /// @brief Converts radians to motor steps for azimuth axis
    /// @param rads Number of radians
    /// @return Azimuth motor steps
    float azRadToStep(float rads) {
        float steps = rads / (2*PI) * (STEPS_PER_REV * AZ_GEAR_REDUCTION);
        return steps;
    }

    /// @brief Converts radians to motor steps for elevation axis
    /// @param rads Number of radians
    /// @return Elevation motor steps
    float elRadToStep(float rads) {
        float steps = rads / (2*PI) * (STEPS_PER_REV * EL_GEAR_REDUCTION);
        return steps;
    }

};