#pragma once

#define PI 3.14159265

// --- Configuration ---
const int STEPS_PER_REV = 1600;
const int AZ_GEAR_REDUCTION = 40;
const int EL_GEAR_REDUCTION = 40;
const int MAX_RPM = 1000; // Physical limit = 1500RPM, DONT APPROACH
const int ACCELERATION = 50;
const int PULSE_WIDTH = 10;

// --- Azimuth Pin Definitions ---
const int AZ_STEP_PIN = 12;
const int AZ_DIR_PIN  = 9;
const int AZ_EN_PIN   = 6;

// --- Elevation Pin Definitions ---
const int EL_STEP_PIN = 13;
const int EL_DIR_PIN  = 10;
const int EL_EN_PIN   = 11; 

class Chassis
{
public:
    Chassis(void);
    void initializeChassis(void);
    bool chassisLoop(void);

    void setSpeed(float azSpeed, float elSpeed);
    void stop(bool az, bool el);
    void enable(bool az, bool el);
    
    void setMaxSpeed(float azSpeed, float elSpeed);
    void setAccel(float azAccel, float elAccel);

protected:
    // Helper Functions

    /// @brief Converts motor steps to output radians for azimuth axis
    /// @param steps Number of motor steps
    /// @return Azimuth radians
    float azStepToRad(float steps) {
        float rads = steps / (STEPS_PER_REV * AZ_GEAR_REDUCTION) * 2*PI;
        return rads;
    }

    /// @brief Converts motor steps to output radians for elevation axis
    /// @param steps Number of motor steps
    /// @return Elevation radians
    float elStepToRad(float steps) {
        float rads = steps / (STEPS_PER_REV * EL_GEAR_REDUCTION) * 2*PI;
        return rads;
    }
    
    /// @brief Converts radians to motor steps for azimuth axis
    /// @param rads Number of radians
    /// @return Azimuth motor steps
    float azRadToStep(float rads) {
        float steps = rads / 2*PI * (STEPS_PER_REV * AZ_GEAR_REDUCTION);
        return steps;
    }

    /// @brief Converts radians to motor steps for elevation axis
    /// @param rads Number of radians
    /// @return Elevation motor steps
    float elRadToStep(float rads) {
        float steps = rads / 2*PI * (STEPS_PER_REV * EL_GEAR_REDUCTION);
        return steps;
    }


};