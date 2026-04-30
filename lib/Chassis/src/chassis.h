
#pragma once

class Chassis
{
public:
    Chassis(void);
    void initializeChassis(void);
    
    void setChassisSpeed(float azSpeed, float elSpeed);
    void stopChassis(void);
    void chassisESTOP(void);
    
protected:
    
    void chassisLoop(void);
    void updateChassis(void);
    
    float azimuthMaxSpeed = 0.1; // rad/s
    float elMaxSpeed = 0.1; // rad/s

    float azAcceleration = 0.1; // rad/s^2
    float elAcceleration = 0.1; // rad/s^2

    /// Define pin numbers
    int AZ_ENC_A_PIN = 33;
    int AZ_ENC_B_PIN = 32;
    int AZ_ALARM_PIN = 28;
    int EL_ENC_A_PIN = 30;
    int EL_ENC_B_PIN = 31;
    int EL_ALARM_PIN = 22;

    int EL_PULSE_PIN = 6;
    int EL_DIR_PIN = 9;
    int EL_ENABLE_PIN = 12;
    int AZ_PULSE_PIN = 11;
    int AZ_DIR_PIN = 13;
    int AZ_ENABLE_PIN = 10;

    int CW_HALL_PIN = 18;
    int CCW_HALL_PIN = 19;

};