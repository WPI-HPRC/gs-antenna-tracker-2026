
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

    float azimuthSpeed; // rad/s
    float elevationSpeed; // rad/s
    
    float azimuthMaxSpeed = 0.1; // rad/s
    float elevationMaxSpeed = 0.1; // rad/s

    /// Define pin numbers
    int azEncAPin = 33;
    int azEncBPin = 32;
    int azAlarmPin = 28;
    int elEncAPin = 30;
    int elEncBPin = 31;
    int elAlarmPin = 22;

    int elPulsePin = 6;
    int elDirPin = 9;
    int elEnablePin = 12;
    int azPulsePin = 11;
    int azDirPin = 13;
    int azEnablePin = 10;

    int CWHallPin = 18;
    int CCWHallPin = 19;

};