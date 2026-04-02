
#pragma once

class StepperDriver
{
public:
    StepperDriver(void);
    void initializeStepperDriver(void);
    
    void setSpeed(float speed);
    void setMaxSpeed(float maxSpeed);
    void stop(void);
    void freeMotor(void);
    
protected:

    int STEP_PIN;
    int DIR_PIN;
    int ENABLE_PIN;

    float speed; // rad/s
    float maxSpeed; // rad/s

};