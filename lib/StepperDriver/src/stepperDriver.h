// arduino accel stepper library which steps the motors for us and is very convenient
// docs: https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
#include <AccelStepper.h>

class StepperDriver
{
public:
    StepperDriver(int stepPin, int dirPin);

    void InitializeStepperDriver(void);
    void SetSpeed(float speed);

    void SetPID(float Kp_, float Ki_, float Kd_);

protected:
    int dirPin;
    int stepPin;

    float stepsPerDegree = -1;

    // PID Tuning
    float Kp;
    float Ki; // integral term which we may or may not use
    float Kd;

    AccelStepper stepper;
};