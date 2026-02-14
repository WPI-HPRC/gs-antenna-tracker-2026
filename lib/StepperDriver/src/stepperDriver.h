// arduino accel stepper library which steps the motors for us and is very convenient
// docs: https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
#include <AccelStepper.h>

class StepperDriver
{
public:
    StepperDriver(int stepPin, int dirPin);

    void InitializeStepperDriver(void);
    void SetSpeed(float speed);

protected:
    int dirPin;
    int stepPin;

    AccelStepper stepper;
};