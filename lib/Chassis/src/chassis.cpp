/**
 * This is mostly gonna be to combine all the sensors and stepper driver controls
 * into one comprehensive system. This wont have any state machine control, but will
 * be directly controlled by tracker-control.cpp. This'll mostly just ensure desired
 * positioning variables are met using all the sensor data on the tracker's chassis.
 */
#include "chassis.h"

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;

/**
 * For taking snapshots and raising the flag.
 */
void Chassis::Timer4OverflowISRHandler(void) 
{
    loopFlag++;

    // leftMotor.speed = leftMotor.CalcEncoderDelta();
    // rightMotor.speed = rightMotor.CalcEncoderDelta();
}

/**
 * ISR for timing. On Timer4 overflow, we take a 'snapshot' of the encoder counts 
 * and raise a flag to let the program it is time to execute the PID calculations.
 * it appears the teensy 3.2 may have a different timing system, might need to change this
 */
ISR(TIMER4_OVF_vect)
{
   Chassis::Timer4OverflowISRHandler();
}





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
    azimuthDriver.SetSpeed(0);
    elevationDriver.SetSpeed(0);
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

void Chassis::UpdatePose(Pose& chassisPose)
{
    chassisPose.az = azimuthEncoder.ReadPosition();
    chassisPose.el = elevationEncoder.ReadPosition();
}

void Chassis::UpdateTwist(Twist& chassisTwist)
{
    chassisTwist.azVel = azimuthEncoder.ReadVelocity();
    chassisTwist.elVel = elevationEncoder.ReadVelocity();
}

bool Chassis::ChassisLoop(Pose& chassisPose, Twist& chassisTwist)
{

    // Below code isn't finished, but will allow ChassisLoop() to run in set intervals using a timer

    bool retVal = false;

    if(loopFlag)
    {
        if(loopFlag > 1) Serial.println("Missed an update in Robot::RobotLoop()!");

#ifdef __LOOP_DEBUG__
        Serial.print(millis());
        Serial.print('\n');
#endif

        // motor updates
        //UpdateMotors();

        /* Update the wheel velocity so it gets back to Robot. */
        //velocity = CalcOdomFromWheelMotion();

        loopFlag = 0;

        retVal = true;
    }

    return retVal;






    unsigned long now = millis();
    
    // PID interval is 20ms
    if (now - lastTime < CONTROL_LOOP_PERIOD_MS) return false; 

    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // get speed from PID and set it
    Pose targetPose; // this should be the input pose for where we want it to go
    Twist calcTwist = CalculatePID(targetPose, dt);
    Chassis::SetSpeed(calcTwist);

    // Update data on position and angular velocity
    UpdatePose(chassisPose);
    UpdateTwist(chassisTwist);
}

// main pid loop, loosely based on https://ascendrobotics.gitbook.io/ascend/vex-robotics/coding/vexcode-pro/advanced/coding-pids/drive-pid-tutorial
Twist Chassis::CalculatePID(Pose targetAngle, float dt) {
    Pose totalError = GetError(targetAngle);
    Twist result;

    float azP = azKp * totalError.az;
    float azDerivative = totalError.az - azLastError;
    float azD = azKd * azDerivative;
    azLastError = totalError.az;
    result.azVel = azP + azD;

    float elP = elKp * totalError.el;
    float elDerivative = totalError.el - elLastError;
    float elD = elKd * elDerivative;
    elLastError = totalError.el;
    result.elVel = elP + elD;

    return result;
}