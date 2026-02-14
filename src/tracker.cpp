/**
 * This is gonna be the very high level stuff, mostly just state machine control
 * and no control loops or direct motor control. Basically just a TrackerLoop()
 * function that goes through a set of checkers and handlers to run our state machine.
 */

#include "tracker.h"

Chassis chassis;

void Tracker::InitializeTracker(void)
{

}

void Tracker::EnterIdleState(void)
{
    chassis.Stop();
    trackerState = TRACKER_IDLE;
}

void Tracker::EnterCalibratingState(void)
{
    trackerState = TRACKER_CALIBRATING;
}

void Tracker::EnterTrackingState(void)
{
    trackerState = TRACKER_TRACKING;
}

/**
 * The main loop for the tracker. Processes both synchronous events (motor control),
 * and asynchronous events (hall effect readings, encoder inputs, etc.)
 */
void Tracker::TrackerLoop(void)
{
    if(trackerState == TRACKER_IDLE)
    {

    } 
    if (trackerState == TRACKER_CALIBRATING)
    {
        
    }
    if (trackerState == TRACKER_TRACKING)
    {
        chassis.ChassisLoop();
    }

    // Checkers and Handlers
    // if(CheckHallEffect()) HandleHallEffect();
}