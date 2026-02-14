#include <Arduino.h>
#include "tracker.h"

/**
 * Code was relocated to chassis
 * There will be two sorts of control/PID loops nested within each other:
 * 
 * The chassis PID loop will take positions/velocities to ensure that the motors get to the desired positions/velocities as quickly as possible
 * 
 * The tracker-control loop will be inputting positions and velocities generated from rocket data after being smoothed/filtered
 */