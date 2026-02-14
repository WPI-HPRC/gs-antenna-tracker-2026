#pragma once
#include <Arduino.h>

/**
 * Pose includes information about the elevation and azimuth axis.
 */
struct Pose
{
    float el = 0;
    float az = 0;

    Pose(void) {}
    Pose(float el_, float az_) : el(el_), az(az_) {}
};

/**
 * Twist is very similar to Pose, but we make a separate struct to avoid confusion.
 * 
 * Whereas Pose is position, Twist contains ang. vel.
 */
struct Twist
{
    float elVel = 0;
    float azVel = 0;

    Twist(void) {}
    Twist(float elVel_, float azVel_) : elVel(elVel_), azVel(azVel_) {}
};
