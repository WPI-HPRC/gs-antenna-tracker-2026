#include "hallEffect.h"
#include <Arduino.h>

/// @brief Creates HallEffect class
/// @param pin Pin number for data to be read from
HallEffect::HallEffect(int pin) {
    pinNum = pin;
}

/// @brief Initializes hall effect sensor pins
void HallEffect::initializeHallEffect() {
    pinMode(pinNum, INPUT);
}

/// @brief Reads the hall effect sensor
/// @return Returns true if magnet detected, false otherwise
bool HallEffect::readSensor() {
    if (digitalRead(pinNum)) return true;
    return false;
}