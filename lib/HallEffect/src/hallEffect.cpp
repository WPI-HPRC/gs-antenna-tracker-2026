#include "hallEffect.h"

/// @brief Creates HallEffect class
/// @param pin Pin number for data to be read from
HallEffect::HallEffect(uint8_t pin) {
    pinNum = pin;
}

/// @brief Initializes hall effect sensor pins
void HallEffect::initializeHallEffect() {
    pinMode(pinNum, INPUT_PULLUP);
}

/// @brief Reads the hall effect sensor
/// @return Returns true if magnet detected, false otherwise
bool HallEffect::readSensor() {
    // Updates reading history
    if (analogRead(pinNum) < 600) { // Threshold for detection
        prevReadings[readingCount % arrSize] = true;
    } else {
        prevReadings[readingCount % arrSize] = false;
    }
    readingCount++;

    // If all readings are the same, update recentState
    bool result = true;
    for (int i = 1; i < arrSize; i++) {
        if (prevReadings[i] != prevReadings[0]) {
            result = false;
            break;
        }
    }

    if (result) {
        recentState = prevReadings[0];
    }

    return recentState;
}