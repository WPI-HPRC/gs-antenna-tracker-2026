#pragma once

#include "tracker.h"

// In serial_comm.cpp
bool Tracker::checkSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            serialBuffer[bufIdx] = '\0'; // Cap the string
            bufIdx = 0;
            return true;
        } else if (bufIdx < 63) {
            serialBuffer[bufIdx++] = c;
        }
    }
    return false;
}

void Tracker::parseSerialInput()
{
    // 1. Check if the buffer has any data
    if (serialBuffer[0] == '\0') return;

    // 2. Use the first character of the buffer for the switch
    switch(serialBuffer[0])
    {
        case 'V': { 
            // serialBuffer looks like "V,0.500,-0.200"
            // We find the first comma (skipping the 'V')
            char* firstComma = strchr(serialBuffer, ',');
            if (firstComma != nullptr) {
                // dataStart points to the first number "0.500"
                char* dataStart = firstComma + 1; 
                
                // atof converts the C-string segment to a float
                float az = atof(dataStart);
                
                // Find the second comma to get the elevation
                char* secondComma = strchr(dataStart, ',');
                float el = 0;
                if (secondComma != nullptr) {
                    el = atof(secondComma + 1);
                }
                
                azInput = az;
                elInput = el;

                // Serial.print("azInput: " + String(azInput));
                // Serial.println("  |  elInput: " + String(elInput));
            }
            break;
        }

        case 'S': {
            // Find the comma to get the state string
            char* stateStart = strchr(serialBuffer, ',');
            if (stateStart != nullptr) {
                stateStart++; // Move past the comma
                
                // strcmp compares C-strings (returns 0 if they match)
                if (strcmp(stateStart, "REMOTE") == 0) enterRemoteState();
                else if (strcmp(stateStart, "IDLE") == 0) enterIdleState();
                else if (strcmp(stateStart, "TRACKING") == 0) enterTrackingState();
                else if (strcmp(stateStart, "CALIBRATE") == 0) enterCalibratingState();
            }
            break;
        }
    }
}