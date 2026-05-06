#pragma once

#include "tracker.h"

bool Tracker::checkSerialInput() {
    // Use while instead of if to clear the buffer faster
    while(Serial.available() > 0) {
        char c = Serial.read();
        
        if(c == '\n') {
        // Message is complete! Now prepare the data string.
            if (serialInput.length() >= 2) {
                data = serialInput.substring(2); 
            }
            return true; 
        } else {
        serialInput += c;
        }
    }

    return false;
}

void Tracker::parseSerialInput()
{
  if (serialInput.length() == 0) return;

  switch(serialInput[0])
  {
    case 'V': { 
        int commaIndex = data.indexOf(',');
        if (commaIndex != -1) {
            float az = data.substring(0, commaIndex).toFloat();
            
            // Only move if we are in a state that allows manual control
            if (trackerState == TRACKER_TRACKING) {
                azimuth->setSpeed(az * MAX_STEPS_PER_SEC);
            }
        }
        break;
    }

    case 'S': // State Input "S,TRACKING"
        data.trim(); // Clean up hidden \r or spaces
        if (data == "TRACKING") enterTrackingState();
        else if (data == "IDLE") enterIdleState();
        break;
  }

  // CRITICAL: Clear the buffer for the next controller packet
  serialInput = "";
  data = "";
}