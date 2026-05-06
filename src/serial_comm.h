#pragma once

#include "tracker.h"

bool Tracker::checkSerialInput() {
    bool messageReceived = false;
    // Process ALL waiting bytes in the buffer
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            if (serialInput.length() >= 2) {
                data = serialInput.substring(2);
            }
            messageReceived = true; 
            // Don't 'return true' here; let it finish clearing the buffer 
            // if there's a second message waiting
        } else {
            serialInput += c;
        }
    }
    return messageReceived;
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
            if (trackerState == TRACKER_REMOTE) {
                chassis->setSpeed(az, 0);
            }
        }
        break;
    }

    case 'S': // State Input "S,TRACKING"
        data.trim(); // Clean up hidden \r or spaces
        if (data == "REMOTE") enterRemoteState();
        else if (data == "IDLE") enterIdleState();
        break;
  }

  // CRITICAL: Clear the buffer for the next controller packet
  serialInput = "";
  data = "";
}