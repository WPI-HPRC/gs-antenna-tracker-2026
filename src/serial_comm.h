#ifndef __SERIAL_COMM_H
#define __SERIAL_COMM_H

#include <Arduino.h>
#include "tracker.h"

String serialInput;
String data = serialInput.substring(2);

bool Tracker::checkSerialInput(void)
{
  if(Serial.available())
  {
    char c = Serial.read();
    char d = Serial.read(); 
    serialInput += c;
    if(c == '\n'&&d=='\n') return true;
    Serial.println(serialInput);
  }

  return false;
}

void Tracker::parseSerialInput(void)
{
  float value;
  switch(serialInput[0])
  {
    case 'C': // Calibrate
        enterCalibratingState();
        break;

    case 'I': // Idle
        enterIdleState();
        break;

    case 'T': // Tracking
        enterTrackingState();
        break;

    case 'M': // Manual
        enterManualState();
        break;

    case 'V': { // Controller Velocity Input, "V,0.500,-0.200"
        int commaIndex = data.indexOf(',');
            if (commaIndex != -1) {
                float az = data.substring(0, commaIndex).toFloat();
                float el = data.substring(commaIndex + 1).toFloat();
                
                if (trackerState == TRACKER_MANUAL) {
                    chassis.setSpeed(az/10.0, el/10.0);
                }
            }
        break;
    }

    case 'S': // Controller State Input, "S,MANUAL" (State Change)
        if (data == "MANUAL") enterManualState();
        else if (data == "TRACKING") enterTrackingState();
        else if (data == "IDLE") enterIdleState();
        break;

  }

  serialInput = "";
}

#endif