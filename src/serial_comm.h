// #ifndef __SERIAL_COMM_H
// #define __SERIAL_COMM_H

// #include <AccelStepper.h>
// #include "tracker.h"

// String serialInput;
// String data = serialInput.substring(2);

// bool Tracker::checkSerialInput()
// {
//   if(Serial.available())
//   {
//     char c = Serial.read();
//     char d = Serial.read(); 
//     serialInput += c;
//     if(c == '\n'&&d=='\n') return true;
//     Serial.println(serialInput);
//   }

//   return false;
// }

// void Tracker::parseSerialInput()
// {
//   float value;
//   switch(serialInput[0])
//   {
//     case 'I': // Idle
//         enterIdleState();
//         break;

//     case 'T': // Tracking
//         enterTrackingState();
//         break;

//     case 'V': { // Controller Velocity Input, "V,0.500,-0.200"
//         int commaIndex = data.indexOf(',');
//             if (commaIndex != -1) {
//                 float az = data.substring(0, commaIndex).toFloat();
//                 float el = data.substring(commaIndex + 1).toFloat();
                
//                 if (trackerState == TRACKER_TRACKING) {
//                     // Map velocity from -1.0 to 1.0 to max speed
//                     azimuth->setSpeed(az * MAX_STEPS_PER_SEC);
//                     // elevation->setSpeed(el * MAX_STEPS_PER_SEC);
//                 }
//             }
//         break;
//     }

//     case 'S': // Controller State Input, "S,MANUAL" (State Change)
//         if (data == "TRACKING") enterTrackingState();
//         else if (data == "IDLE") enterIdleState();
//         break;

//   }

//   serialInput = "";
// }

// #endif