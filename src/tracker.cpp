#include "tracker.h"
#include "serial_comm.h"

void Tracker::trackerLoop() {
    
    if (checkSerialInput()) {
        parseSerialInput();
    }

    if (trackerState == TRACKER_TRACKING) {
        azimuth->runSpeed();
    }

    // azimuth->runSpeed();

    // delay(100);

    // Check for Keyboard Input
    // if (Serial.available() > 0) {
    //     char incomingChar = Serial.read();

    //     switch (incomingChar) {
    //         case 'a': // Turn Left
    //             Serial.println("Azimuth: Moving Left");
    //             azimuth->setSpeed(-MAX_STEPS_PER_SEC);
    //             break;

    //         case 'd': // Turn Right
    //             Serial.println("Azimuth: Moving Right");
    //             azimuth->setSpeed(MAX_STEPS_PER_SEC);
    //             break;

    //         case 's': // Stop
    //             Serial.println("Azimuth: Stopping");
    //             azimuth->setSpeed(0);
    //             break;

    //         case 'f': // Toggle Free Motor / Enable
    //             motorsEnabled = !motorsEnabled;
    //             digitalWrite(AZ_EN_PIN, motorsEnabled ? LOW : HIGH);
    //             // digitalWrite(EL_EN_PIN, motorsEnabled ? LOW : HIGH);
    //             Serial.print("Motors: ");
    //             Serial.println(motorsEnabled ? "ENABLED" : "FREE (DISABLED)");
    //             break;
    //     }
    // }

    // // Constant speed movement requires runSpeed()
    // if (motorsEnabled) {
    //     azimuth->runSpeed();
    // }
}

void Tracker::initializeTracker() {
    azimuth = new AccelStepper(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);

    // Set up Enable Pin
    pinMode(AZ_EN_PIN, OUTPUT);
    digitalWrite(AZ_EN_PIN, HIGH); // Most drivers are Active HIGH to disable to start

    // Configure Azimuth
    azimuth->setMaxSpeed(MAX_STEPS_PER_SEC);
    azimuth->setAcceleration(500); // Smooth start/stop
    azimuth->setMinPulseWidth(10);

//     Serial.println("Teensy Stepper Control Ready.");
//     Serial.println("Controls: [a] Left, [d] Right, [s] Stop, [f] Free/Toggle Enable");
}

void Tracker::enterIdleState() {
    // Serial.println("Entering IDLE state...");
    azimuth->setSpeed(0);
    digitalWrite(AZ_EN_PIN, HIGH);
    trackerState = TRACKER_IDLE;
}

void Tracker::enterTrackingState() {
    // Serial.println("Entering TRACKING state...");
    digitalWrite(AZ_EN_PIN, LOW);
    trackerState = TRACKER_TRACKING;
}