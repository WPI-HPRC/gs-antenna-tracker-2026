#include <AccelStepper.h>

// --- Configuration ---
const int STEPS_PER_REV = 1600; // Standard 1.8 degree motor
const float MAX_RPM = 30.0;
const float GEAR_REDUCTION = 10.0;
const float MAX_STEPS_PER_SEC = (MAX_RPM * GEAR_REDUCTION * STEPS_PER_REV) / 60.0;
// --- Azimuth Pin Definitions ---
const int AZ_STEP_PIN = 12;
const int AZ_DIR_PIN  = 9;
const int AZ_EN_PIN   = 6; // "Free motor" pin (Enable)

/* 
// --- Elevation Pin Definitions (Commented Out) ---
const int EL_STEP_PIN = 13;
const int EL_DIR_PIN  = 10;
const int EL_EN_PIN   = 11; 
*/

// Initialize the stepper(s)
// Interface type 1 means an external driver with Step and Direction pins
AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
// AccelStepper elevation(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

bool motorsEnabled = true;

void setup() {
  Serial.begin(115200);
  
  // Set up Enable Pin
  pinMode(AZ_EN_PIN, OUTPUT);
  digitalWrite(AZ_EN_PIN, LOW); // Most drivers are Active LOW to enable

  // Configure Azimuth
  azimuth.setMaxSpeed(MAX_STEPS_PER_SEC);
  azimuth.setAcceleration(500); // Smooth start/stop
  azimuth.setMinPulseWidth(10);

  /*
  // Configure Elevation (Commented Out)
  pinMode(EL_EN_PIN, OUTPUT);
  digitalWrite(EL_EN_PIN, LOW);
  elevation.setMaxSpeed(MAX_STEPS_PER_SEC);
  elevation.setAcceleration(500);
  */

  Serial.println("Teensy Stepper Control Ready.");
  Serial.println("Controls: [a] Left, [d] Right, [s] Stop, [f] Free/Toggle Enable");
}

void loop() {
  // Check for Keyboard Input
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();

    switch (incomingChar) {
      case 'a': // Turn Left
        Serial.println("Azimuth: Moving Left");
        azimuth.setSpeed(-MAX_STEPS_PER_SEC);
        break;

      case 'd': // Turn Right
        Serial.println("Azimuth: Moving Right");
        azimuth.setSpeed(MAX_STEPS_PER_SEC);
        break;

      case 's': // Stop
        Serial.println("Azimuth: Stopping");
        azimuth.setSpeed(0);
        break;

      case 'f': // Toggle Free Motor / Enable
        motorsEnabled = !motorsEnabled;
        digitalWrite(AZ_EN_PIN, motorsEnabled ? LOW : HIGH);
        // digitalWrite(EL_EN_PIN, motorsEnabled ? LOW : HIGH);
        Serial.print("Motors: ");
        Serial.println(motorsEnabled ? "ENABLED" : "FREE (DISABLED)");
        break;
    }
  }

  // Constant speed movement requires runSpeed()
  if (motorsEnabled) {
    azimuth.runSpeed();
    // elevation.runSpeed();
  }
}

// #include <AccelStepper.h>

// void setup() {
//   pinMode(6, OUTPUT);
// }

// void loop() {
//   digitalWrite(6, LOW);
// }