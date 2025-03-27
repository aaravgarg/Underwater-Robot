#include <Stepper.h>

// Change this to the number of steps on your motor
#define STEPS 100

const int topOpenPin = 27;
const int topClosePin = 45;
const int bottomOpenPin = 44;
const int bottomClosePin = 26;

Stepper bottomStepper(STEPS, 46, 47, 48, 49);
Stepper topStepper(STEPS, 28, 26, 34, 38);

//Stepper syringeStepper(STEPS, 28, 26, 34, 38);

// Enum for directions
enum Direction { FORWARD, BACKWARD };

// Movement states for both steppers
Direction bottomDirection = FORWARD;
Direction topDirection = FORWARD;
bool bottomMoving = false;
bool topMoving = false;

void setup() {
  Serial.begin(9600);
  // Set the speed of the motors to 50 RPMs
  topStepper.setSpeed(50);
  bottomStepper.setSpeed(50);
  
  pinMode(topOpenPin, INPUT_PULLUP);
  pinMode(topClosePin, INPUT_PULLUP);
  pinMode(bottomOpenPin, INPUT_PULLUP);
  pinMode(bottomClosePin, INPUT_PULLUP);
}

void loop() {
  //moveBottomStepper();
  moveTopStepper();
}
/*
void moveBottomStepper() {
  if (bottomMoving) {
    if (bottomDirection == FORWARD) {
      bottomStepper.step(STEPS);
      if (digitalRead(bottomClosePin) == LOW) {
        Serial.println("Bottom reached close limit.");
        bottomDirection = BACKWARD;
        bottomMoving = false;  // Stop moving until direction change is completed
      }
    } else if (bottomDirection == BACKWARD) {
      bottomStepper.step(-STEPS);
      if (digitalRead(bottomOpenPin) == LOW) {
        Serial.println("Bottom reached open limit.");
        bottomDirection = FORWARD;
        bottomMoving = false;  // Stop moving until direction change is completed
      }
    }
  } else {
    // Start moving in the current direction if not already moving
    bottomMoving = true;
  }
}
*/
void moveTopStepper() {
  if (topMoving) {
    if (topDirection == FORWARD) {
      topStepper.step(STEPS);
      if (digitalRead(topClosePin) == LOW || digitalRead(bottomClosePin) == LOW) {
        topDirection = BACKWARD;
        topMoving = false;  // Stop moving until direction change is completed
      }
    } else if (topDirection == BACKWARD) {
      topStepper.step(-STEPS);
      if (digitalRead(topOpenPin) == LOW || digitalRead(bottomOpenPin) == LOW) {
        topDirection = FORWARD;
        topMoving = false;  // Stop moving until direction change is completed
      }
    }
  } else {
    // Start moving in the current direction if not already moving
    topMoving = true;
  }
}
