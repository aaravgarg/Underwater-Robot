#include <Stepper.h>
#include <Servo.h>
Servo rightServo;  // Create a servo object to control the right servo motor
Servo leftServo;   // Create a servo object to control the left servo motor

#define STEPS 100
const int topOpenPin = 27;
const int topClosePin = 45;
const int bottomOpenPin = 44;
const int bottomClosePin = 26;


bool topOpen = false;
bool topClose = false;
bool bottomOpen = false;
bool bottomClose = false;

// create an instance of the topStepper class, specifying the number of steps of the motor and the pins it's attached to
Stepper topStepper(STEPS, 22, 23, 24, 25);

enum Direction { FORWARD, BACKWARD };
Direction topDirection = FORWARD;
bool topMoving = false;

void setup() {
  Serial.begin(9600);
  // set the speed of the motor to 30 RPMs
  topStepper.setSpeed(50);
  pinMode(topOpenPin, INPUT_PULLUP);
  pinMode(topClosePin, INPUT_PULLUP);
  rightServo.attach(10);   // Attach the right servo motor to pin 10
  leftServo.attach(11);    // Attach the left servo motor to pin 11
}


void loop() {
  if (Serial.available() > 0) { // Check if data is available
    char command = Serial.read(); // Read the incoming character
    // Call functions based on the input
    if (command == 'O') {
      openWing();  
    } else if (command == 'C') {
      closeWing(); 
    } else if (command == 'S'){
      moveTopStepper();
    }
  }
}

void openWing(){
  // Open the wings
  for (int pos = 25, poss = 120; pos <= 95 && poss >= 50; pos += 1, poss -= 1) {
    rightServo.write(pos);   // Move the right servo
    leftServo.write(poss);   // Move the left servo
    delay(15);               // Wait for the servos to reach the position
  }
}

void closeWing(){
  // Close the wings
  for (int pos = 95, poss = 50; pos >= 25 && poss <= 120; pos -= 1, poss += 1) {
    rightServo.write(pos);   // Move the right servo
    leftServo.write(poss);   // Move the left servo
    delay(15);               // Wait for the servos to reach the position
  }
}

void moveTopStepper(){
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




