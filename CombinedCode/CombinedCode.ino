#include <Stepper.h>
#include <Servo.h>

Servo rightServo;  // Create a servo object to control the right servo motor
Servo leftServo;   // Create a servo object to control the left servo motor
Servo massServo;

// Change this to the number of steps on your motor
#define STEPS 100

const int topOpenPin = 27;
const int topClosePin = 45;
const int bottomOpenPin = 44;
const int bottomClosePin = 26; 

// Create instances of the Stepper class for top and bottom steppers
Stepper syringeStepper(STEPS, 36, 37, 38, 39);

enum Direction { FORWARD, BACKWARD };
Direction syringeDirection = FORWARD;
bool syringeMoving = false;

void setup() {
  Serial.begin(9600);
  
  // Set the speed of the motors to 50 RPMs
  syringeStepper.setSpeed(50);
  
  pinMode(topOpenPin, INPUT_PULLUP);
  pinMode(topClosePin, INPUT_PULLUP);
  pinMode(bottomOpenPin, INPUT_PULLUP);
  pinMode(bottomClosePin, INPUT_PULLUP);
  
  rightServo.attach(43);   // Attach the right servo motor to pin 43
  leftServo.attach(49);    // Attach the left servo motor to pin 49
  massServo.attach(42);

  Serial.println("Ready for commands:");
  Serial.println("o - Open wings");
  Serial.println("c - Close wings");
  Serial.println("s - Move syringe steppers");
  Serial.println("m - Move moving mass");
  Serial.println("r - Rotate moving mass");
  
  rightServo.write(25);
  leftServo.write(120);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'o':
        openWing();
        break;
      case 'c':
        closeWing();
        break;
      case 's':
        moveSyringeSteppers();
        break;
      /*
      case 'm':
        moveMovingMass();
        break;
        */
      case 'r':
        rotateMovingMass();
        break;
      default:
        Serial.println("Invalid command");
    }
  }
}

void openWing() {
  Serial.println("Opening wings");
  for (int pos = 25, poss = 120; pos <= 95 && poss >= 50; pos += 1, poss -= 1) {
    rightServo.write(pos);
    leftServo.write(poss);
    delay(15);
  }
  Serial.println("Wings opened");
}

void closeWing() {
  Serial.println("Closing wings");
  for (int pos = 95, poss = 50; pos >= 25 && poss <= 120; pos -= 1, poss += 1) {
    rightServo.write(pos);
    leftServo.write(poss);
    delay(15);
  }
  Serial.println("Wings closed");
}

void moveSyringeSteppers() {
  Serial.println("Moving syringe steppers");
  
  while (true) { // Continuous loop
    if (syringeDirection == FORWARD) {
      syringeStepper.step(1); // Move one step forward
      if (digitalRead(topClosePin) == LOW || digitalRead(bottomClosePin) == LOW) {
        syringeDirection = BACKWARD; // Change direction
        Serial.println("Reached close position, changing direction");
        break; // Exit loop
      }
    } else {
      syringeStepper.step(-1); // Move one step backward
      if (digitalRead(topOpenPin) == LOW || digitalRead(bottomOpenPin) == LOW) {
        syringeDirection = FORWARD; // Change direction
        Serial.println("Reached open position, changing direction");
        break; // Exit loop
      }
    }
    delay(10); // Small delay to control speed and avoid overwhelming the serial output
  }
  
}

// Uncomment and implement these functions if needed in future testing
/*
void moveMovingMass() {
  Serial.println("Moving the moving mass");
}
*/
void rotateMovingMass() {
  for (int pos = 95; pos >= 25; pos -= 1) {
    massServo.write(pos);
    delay(15);
  }
}
