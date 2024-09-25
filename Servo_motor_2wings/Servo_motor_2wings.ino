#include <Servo.h>
Servo rightServo;  // Create a servo object to control the right servo motor
Servo leftServo;   // Create a servo object to control the left servo motor

void setup() {
  rightServo.attach(29);   // Attach the right servo motor to pin 10
  leftServo.attach(43);    // Attach the left servo motor to pin 11
}

void loop() {
  // Open the wings
  for (int pos = 25, poss = 120; pos <= 95 && poss >= 50; pos += 1, poss -= 1) {
    rightServo.write(pos);   // Move the right servo
    leftServo.write(poss);   // Move the left servo
    delay(15);               // Wait for the servos to reach the position
  }
  delay(1000); // Wait for 1 second

  // Close the wings
  for (int pos = 95, poss = 50; pos >= 25 && poss <= 120; pos -= 1, poss += 1) {
    rightServo.write(pos);   // Move the right servo
    leftServo.write(poss);   // Move the left servo
    delay(15);               // Wait for the servos to reach the position
  }

  delay(1000); // Wait for 1 second
}