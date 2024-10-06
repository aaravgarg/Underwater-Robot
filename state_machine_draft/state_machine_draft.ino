#include <Stepper.h>
#include <Servo.h>
#include "constants.h"

Servo rightServo; // Create a servo object to control the right servo motor
Servo leftServo;  // Create a servo object to control the left servo motor

// create an instance of the topStepper class, specifying the number of steps of the motor and the pins it's attached to
Stepper topStepper(STEPS, 22, 23, 24, 25);

RobotState g_robotState = DISABLED;

Direction topDirection = FORWARD;
bool topMoving = false;

void setup()
{
  Serial.begin(9600);
  // set the speed of the motor to 30 RPMs
  topStepper.setSpeed(50);
  pinMode(topOpenPin, INPUT_PULLUP);
  pinMode(topClosePin, INPUT_PULLUP);
  rightServo.attach(10); // Attach the right servo motor to pin 10
  leftServo.attach(11);  // Attach the left servo motor to pin 11
}

void handleIdleState()
{
  // TODO
}

void handleGlidingState()
{
}

void handleSurfacingState()
{
  // Enable the pump
}

void setDepth()
{
  // TODO
}

void setSpeed()
{
  // TODO
}

void loop()
{
  switch (g_robotState)
  {
  case DISABLED:
    break;
  case IDLE:
    break;
  case GLIDING:
    break;
  case SURFACING:
    break;
  default:
    g_robotState = DISABLED;
    break;
  }
}
