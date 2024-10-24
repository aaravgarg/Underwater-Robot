#include <Stepper.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <math.h>

// Define servo objects
Servo rightServo;
Servo leftServo;
Servo massServo;
Servo rudderServo;
Servo thruster; // Using the Servo class (send PWM signal)

#define STEPS 100 // Change this to the number of steps on your motor
#define STRAIGHT_RUDDER_POS 90

#define TOP_OPEN_PIN 27
#define TOP_CLOSE_PIN 45
#define BOTTOM_OPEN_PIN 44
#define BOTTOM_CLOSE_PIN 26

#define RIGHT_SERVO_PIN 43
#define LEFT_SERVO_PIN 49
#define MASS_SERVO_PIN 42
#define RUDDER_SERVO_PIN 40

#define TEST_POT_PIN A0
#define THRUSTER_PWM_PIN 13


int ADXL345 = 0x53;
float IMU_GScale = 0.0039f;

struct imu_data {
  float x_offset = 0.0f;
  float y_offset = 0.0f;
  float z_offset = 0.0f;

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_deg = 0.0f;
} g_imu_data;

// Create instances of the Stepper class for top and bottom steppers
Stepper syringeStepper(STEPS, 36, 37, 38, 39);

// double rollSetpoint = 0;
// double rollInput;
// double rollOutput;
// PID rollController(&rollInput, &rollOutput, &rollSetpoint, 1, 0, 0, DIRECT);

enum Direction { FORWARD, BACKWARD };
Direction syringeDirection = FORWARD;
bool syringeMoving = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  configIMU();
  calibrateIMU();

  // Set the speed of the motors to 50 RPMs
  syringeStepper.setSpeed(50);
  
  // Set the pins to input pullup mode
  pinMode(TOP_OPEN_PIN, INPUT_PULLUP);
  pinMode(TOP_CLOSE_PIN, INPUT_PULLUP);
  pinMode(BOTTOM_OPEN_PIN, INPUT_PULLUP);
  pinMode(BOTTOM_CLOSE_PIN, INPUT_PULLUP);
  
  // Attach the servo motors to the pins
  rightServo.attach(RIGHT_SERVO_PIN);
  leftServo.attach(LEFT_SERVO_PIN);
  massServo.attach(MASS_SERVO_PIN);
  rudderServo.attach(RUDDER_SERVO_PIN);
  thruster.attach(THRUSTER_PWM_PIN);

  Serial.println("Ready for commands:");
  Serial.println("o - Open wings");
  Serial.println("c - Close wings");
  Serial.println("s - Move syringe steppers");
  Serial.println("m - Move moving mass");
  Serial.println("r - Rotate moving mass");
  Serial.println("i - Calibrate IMU");
  Serial.println("x - move rudder to turn right");
  Serial.println("y - move rudder to turn left");
  Serial.println("z - reset rudder to straight forward");
  
  rightServo.write(25);
  leftServo.write(120);
  rudderServo.write(STRAIGHT_RUDDER_POS);

  arm_esc(); // necessary for BL_HELI_S ESCs
  //rollController.SetMode(AUTOMATIC);
}

void loop() {
  updateIMU();
  //printIMU();
  delay(700);

  int pot_read = analogRead(TEST_POT_PIN);
  Serial.print("pot val: ");
  Serial.println(pot_read);
  set_thruster_speed(pot_read);

  rotateMovingMass(); // live response to roll

  // check if there is no data available in the serial buffer
  if (Serial.available() <= 0) {
    return;
  }
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
    case 'i':
      calibrateIMU();
      break;
    case 'x':
      rudderRight();
      break;
    case 'y':
      rudderLeft();
      break;
    case 'z':
      rudderReset();
      break;
    default:
      Serial.println("Invalid command");
  }

}

// BL_HELI_S init sequence
void arm_esc() {
  Serial.println("Arming the ESC");
  set_thruster_speed(1023);
  delay(5 * 1000);

  Serial.println("Calibrate minimum");
  set_thruster_speed(0);
  delay(5 * 1000);

  Serial.println("Resetting to Zero");
  set_thruster_speed(512);
  delay(2 * 1000);
  
  Serial.println("ESC READY");
}

// Convert input from 0-1023 to 1100-1900 PWM signal and write to the thruster
void set_thruster_speed(int input) {
  int thruster_PWM = map(input, 0, 1023, 1100, 1900);
  
  thruster.writeMicroseconds(thruster_PWM);
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
      if (digitalRead(TOP_CLOSE_PIN) == LOW || digitalRead(BOTTOM_CLOSE_PIN) == LOW) {
        syringeDirection = BACKWARD; // Change direction
        Serial.println("Reached close position, changing direction");
        break; // Exit loop
      }
    } else {
      syringeStepper.step(-1); // Move one step backward
      if (digitalRead(TOP_OPEN_PIN) == LOW || digitalRead(BOTTOM_OPEN_PIN) == LOW) {
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
  static float lastRollAngle = 0.0;
  float currentRollAngle = -g_imu_data.roll_deg + 90;
  currentRollAngle = constrain(currentRollAngle, 0, 180);  // cut the value to between 0 and 180

  // only update the position if the roll is significant
  if (abs(currentRollAngle - lastRollAngle)  < 5.0) {
    return;
  }

  massServo.write(currentRollAngle);
  lastRollAngle = currentRollAngle; 

  // debug output
  // Serial.print("Servo position updated to: ");
  // Serial.println(currentRollAngle);
}

void calibrateIMU() {
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32);
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);

    if (Wire.available() < 6) {
      Serial.println("Error: Failed to read IMU data for calibration.");
      return;
    }

    g_imu_data.x_offset = (Wire.read() | (Wire.read() << 8)) * IMU_GScale;  
    g_imu_data.y_offset = (Wire.read() | (Wire.read() << 8)) * IMU_GScale;
    g_imu_data.z_offset = ((Wire.read() | (Wire.read() << 8)) * IMU_GScale) - 1.0;  // subtract 1g due to gravity on Z

    Serial.println("IMU Calibrated.");
}

void configIMU() {
  // Set data rate to 100 Hz
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2C);  
  Wire.write(0x0A);  
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Failed to set data rate.");
    return;
  }

  // Set measurement mode
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);  
  Wire.write(0x08);  
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Failed to set measurement mode.");
    return;
  }

  // Set range to ±2g, FULL_RES enabled
  Wire.beginTransmission(ADXL345);
  Wire.write(0x31);  
  Wire.write(0x08);  
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Failed to set range.");
    return;
  }

  delay(10);
  Serial.println("ADXL345 Initialized.");
}

void updateIMU() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);  // read from the DATA_X0 register
  Wire.endTransmission(false);
  
  // request 6 bytes (2 per axis)
  Wire.requestFrom(ADXL345, 6, true);
  if (Wire.available() < 6) {
    Serial.println("Error: Could not read from ADXL345");
    return;
  }

  g_imu_data.x = ((Wire.read() | (Wire.read() << 8)) * IMU_GScale) - g_imu_data.x_offset;  // X in g, apply offset
  g_imu_data.y = ((Wire.read() | (Wire.read() << 8)) * IMU_GScale) - g_imu_data.y_offset;  // Y in g, apply offset
  g_imu_data.z = ((Wire.read() | (Wire.read() << 8)) * IMU_GScale) - g_imu_data.z_offset;  // Z in g, apply offset

  // use trigonometry to calculate the pitch and roll in degrees
  g_imu_data.pitch_deg = atan2(g_imu_data.y, sqrt(g_imu_data.x * g_imu_data.x + g_imu_data.z * g_imu_data.z)) * 180.0 / PI;
  g_imu_data.roll_deg = atan2(-g_imu_data.x, sqrt(g_imu_data.y * g_imu_data.y + g_imu_data.z * g_imu_data.z)) * 180.0 / PI;

  // cannot calculate yaw from accel
  g_imu_data.yaw_deg = 0;
}

void printIMU() {
  Serial.print("Roll: ");
  Serial.print(g_imu_data.roll_deg);
  Serial.print("\tPitch: ");
  Serial.print(g_imu_data.pitch_deg);
  Serial.print("\tYaw: ");
  Serial.println(g_imu_data.yaw_deg);
}

void rudderRight(){
  Serial.println("Turning Right");
  int pos = rudderServo.read(); 
  for (pos; pos <= 135; pos += 1) { //Moves Rudder to Full Right Turn 
    rudderServo.write(pos);   
    delay(15);              
  }
}

void rudderLeft() {
  Serial.println("Turning Left");
  int pos = rudderServo.read();
  for (pos; pos >= 42.5; pos -= 1) { //Moves Rudder from Full Right to Full Left Turn
    rudderServo.write(pos);  
    delay(15);               
  }
}

void rudderReset() {
  Serial.println("Going Straight");
  int pos = rudderServo.read();
  if (pos > STRAIGHT_RUDDER_POS) {
    for (pos; pos >=  STRAIGHT_RUDDER_POS; pos -= 1){
      rudderServo.write(pos);
      delay(15);
    }
  }
  else if (pos <  STRAIGHT_RUDDER_POS) {
    for (pos; pos <=  STRAIGHT_RUDDER_POS; pos += 1) {
      rudderServo.write(pos);
      delay(15);
    }
  }
}
