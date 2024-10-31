#include <Stepper.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "constants.h"

// Define servo objects
Servo rightServo;
Servo leftServo;
Servo massServo;
Servo rudderServo;
Servo thruster; // Using the Servo class (send PWM signal)

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

  rightServo.write(25);
  leftServo.write(120);
  rudderServo.write(STRAIGHT_RUDDER_POS);

  arm_esc(); // necessary for BL_HELI_S ESCs

  print_menu();
}

void print_menu() {
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
    Wire.write(ADXL345_DATA_REGISTER);
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);

    if (Wire.available() < 6) {
      Serial.println("Error: Failed to read IMU data for calibration.");
      return;
    }

    g_imu_data.x_offset = (Wire.read() | (Wire.read() << 8)) * IMU_G_SCALE;  
    g_imu_data.y_offset = (Wire.read() | (Wire.read() << 8)) * IMU_G_SCALE;
    g_imu_data.z_offset = ((Wire.read() | (Wire.read() << 8)) * IMU_G_SCALE) - 1.0;  // subtract 1g due to gravity on Z

    Serial.println("IMU Calibrated.");
}

void configIMU() {
  // Set data rate to 100 Hz
  Wire.beginTransmission(ADXL345);
  Wire.write(ADXL345_BW_RATE);  
  Wire.write(ADXL345_DATA_RATE_100HZ);  
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Failed to set data rate.");
    return;
  }

  // Set measurement mode
  Wire.beginTransmission(ADXL345);
  Wire.write(ADXL345_POWER_CTL);  
  Wire.write(ADXL345_MEASURE_MODE);  
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Failed to set measurement mode.");
    return;
  }

  // Set range to ±2g, FULL_RES enabled
  Wire.beginTransmission(ADXL345);
  Wire.write(ADXL345_DATA_FORMAT);  
  Wire.write(ADXL345_RANGE_2G);  
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: Failed to set range.");
    return;
  }

  delay(10);
  Serial.println("ADXL345 Initialized.");
}

void updateIMU() {
  Wire.beginTransmission(ADXL345);
  Wire.write(ADXL345_DATA_REGISTER);  // read from the DATA_X0 register
  Wire.endTransmission(false);
  
  // request 6 bytes (2 per axis)
  Wire.requestFrom(ADXL345, 6, true);
  if (Wire.available() < 6) {
    Serial.println("Error: Could not read from ADXL345");
    return;
  }

  g_imu_data.x = ((Wire.read() | (Wire.read() << 8)) * IMU_G_SCALE) - g_imu_data.x_offset;  // X in g, apply offset
  g_imu_data.y = ((Wire.read() | (Wire.read() << 8)) * IMU_G_SCALE) - g_imu_data.y_offset;  // Y in g, apply offset
  g_imu_data.z = ((Wire.read() | (Wire.read() << 8)) * IMU_G_SCALE) - g_imu_data.z_offset;  // Z in g, apply offset

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

// from -45 to 45 degrees
void setRudder(int angle) {
  // Clamp angle to the range -45 to 45
  if (angle < -45) {
    angle = -45;
  } else if (angle > 45) {
    angle = 45;
  }
  
  int pos = map(angle, -45, 45, 45, 135);
  rudderServo.write(pos);
}

void setPitch(int angle) {
  // PID controller return output

  // map the output to
  //int pos = neutral position;
  if (angle > 0) {
    // map the output to a negative position
  }

  if (angle < 0) {
    // map the output to a positive position
  }
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
