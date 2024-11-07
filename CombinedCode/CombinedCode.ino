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

  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
} g_imu_data;

struct robot_state {
  float pitch_target = 0.0;
  float yaw_target = 0.0;
  float roll_target = 0.0;
  Mode mode = DISABLED;
} g_robotState;

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

  //arm_esc(); // necessary for BL_HELI_S ESCs

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
  //updateIMU();
  //printIMU();
  delay(20); //!! KEEP THIS LOW AS POSSIBLE FOR BEST YAW MEASURMENT

  //thrusterTest();

  // check if there is no data available in the serial buffer
  if (Serial.available() > 0) {
    char command = Serial.read();
    changeRobotState(command);
  }

  switch (g_robotState.mode)
  {
  case DISABLED:
    // do nothing
    Serial.println("disabled");
    break;
  case IDLE:
    // handle idle
    Serial.println("idle");
    break;
  case GLIDING:
    handle_gliding();
    break;
  case SURFACING:
    handle_surfacing();
    break;
  default:
    g_robotState.mode = DISABLED;
    break;
  }

  //handleCommandDebug(command);
}

void changeRobotState(char input) {
  switch (input) {
    case 'd':
      g_robotState.mode = DISABLED;
      break;
    case 'i':
      g_robotState.mode = IDLE;
      break;
    case 'g':
      g_robotState.mode = GLIDING;
      break;
    case 's':
      g_robotState.mode = SURFACING;
      break;
    default:
      Serial.println("Invalid command");
      g_robotState.mode = DISABLED;
      break;
  }
}

void handle_gliding() {
  Serial.println("gliding");
  rotateMovingMass(); // live response to roll

}

void handle_surfacing() {
  Serial.println("surfacing");
  rotateMovingMass(); // live response to roll
}

void thrusterTest() {
  int pot_read = analogRead(TEST_POT_PIN);
  //Serial.print("pot val: ");
  //Serial.println(pot_read);
  set_thruster_speed(pot_read);
}



void handleCommandDebug(char command) {
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
  Wire.beginTransmission(MPU6050);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);

  if (Wire.available() < 6) {
    Serial.println("Error: Failed to read IMU data for calibration.");
    return;
  }

  g_imu_data.x_offset = (Wire.read() << 8 | Wire.read()) / 16384.0;
  g_imu_data.y_offset = (Wire.read() << 8 | Wire.read()) / 16384.0;
  g_imu_data.z_offset = (Wire.read() << 8 | Wire.read()) / 16384.0 - 1.0;  // subtract 1g due to gravity on Z

  Serial.println("IMU Calibrated.");
}

void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) {
    Serial.print("Error: Failed to write to register ");
    Serial.println(reg, HEX);
  }
}

void configIMU() {
  writeRegister(MPU6050, MPU6050_PWR, 0);             // Wake up the MPU6050
  writeRegister(MPU6050, MPU6050_SMPLRT_DIV, 7);      // Set sample rate to 1 kHz / (1 + 7) = 125 Hz
  writeRegister(MPU6050, MPU6050_CONFIG, 0);          // Set DLPF to 260 Hz
  writeRegister(MPU6050, MPU6050_GYRO_CONFIG, 0x10);  // Set gyro range to ±1000 deg/s
  writeRegister(MPU6050, MPU6050_ACCEL_CONFIG, 0);    // Set accelerometer range to ±2g

  delay(10);
  Serial.println("MPU6050 Initialized.");
}

void updateIMU() {
  static unsigned long lastUpdateTime = 0;
  static float yaw = 0.0f;

  Wire.beginTransmission(MPU6050);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  
  // request 14 bytes (6 accel, 2 temp, 6 gyro)
  Wire.requestFrom(MPU6050, 14, true);
  if (Wire.available() < 14) {
    Serial.println("Error: Could not read from MPU6050");
    return;
  }

  g_imu_data.x = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - g_imu_data.x_offset;  // X in g, apply offset
  g_imu_data.y = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - g_imu_data.y_offset;  // Y in g, apply offset
  g_imu_data.z = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - g_imu_data.z_offset;  // Z in g, apply offset

  // skip temperature data
  Wire.read();
  Wire.read();

  g_imu_data.gyro_x = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;  // X in deg/s 
  g_imu_data.gyro_y = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;  // Y in deg/s
  g_imu_data.gyro_z = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;  // Z in deg/s

  // use trigonometry to calculate the pitch and roll in degrees
  g_imu_data.pitch_deg = atan2(g_imu_data.y, sqrt(g_imu_data.x * g_imu_data.x + g_imu_data.z * g_imu_data.z)) * 180.0 / PI;
  g_imu_data.roll_deg = atan2(-g_imu_data.x, sqrt(g_imu_data.y * g_imu_data.y + g_imu_data.z * g_imu_data.z)) * 180.0 / PI;

  // integrate gyro data to get yaw
  unsigned long currentTime = millis(); 
  float dt = (currentTime - lastUpdateTime) / 1000.0;
  yaw += g_imu_data.gyro_z * dt;

  // complementary filter to reduce yaw noise
  yaw = COMPLEMENTARY_FILTER_ALPHA * yaw + (1 - COMPLEMENTARY_FILTER_ALPHA) * g_imu_data.yaw_deg;
  g_imu_data.yaw_deg = yaw;

  lastUpdateTime = currentTime;
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