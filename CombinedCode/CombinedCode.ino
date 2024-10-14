#include <Stepper.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <math.h>

Servo rightServo;  // Create a servo object to control the right servo motor
Servo leftServo;   // Create a servo object to control the left servo motor
Servo massServo;
Servo rudderServo;

// Change this to the number of steps on your motor
#define STEPS 100
#define STRAIGHTRUDDERPOSITION 90

const int topOpenPin = 27;
const int topClosePin = 45;
const int bottomOpenPin = 44;
const int bottomClosePin = 26; 

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
  
  pinMode(topOpenPin, INPUT_PULLUP);
  pinMode(topClosePin, INPUT_PULLUP);
  pinMode(bottomOpenPin, INPUT_PULLUP);
  pinMode(bottomClosePin, INPUT_PULLUP);
  
  rightServo.attach(43);   // Attach the right servo motor to pin 43
  leftServo.attach(49);    // Attach the left servo motor to pin 49
  massServo.attach(42);
  rudderServo.attach(40);

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
  rudderServo.write(STRAIGHTRUDDERPOSITION);


  //rollController.SetMode(AUTOMATIC);
}

void loop() {
  updateIMU();
  //printIMU();
  delay(700);

  rotateMovingMass(); // live response to roll

  //rollController.Compute();

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
  if (pos > STRAIGHTRUDDERPOSITION) {
    for (pos; pos >=  STRAIGHTRUDDERPOSITION; pos-=1){
      rudderServo.write(pos);
      delay(15);
    }
  }
  else if (pos <  STRAIGHTRUDDERPOSITION) {
    for (pos; pos <=  STRAIGHTRUDDERPOSITION; pos += 1) {
      rudderServo.write(pos);
      delay(15);
    }
  }
}
