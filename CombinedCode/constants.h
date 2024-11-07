#ifndef CONSTANTS_H
#define CONSTANTS_H

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

// ADXL345 registers and configuration: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
// #define ADXL345 0x53
// #define ADXL345_DATA_REGISTER 0x32
// #define ADXL345_BW_RATE 0x2C
// #define ADXL345_POWER_CTL 0x2D
// #define ADXL345_DATA_FORMAT 0x31
// #define ADXL345_DATA_RATE_100HZ 0x0A
// #define ADXL345_MEASURE_MODE 0x08
// #define ADXL345_RANGE_2G 0x08
// #define IMU_G_SCALE 0.0039f

// MPU6050 registers and configuration: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
#define MPU6050 0x68
#define MPU6050_PWR 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 32.8

// lower value will reduce yaw drift at the expense of sensor accuracy
#define COMPLEMENTARY_FILTER_ALPHA 0.8 

enum Mode
{
    DISABLED,
    IDLE,
    GLIDING,
    SURFACING,
    CYCLE
};

enum Direction
{
    FORWARD,
    BACKWARD
};

#endif // CONSTANTS_H