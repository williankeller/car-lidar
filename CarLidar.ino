#include <Wire.h>
#include <IBusBM.h>
#include <RPLidar.h>

RPLidar lidar;
IBusBM ibus;

// Motor control and speed pins
const int LeftMotorPin1 = 2;
const int LeftMotorPin2 = 3;
const int RightMotorPin1 = 4;
const int RightMotorPin2 = 5;
const int LeftMotorSpeedPin = 6;
const int RightMotorSpeedPin = 7;

// RPLIDAR motor speed control
const int RPLIDAR_MOTOR = 8;
const float MAX_LIDAR_DISTANCE_CM = 30.0;
const int MAX_MOTOR_SPEED = 255;

// RC channel mappings
enum Channels {
  CH1_LEFT_RIGHT = 0,
  CH2_FORWARD_REVERSE = 1,
  CH3_THROTTLE = 2,
  CH4_RUDDER = 3,
  CH5_AUX1 = 4
};

// Global variables for obstacle detection
bool obstacleDetected = false;
bool turnLeft = false;
bool turnRight = false;
bool turnAround = false;
bool shouldReverse = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  ibus.begin(Serial2);
  lidar.begin(Serial1);

  pinMode(LeftMotorPin1, OUTPUT);
  pinMode(LeftMotorPin2, OUTPUT);
  pinMode(RightMotorPin1, OUTPUT);
  pinMode(RightMotorPin2, OUTPUT);
  pinMode(LeftMotorSpeedPin, OUTPUT);
  pinMode(RightMotorSpeedPin, OUTPUT);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {
  int rcAux1 = readChannel(CH5_AUX1, 0, 255, 0);
  int rcForwardReverse = readChannel(CH2_FORWARD_REVERSE, -255, 255, 0);
  int rcLeftRight = readChannel(CH1_LEFT_RIGHT, -255, 255, 0);

  resetObstacleDetection(); // Reset obstacle detection state
  controlLidar(rcAux1);

  int leftMotorSpeed, rightMotorSpeed;

  calculateMotorSpeeds(rcAux1, rcForwardReverse, rcLeftRight, leftMotorSpeed, rightMotorSpeed);

  if (obstacleDetected) adjustForObstacle(leftMotorSpeed, rightMotorSpeed);

  controlMotors(leftMotorSpeed, rightMotorSpeed);
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void controlLidar(int rcAux1) {
  if (rcAux1 < 10) {
    analogWrite(RPLIDAR_MOTOR, 0);
    return;
  }
  analogWrite(RPLIDAR_MOTOR, MAX_MOTOR_SPEED);
  if (rcAux1 > 10) processLidarData();
}

void processLidarData() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance / 10.0; // Convert to cm
    float angle = lidar.getCurrentPoint().angle;
    byte quality = lidar.getCurrentPoint().quality;

    if (quality > 0 && distance > 0.00 && (angle > 270 || angle < 90) && distance <= MAX_LIDAR_DISTANCE_CM) {
      Serial.println("Distance: " + String(distance) + "cm - Angle: " + String(angle));
      detectObstacleAndAdjust(angle, distance);
    }
  } else {
    attemptLidarRecovery();
  }
}

void attemptLidarRecovery() {
  analogWrite(RPLIDAR_MOTOR, 0);
  Serial.println("Nope :(");

  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    lidar.startScan();
    analogWrite(RPLIDAR_MOTOR, MAX_MOTOR_SPEED);
    delay(1000);
  }
}

void detectObstacleAndAdjust(float angle, float distance) {
  obstacleDetected = true;

  turnLeft = angle >= 0 && angle < 90;
  turnRight = angle > 270 && angle <= 360;

  // If the obstacle is directly in front of the car, turn around
  if (angle > 340 && angle < 20 && distance <= 15) {
    Serial.println("------------------FRONNNTTTTTT");
    turnAround = true;

    // If the object is too close, stop the car and reverse a bit
    if (distance <= 15) {
      Serial.println("------------------Reverse");
      shouldReverse = true;
    }
  }
  // If the object is not directly in front of the car, prioritize turning left or right
  //if (turnLeft || turnRight) turnAround = false;

  // if reverse, prioritize reversing
  if (shouldReverse || turnAround) {
    turnLeft = false;
    turnRight = false;
  }

  if (turnLeft) Serial.println("Turn left");
  if (turnRight) Serial.println("Turn right");
}

void resetObstacleDetection() {
  obstacleDetected = false;
  turnLeft = false;
  turnRight = false;
  shouldReverse = false;
}

void adjustForObstacle(int &leftMotorSpeed, int &rightMotorSpeed) {
  Serial.println("Obstacle detected!");
  if (turnLeft) {
    leftMotorSpeed = -MAX_MOTOR_SPEED;
    rightMotorSpeed = MAX_MOTOR_SPEED;
  } else if (turnRight) {
    leftMotorSpeed = MAX_MOTOR_SPEED;
    rightMotorSpeed = -MAX_MOTOR_SPEED;
  } else if (turnAround) {
    leftMotorSpeed = -MAX_MOTOR_SPEED;
    rightMotorSpeed = -MAX_MOTOR_SPEED;
  } else if (reverse) {
    leftMotorSpeed = -MAX_MOTOR_SPEED;
    rightMotorSpeed = -MAX_MOTOR_SPEED;
  }
}

void calculateMotorSpeeds(int rcAux1, int rcForwardReverse, int rcLeftRight, int &leftMotorSpeed, int &rightMotorSpeed) {
  // Use rcAux1 for forward speed in auto mode
  if (rcAux1 > 100) {
      rcForwardReverse = rcAux1;
  }
  leftMotorSpeed = rcForwardReverse + rcLeftRight;
  rightMotorSpeed = rcForwardReverse - rcLeftRight;
}

void controlMotors(int leftSpeed, int rightSpeed) {
  controlMotor(LeftMotorPin1, LeftMotorPin2, LeftMotorSpeedPin, leftSpeed);
  controlMotor(RightMotorPin1, RightMotorPin2, RightMotorSpeedPin, rightSpeed);
}

void controlMotor(int pin1, int pin2, int speedPin, int speed) {
  digitalWrite(pin1, speed > 0 ? HIGH : LOW);
  digitalWrite(pin2, speed > 0 ? LOW : HIGH);
  analogWrite(speedPin, constrain(abs(speed), 0, 255));
}
