#include <Wire.h>
#include <IBusBM.h>
#include <RPLidar.h>

RPLidar lidar;
IBusBM ibus;

int LeftMotorPin1 = 2;
int LeftMotorPin2 = 3;
int RightMotorPin1 = 4;
int RightMotorPin2 = 5;

// Speed control pins (PWM capable)
int LeftMotorSpeedPin = 6;
int RightMotorSpeedPin = 7;

// The PWM pin for control the speed of RPLIDAR's motor.
constexpr int RPLIDAR_MOTOR = 8;

constexpr float MAX_lidarDistanceCm_CM = 20.0;
constexpr int MAX_MOTOR_SPEED = 255;

// RC channel mappings
// These 2 channels represents the same stick.
// Poiting left value -100
// Poiting right value 100
// Poiting up value 100
// Poiting down value -100
#define CH1_LEFT_RIGHT 0
#define CH2_FORWARD_REVERSE 1

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void setup() {
  // Start serial monitor
  Serial.begin(115200);
 
  // Attach iBus object to serial port
  //Serial2.begin(9600);
  //ibus.begin(Serial2);

  // Bind the RPLIDAR driver to the Arduino hardware serial
  lidar.begin(Serial1);

  // Set motor control pins as outputs
  pinMode(LeftMotorPin1, OUTPUT);
  pinMode(LeftMotorPin2, OUTPUT);
  pinMode(RightMotorPin1, OUTPUT);
  pinMode(RightMotorPin2, OUTPUT);
  
  // Initialize PWM pins for motor speed control
  pinMode(LeftMotorSpeedPin, OUTPUT);
  pinMode(RightMotorSpeedPin, OUTPUT);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {
  // Read RC channel values
  int rcLeftRight = readChannel(CH1_LEFT_RIGHT, -255, 255, 0);
  int rcForwardReverse = readChannel(CH2_FORWARD_REVERSE, -255, 255, 0);

  // Determine direction and speed for each motor
  int leftMotorSpeed = rcForwardReverse + rcLeftRight;
  int rightMotorSpeed = rcForwardReverse - rcLeftRight;

  // Correct speeds to be within PWM range and set direction
  controlMotor(LeftMotorPin1, LeftMotorPin2, LeftMotorSpeedPin, leftMotorSpeed);
  controlMotor(RightMotorPin1, RightMotorPin2, RightMotorSpeedPin,  rightMotorSpeed);

  if (IS_OK(lidar.waitPoint())) {
    float lidarDistanceCm = lidar.getCurrentPoint().distance / 10.0;
    float lidarAngleDeg = lidar.getCurrentPoint().angle;
    //bool  startBit = lidar.getCurrentPoint().startBit;
    byte  quality  = lidar.getCurrentPoint().quality;

    if (quality == 0 || lidarDistanceCm == 0.00) {
      return;
    }

    if ((lidarAngleDeg > 270 || lidarAngleDeg < 90)) {
      // Check if the object is within the maximum distance
      if (lidarDistanceCm <= MAX_lidarDistanceCm_CM) {
        Serial.println("Distance: " + String(lidarDistanceCm) + "cm - Angle: " + String(lidarAngleDeg));
      }
    }
  } else {
    // Stop the rplidar motor
    analogWrite(RPLIDAR_MOTOR, 0);
    Serial.println("Nope :(");

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, MAX_MOTOR_SPEED);
      delay(1000);
    }
  }
}

void controlMotor(int pin1, int pin2, int speedPin, int speed) {
  if (speed > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    speed = -speed; // Make speed positive for PWM
  }
  analogWrite(speedPin, constrain(speed, 0, 255)); // Constrain speed to valid PWM range
}