#include "servo_position.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 170
#define SERVOMAX 650

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servoCount = 2; // Number of servos you have
const int servoPins[] = {0, 1};

double defaultPositions[servoCount] = {SERVO_1_POS[0], SERVO_2_POS[0]}; // Default position is when lid is closed
double servoAngles[servoCount]; // Array to store current servo angles

bool isRotating = false; // Flag to track if rotation is in progress
bool isAtZero = false;    // Flag to track servo position (0 or x degrees)

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  pinMode(13, INPUT_PULLUP); // Pin for button press

  // Initialize servo angles to default positions
  for (int i = 0; i < servoCount; i++)
  {
    servoAngles[i] = defaultPositions[i];
    pwm.setPWM(servoPins[i], 0, angleToPulse(servoAngles[i]));
  }
  // servoMax(); // Set servos to minimum position
  setDefaultPositions();
  delay(1000); // Wait for servos to reach minimum position
}

void closeLid() {
  isRotating = true;
  for (int i = 0; i < 200; i++) {
    moveServo(0, 1, SERVO_1_POS[i], SERVO_2_POS[i]);
    // moveServo(1, SERVO_2_POS[i]);

    // Serial.print("servo1:");
    // Serial.print(SERVO_1_POS[i]);
    // Serial.print(",servo2:");
    // Serial.println(SERVO_2_POS[i]);
    delay(25);
  }
  isRotating = false;
  Serial.println("Closed");
}

void openLid() {
  isRotating = true;
  for (int i = 199; i >= 0; i--) {
    moveServo(0, 1, SERVO_1_POS[i], SERVO_2_POS[i]);
    // moveServo(1, SERVO_2_POS[i]);

    // Serial.print("servo1:");
    // Serial.print(SERVO_1_POS[i]);
    // Serial.print(",servo2:");
    // Serial.println(SERVO_2_POS[i]);
    delay(25);
  }
  isRotating = false;
  Serial.println("Opened");
}

// void servoMin() {
//   isRotating = true;
//   for (int i = 0; i < servoCount; i++) {
//     moveServo(i, 0);
//   }
//   isRotating = false;
//   Serial.println("Min");
// }

// void servoMax() {
//   isRotating = true;
//   for (int i = 0; i < servoCount; i++) {
//     moveServo(i, 180);
//   }
//   isRotating = false;
//   Serial.println("Max");
// }

void loop()
{

  int buttonState = digitalRead(13);
  if (buttonState == LOW && !isRotating) {
    if (isAtZero) {
      openLid();
      // servoMax();
    } else {
      closeLid();
      // servoMin();
    }
    isAtZero = !isAtZero;
  }
}

void moveServo(int servoIndex_1, int servoIndex_2, double targetAngle_1, double targetAngle_2)
{
      pwm.setPWM(servoPins[servoIndex_2], 0, angleToPulse(targetAngle_2));
      pwm.setPWM(servoPins[servoIndex_1], 0, angleToPulse(targetAngle_1));


}

void setDefaultPositions()
{
  // Set all servos to their default positions
  moveServo(0, 1, defaultPositions[0], defaultPositions[1]);
  Serial.println("Home");
}

double angleToPulse(double angle)
{
  // Convert angle to servo pulse length
  Serial.println(int(myMap(angle)));
  return myMap(angle);
}

double myMap(double angle)
{
  return SERVOMIN + (((angle - 0) * (SERVOMAX - SERVOMIN)) / (180));
}