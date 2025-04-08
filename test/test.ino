#include "servo_position.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 170
#define SERVOMAX 650

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servoCount = 2; // Number of servos you have
const int servoPins[] = {0, 1};

int defaultPositions[servoCount] = {SERVO_1_POS[99], SERVO_2_POS[99]}; // Default position is when lid is closed
int servoAngles[servoCount]; // Array to store current servo angles

bool isRotating = false; // Flag to track if rotation is in progress
bool isAtZero = true;    // Flag to track servo position (0 or x degrees)

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
  delay(1000); // Wait for servos to reach minimum position
}

void closeLid() {
  isRotating = true;
  for (int i = 0; i < 100; i++) {
    moveServo(0, SERVO_1_POS[i]);
    moveServo(1, SERVO_2_POS[i]);

    Serial.print("servo1:");
    Serial.print(SERVO_1_POS[i]);
    Serial.print(",servo2:");
    Serial.println(SERVO_2_POS[i]);
    // delay(25);
  }
  isRotating = false;
  Serial.println("Closed");
}

void openLid() {
  isRotating = true;
  for (int i = 99; i >= 0; i--) {
    moveServo(0, SERVO_1_POS[i]);
    moveServo(1, SERVO_2_POS[i]);

    Serial.print("servo1:");
    Serial.print(SERVO_1_POS[i]);
    Serial.print(",servo2:");
    Serial.println(SERVO_2_POS[i]);
    // delay(25);
  }
  isRotating = false;
  Serial.println("Opened");
}

void servoMin() {
  isRotating = true;
  for (int i = 0; i < servoCount; i++) {
    moveServo(i, 0);
  }
  isRotating = false;
  Serial.println("Min");
}

void servoMax() {
  isRotating = true;
  for (int i = 0; i < servoCount; i++) {
    moveServo(i, 180);
  }
  isRotating = false;
  Serial.println("Max");
}

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

void moveServo(int servoIndex, int targetAngle)
{
  // Smoothly move the servo to the target angle
  for (int i = servoAngles[servoIndex]; i != targetAngle; i += (targetAngle > i) ? 1 : -1)
  {
    servoAngles[servoIndex] = i;
    pwm.setPWM(servoPins[servoIndex], 0, angleToPulse(i));
    delay(10); // Adjust the delay for smoother motion
  }
}

void setDefaultPositions()
{
  // Set all servos to their default positions
  for (int i = 0; i < servoCount; i++)
  {
    moveServo(i, defaultPositions[i]);
  }
}

int angleToPulse(int angle)
{
  // Convert angle to servo pulse length
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}