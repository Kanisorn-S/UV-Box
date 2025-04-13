#include "servo_position.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 170
#define SERVOMAX 650

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servoCount = 2; // Number of servos you have
const int servoPins[] = {3, 1};

double defaultPositions[servoCount] = {SERVO_1_POS[0], SERVO_2_POS[0]}; // Default position is when lid is open
double servoAngles[servoCount]; // Array to store current servo angles
int servoPWM[servoCount]; // Array to store current servo PWM values

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
    servoPWM[i] = int(myAngleToPulse(defaultPositions[i], false, i));
    pwm.setPWM(servoPins[i], 0, myAngleToPulse(servoAngles[i], false, i));
  }
  // servoMax(); // Set servos to minimum position
  setDefaultPositions();
  delay(1000); // Wait for servos to reach minimum position
}

void closeLid() {
  isRotating = true;
  for (int i = 0; i < 100; i++) {
    // myMoveServo(0, 1, SERVO_1_POS[i], SERVO_2_POS[i], i);
    PWMMoveServo(0, 1, int(myAngleToPulse(SERVO_1_POS[i], false, i)), int(myAngleToPulse(SERVO_2_POS[i], false, i)), i);
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
  for (int i = 99; i >= 0; i--) {
    // myMoveServo(0, 1, SERVO_1_POS[i], SERVO_2_POS[i], i);
    PWMMoveServo(0, 1, int(myAngleToPulse(SERVO_1_POS[i], false, i)), int(myAngleToPulse(SERVO_2_POS[i], false, i)), i);
    // moveServo(1, SERVO_2_POS[i]);

    // Serial.print("servo1:");
    // Serial.print(SERVO_1_POS[i]);
    // Serial.print(",servo2:");
    // Serial.println(SERVO_2_POS[i]);
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

void myMoveServo(int servoIndex_1, int servoIndex_2, double targetAngle_1, double targetAngle_2, int i)
{
      pwm.setPWM(servoPins[servoIndex_2], 0, myAngleToPulse(targetAngle_2, false, i));
      pwm.setPWM(servoPins[servoIndex_1], 0, myAngleToPulse(targetAngle_1, true, i));
      delay(15);
}

void PWMMoveServo(int servoIndex_1, int servoIndex_2, int targetPWM_1, int targetPWM_2, int i)
{
  // Calculate the total steps for each servo
  int steps_1 = abs(targetPWM_1 - servoPWM[servoIndex_1]);
  int steps_2 = abs(targetPWM_2 - servoPWM[servoIndex_2]);

  // Function to calculate the greatest common divisor (GCD)
  auto gcd = [](int a, int b) {
    while (b != 0) {
      int temp = b;
      b = a % b;
      a = temp;
    }
    return a;
  };

  // Calculate the least common multiple (LCM)
  int maxSteps = (steps_1 * steps_2) / gcd(steps_1, steps_2);

  // Calculate the step interval for each servo
  int interval_1 = maxSteps / steps_1;
  int interval_2 = maxSteps / steps_2;

  // Smoothly move both servos to their target positions
  for (int step = 0; step <= maxSteps; step++) {
    if (step % interval_1 == 0 && steps_1 > 0) {
      servoPWM[servoIndex_1] += (targetPWM_1 > servoPWM[servoIndex_1]) ? 1 : -1;
      pwm.setPWM(servoPins[servoIndex_1], 0, servoPWM[servoIndex_1]);
    }
    if (step % interval_2 == 0 && steps_2 > 0) {
      servoPWM[servoIndex_2] += (targetPWM_2 > servoPWM[servoIndex_2]) ? 1 : -1;
      pwm.setPWM(servoPins[servoIndex_2], 0, servoPWM[servoIndex_2]);
    }
    delay(10); // Adjust the delay for smoother motion
  }

  // Ensure the final positions are set precisely
  servoPWM[servoIndex_1] = targetPWM_1;
  servoPWM[servoIndex_2] = targetPWM_2;

  pwm.setPWM(servoPins[servoIndex_1], 0, targetPWM_1);
  pwm.setPWM(servoPins[servoIndex_2], 0, targetPWM_2);
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
  myMoveServo(0, 1, defaultPositions[0], defaultPositions[1], 0);
  Serial.println("Home");
}

double myAngleToPulse(double angle, bool debug, int i)
{
  // Convert angle to servo pulse length
  if (debug) 
  {
    Serial.println(i + int(myMap(angle)));
  }
  return myMap(angle);
}

double myMap(double angle)
{
  return SERVOMIN + (((angle - 0) * (SERVOMAX - SERVOMIN)) / (180));
}

int angleToPulse(int angle)
{
  // Convert angle to servo pulse length
  Serial.println(map(angle, 0, 180, SERVOMIN, SERVOMAX));
  return map(angle, 0, 18, SERVOMIN, SERVOMAX);
}