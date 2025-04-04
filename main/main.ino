#include "servo_position.h"
#include <Servo.h>

Servo servo_1;
Servo servo_2;

bool isRotating = false; // Flag to track if rotation is in progress
bool isAtZero = true;    // Flag to track servo position (0 or x degrees)
// int servoMax_1 = 180;              // Target angle for servo_1 rotation
// int servoMax_2 = 90;              // Target angle for servo_2 rotation

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT); 
  servo_1.attach(9);
  servo_2.attach(10);
}

float radToDeg(float radians) {
  return radians * (180.0 / PI);
}

void rotateServo(Servo &servo, int startAngle, int endAngle) {
  isRotating = true;
  int step = (startAngle < endAngle) ? 1 : -1;
  for (int angle = startAngle; angle != endAngle + step; angle += step) {
    servo.write(angle);
    delay(15); 
  }
  isRotating = false;
}

void closeLid() {
  for (int i = 0; i < 100; i++) {
    servo_1.write(radToDeg(SERVO_1_POS[i]));
    servo_2.write(radtoDeg(SERVO_2_POS[i]));
    delay(15);
  }
}

void openLid() {
  for (int i = 99; i >= 0; i--) {
    servo_1.write(radToDeg(SERVO_1_POS[i]));
    servo_2.write(radToDeg(SERVO_2_POS[i]));
    delay(15);
  }
}


void loop() {

  int buttonState = digitalRead(2);
  if (buttonState == LOW && !isRotating) {
    if (isAtZero) {
      openLid();
      // rotateServo(servo_1, 0, servoMax_1);
      // rotateServo(servo_2, 0, servoMax_2);
    } else {
      closeLid();
      // rotateServo(servo_1, servoMax_1, 0);
      // rotateServo(servo_2, servoMax_2, 0);
    }
    isAtZero = !isAtZero;
  } else {
    Serial.println("Button released");
    digitalWrite(13, LOW);
  }
}