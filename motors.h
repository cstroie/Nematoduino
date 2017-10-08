#include "pin_config.h"

#include <AFMotor.h>
AF_DCMotor motorR(3, MOTOR12_1KHZ); // Set motor #1, 1kHz PWM
AF_DCMotor motorL(4, MOTOR12_1KHZ); // Set motor #2, 1kHz PWM

const uint8_t spdBoost = 150;
const uint8_t turnTime = 100;

void MotorsInit() {
  //pinMode(leftMotorPinA, OUTPUT);
  //pinMode(leftMotorPinB, OUTPUT);
  //pinMode(rightMotorPinA, OUTPUT);
  //pinMode(rightMotorPinB, OUTPUT);
}

void rightMotorForward(uint8_t spd) {
  //analogWrite(rightMotorPinA, spd);
  //digitalWrite(rightMotorPinB, LOW);
  motorR.setSpeed(spd);
  motorR.run(FORWARD);
}

void rightMotorBackward(uint8_t spd) {
  //digitalWrite(rightMotorPinA, LOW);
  //analogWrite(rightMotorPinB, spd);
  motorR.setSpeed(spd);
  motorR.run(BACKWARD);
}

void rightMotorOff() {
  //digitalWrite(rightMotorPinA, LOW);
  //digitalWrite(rightMotorPinB, LOW);
  motorR.run(RELEASE);
}

void leftMotorForward(uint8_t spd) {
  //analogWrite(leftMotorPinA, spd);
  //digitalWrite(leftMotorPinB, LOW);
  motorL.setSpeed(spd);
  motorL.run(FORWARD);
}

void leftMotorBackward(uint8_t spd) {
  //digitalWrite(leftMotorPinA, LOW);
  //analogWrite(leftMotorPinB, spd);
  motorL.setSpeed(spd);
  motorL.run(BACKWARD);
}

void leftMotorOff() {
  //digitalWrite(leftMotorPinA, LOW);
  //digitalWrite(leftMotorPinB, LOW);
  motorL.run(RELEASE);
}

void RunMotors(int16_t leftSpd, int16_t rightSpd) {
  uint8_t leftSpdMotor;
  uint8_t rightSpdMotor;

  // Left forward
  if (leftSpd >= 0) {
    leftSpd += spdBoost;
    if (leftSpd > 255) {
      leftSpdMotor = 255;
    }
    else {
      leftSpdMotor = leftSpd;
    }
    leftMotorForward(leftSpdMotor);
  }
  // Left backward
  else if (leftSpd < 0) {
    leftSpd -= spdBoost;
    if (abs(leftSpd) > 255) {
      leftSpdMotor = 255;
    }
    else {
      leftSpdMotor = abs(leftSpd);
    }
    leftMotorBackward(leftSpdMotor);
  }

  // Right forward
  if (rightSpd >= 0) {
    rightSpd += spdBoost;
    if (rightSpd > 255) {
      rightSpdMotor = 255;
    }
    else {
      rightSpdMotor = rightSpd;
    }
    rightMotorForward(rightSpdMotor);
  }
  // Right backward
  else if (rightSpd < 0) {
    rightSpd -= spdBoost;
    if (abs(rightSpd) > 255) {
      rightSpdMotor = 255;
    }
    else {
      rightSpdMotor = abs(rightSpd);
    }
    rightMotorBackward(rightSpdMotor);
  }

  if ((rightSpd > 0) && (leftSpd < 0)) {
    delay(turnTime);
  }
  else if ((rightSpd < 0) && (leftSpd > 0)) {
    delay(turnTime);
  }
}

void MotorsOff() {
  leftMotorOff();
  rightMotorOff();
}

