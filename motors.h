#include "pin_config.h"

#include <AFMotor.h>
AF_DCMotor motR(3, MOTOR34_1KHZ); // Set motor #3, 1kHz PWM
AF_DCMotor motL(4, MOTOR34_1KHZ); // Set motor #4, 1kHz PWM

const uint8_t spdBoost = 0;
const uint8_t turnTime = 0;

void MotorsInit() {
}

void rightMotorForward(uint8_t spd) {
  motR.run(FORWARD);
  motR.setSpeed(spd);
}

void rightMotorBackward(uint8_t spd) {
  motR.run(BACKWARD);
  motR.setSpeed(spd);
}

void rightMotorOff() {
  motR.run(RELEASE);
}

void leftMotorForward(uint8_t spd) {
  motL.run(FORWARD);
  motL.setSpeed(spd);
}

void leftMotorBackward(uint8_t spd) {
  motL.run(BACKWARD);
  motL.setSpeed(spd);
}

void leftMotorOff() {
  motL.run(RELEASE);
}

void RunMotors(int16_t leftSpd, int16_t rightSpd) {
  uint8_t leftSpdMotor;
  uint8_t rightSpdMotor;

  // Left forward
  if (leftSpd > 0) {
    leftSpd += spdBoost;
    if (leftSpd > 255)
      leftSpdMotor = 255;
    else
      leftSpdMotor = leftSpd;
    leftMotorForward(leftSpdMotor);
  }
  // Left backward
  else if (leftSpd < 0) {
    leftSpd -= spdBoost;
    if (abs(leftSpd) > 255)
      leftSpdMotor = 255;
    else
      leftSpdMotor = abs(leftSpd);
    leftMotorBackward(leftSpdMotor);
  }
  // Left stop
  else
    leftMotorOff();

  // Right forward
  if (rightSpd > 0) {
    rightSpd += spdBoost;
    if (rightSpd > 255)
      rightSpdMotor = 255;
    else
      rightSpdMotor = rightSpd;
    rightMotorForward(rightSpdMotor);
  }
  // Right backward
  else if (rightSpd < 0) {
    rightSpd -= spdBoost;
    if (abs(rightSpd) > 255)
      rightSpdMotor = 255;
    else
      rightSpdMotor = abs(rightSpd);
    rightMotorBackward(rightSpdMotor);
  }
  // Right stop
  else
    rightMotorOff();

  // Turning
  if (((rightSpd > 0) && (leftSpd < 0)) or
      ((rightSpd < 0) && (leftSpd > 0)))
    delay(turnTime);
}

void MotorsOff() {
  leftMotorOff();
  rightMotorOff();
}

