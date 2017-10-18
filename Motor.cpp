/*
  Motor.cpp

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(uint8_t leftMotor, uint8_t rightMotor, uint8_t prescaler = MOTOR34_1KHZ) : motL{leftMotor,  prescaler}, motR{rightMotor, prescaler} {
}

void Motor::run(int16_t leftSpeed, int16_t rightSpeed) {
  uint8_t speed;

  // Left forward
  if (leftSpeed > 0) {
    leftSpeed += speedBoost;
    if (leftSpeed > 255)  speed = 255;
    else                  speed = leftSpeed;
    motL.run(FORWARD);
    motL.setSpeed(speed);
  }
  // Left backward
  else if (leftSpeed < 0) {
    leftSpeed -= speedBoost;
    if (abs(leftSpeed) > 255) speed = 255;
    else                      speed = abs(leftSpeed);
    motL.run(BACKWARD);
    motL.setSpeed(speed);
  }
  // Left stop
  else
    motL.run(RELEASE);

  // Right forward
  if (rightSpeed > 0) {
    rightSpeed += speedBoost;
    if (rightSpeed > 255) speed = 255;
    else                  speed = rightSpeed;
    motR.run(FORWARD);
    motR.setSpeed(speed);
  }
  // Right backward
  else if (rightSpeed < 0) {
    rightSpeed -= speedBoost;
    if (abs(rightSpeed) > 255)  speed = 255;
    else                        speed = abs(rightSpeed);
    motR.run(BACKWARD);
    motR.setSpeed(speed);
  }
  // Right stop
  else
    motR.run(RELEASE);

  // Turning
  if ((turnTime > 0) and
      (((rightSpeed > 0) and (leftSpeed < 0)) or
       ((rightSpeed < 0) and  leftSpeed > 0)))
    delay(turnTime);
}

void Motor::off() {
  motL.run(RELEASE);
  motR.run(RELEASE);
}

