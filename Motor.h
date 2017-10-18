/*
  Motor.h

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "AFMotor.h"

class Motor {
  public:
    Motor(uint8_t pinLeft, uint8_t pinRight, uint8_t prescaler);
    void run(int16_t leftSpeed, int16_t rightSpeed);
    void off();
    uint8_t speedBoost;
    uint8_t turnTime;
  private:
    AF_DCMotor motL;
    AF_DCMotor motR;
};

#endif /* MOTOR_H */
