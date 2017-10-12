/*
  Sonar.cpp

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(uint8_t pinTrigger, uint8_t pinEcho, uint8_t maxDistance) : NewPing(pinTrigger, pinEcho, maxDistance) {
}

uint8_t Sonar::echo() {
  distance = (uint8_t)NewPing::ping_cm();
  // Use a larger distance if the sensor is missing
  if (distance == 0) distance = 255;
  return distance;
}
