/*
  Sonar.h

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#ifndef SONAR_H
#define SONAR_H

#include "Arduino.h"
#include "NewPing.h"

class Sonar : public NewPing {
  public:
    Sonar(uint8_t pinTrigger, uint8_t pinEcho, uint8_t maxDistance);
    uint8_t echo();
    uint8_t distance;
};

#endif /* SONAR_H */
