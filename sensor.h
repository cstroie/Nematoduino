#ifndef newping_h
#define newping_h

#include "pin_config.h"
#include <NewPing.h>

// NewPing setup of pins and maximum distance.
NewPing sonar(tPin, rPin, 100);


void SensorInit() {
}

long SensorDistance() {
  return sonar.ping_cm();
}

#endif
