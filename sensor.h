#ifndef newping_h
#define newping_h

#include "pin_config.h"
#include <NewPing.h>

// NewPing setup of pins and maximum distance.
NewPing sonar(tPin, rPin, 100);


void SensorInit() {
}

int SensorDistance() {
  int dist = (int)sonar.ping_cm();
  // Use a larger distance if the sensor is missing
  if (dist == 0) dist = 1000;
  return dist;
}

#endif
