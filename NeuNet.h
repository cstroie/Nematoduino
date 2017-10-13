/*
  NeuNet.h

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#ifndef NEUNET_H
#define NEUNET_H

#include "Arduino.h"
#include "Neurons.h"

// Struct for representing a neuron connection
struct neuroConnection {
  uint16_t id;
  int8_t weight;
};


class NeuNet {
  public:
    NeuNet();
    void init();
    void setState(uint16_t id, int8_t value);
    int16_t getState(uint16_t id);
    void setStateNext(uint16_t id, int8_t value);
    int16_t getStateNext(uint16_t id);
    void addStateNext(uint16_t id, int8_t value);
    void copyState();

  private:
    /* Neurons that are connected to others */
    int8_t * neuroState;
    int8_t * neuroStateNext;
    /* Muscles that aren't connected to other cells */
    int16_t * muscleState;
    int16_t * muscleStateNext;
    /* How many cycles a neuron has been idle */
    uint8_t * neuroIdle;

};




#endif /* NEUNET_H */
