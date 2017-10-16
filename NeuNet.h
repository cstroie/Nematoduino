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
#include "Motor.h"

// Struct for representing a neuron connection
struct neuroConnection {
  uint16_t id;
  int8_t weight;
};


class NeuNet {
  public:
    NeuNet(Motor* m);
    void    init();
    void    setState(uint16_t id, int8_t value);
    int16_t getState(uint16_t id);
    void    setStateNext(uint16_t id, int8_t value);
    int16_t getStateNext(uint16_t id);
    void    addStateNext(uint16_t id, int8_t value);
    void    copyState();
    neuroConnection parseROM(uint16_t romWord);
    void    ping(uint16_t id);
    void    discharge(uint16_t id);
    void    cycle();
    void    idle();
    void    muscles();
  private:
    /* Neurons that are connected to others */
    int8_t * neuroState;
    int8_t * neuroStateNext;
    /* Muscles that aren't connected to other cells */
    int16_t * muscleState;
    int16_t * muscleStateNext;
    /* How many cycles a neuron has been idle */
    uint8_t * neuroIdle;
    /* Running average of activity for 'significant' motor neurons */
    int16_t avgMotoNeuron;
    int16_t avgMotoRight;
    int16_t avgMotoLeft;
    /* Motors */
    Motor * _motors;
};

#endif /* NEUNET_H */
