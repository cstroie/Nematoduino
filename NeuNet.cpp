/*
  NeuNet.cpp

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#include "Arduino.h"
#include "NeuNet.h"

NeuNet::NeuNet() {
}

void NeuNet::init() {
  /* Neurons that are connected to others */
  neuroState      = (int8_t*) calloc(N_NTOTAL, sizeof(int8_t));
  neuroStateNext  = (int8_t*) calloc(N_NTOTAL, sizeof(int8_t));
  /* Muscles that aren't connected to other cells */
  muscleState     = (int16_t*)calloc(N_NTOTAL - N_MAX, sizeof(int16_t));
  muscleStateNext = (int16_t*)calloc(N_NTOTAL - N_MAX, sizeof(int16_t));
  /* How many cycles a neuron has been idle */
  neuroIdle       = (uint8_t*)calloc(N_MAX, sizeof(uint8_t));
}

void NeuNet::setState(uint16_t id, int8_t value) {
  if (id < N_MAX) neuroState[id] = value;         // Neuron connection
  else            neuroState[id - N_MAX] = value; // Muscle connection

}

int16_t NeuNet::getState(uint16_t id) {
  if (id < N_MAX) return neuroState[id];          // Neuron connection
  else            return muscleState[id - N_MAX]; // Muscle connection

}

void NeuNet::setStateNext(uint16_t id, int8_t value) {
  // Neuron connection
  if (id < N_MAX) {
    if      (value > 127)   neuroStateNext[id] = 127;   // Upper limit
    else if (value < -128)  neuroStateNext[id] = -128;  // Lower limit
    else                    neuroStateNext[id] = value; // Use the specified value
  }
  else
    // Muscle connection
    muscleStateNext[id - N_MAX] = value;
}

int16_t NeuNet::getStateNext(uint16_t id) {
  if (id < N_MAX) return neuroStateNext[id];          // Neuron connection
  else            return muscleStateNext[id - N_MAX]; // Muscle connection
}

void NeuNet::addStateNext(uint16_t id, int8_t value) {
  int16_t val = getStateNext(id);
  setStateNext(id, val + value);
}

void NeuNet::copyState() {
  memcpy(neuroState,  neuroStateNext,  sizeof(neuroStateNext));
  memcpy(muscleState, muscleStateNext, sizeof(muscleStateNext[0]) * (N_NTOTAL - N_MAX));
}

