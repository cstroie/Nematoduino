/*
  NeuNet.cpp

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.
*/

#include "Arduino.h"
#include "NeuNet.h"
#include "Motor.h"

NeuNet::NeuNet(Motor* motors) {
  _motors = motors;
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
  /* Averages */
  avgMotoNeuron = 0;
  avgMotoRight = 0;
  avgMotoLeft = 0;
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

/* Parse a word of the ROM into a neuron id and connection weight */
neuroConnection NeuNet::parseROM(uint16_t romWord) {
  uint8_t* romByte;
  romByte = (uint8_t*)&romWord;
  /* The id requires 9 bits */
  uint16_t id = romByte[1] + ((romByte[0] & 0b10000000) << 1);
  /* The weight can be negative */
  uint8_t weightBits = romByte[0] & 0b01111111;
  weightBits = weightBits + ((weightBits & 0b01000000) << 1);
  int8_t weight = (int8_t)weightBits;
  /* Return the struct */
  neuroConnection neuralConn = {id, weight};
  return neuralConn;
}

/* Propagate each neuron connection weight into the next state */
void NeuNet::ping(uint16_t id) {
  uint16_t address = pgm_read_word_near(nrConnectome + id + 1);
  uint16_t len = pgm_read_word_near(nrConnectome + id + 1 + 1) - pgm_read_word_near(nrConnectome + id + 1);
  for (int i = 0; i < len; i++) {
    neuroConnection neuralConn = parseROM(pgm_read_word_near(nrConnectome + address + i));
    addStateNext(neuralConn.id, neuralConn.weight);
  }
}

void NeuNet::discharge(uint16_t id) {
  ping(id);
  setStateNext(id, 0);
}

// Complete one cycle ('tick') of the nematode neural system
void NeuNet::cycle() {
  for (int i = 0; i < N_MAX; i++)
    if (getState(i) > N_THRESHOLD) discharge(i);
  muscles();
  idle();
  copyState();
}

/* Flush neurons that have been idle for a while */
void NeuNet::idle() {
  for (int i = 0; i < N_MAX; i++) {
    if (getStateNext(i) == getState(i)) {
      neuroIdle[i] += 1;
    }
    if (neuroIdle[i] > 10) {
      setStateNext(i, 0);
      neuroIdle[i] = 0;
    }
  }
}

/* Determine how muscle weights map to motors */
void NeuNet::muscles() {
  int32_t bodyTotal = 0;

  /* Gather totals on left and right side muscles */
  for (int i = 0; i < N_NBODYMUSCLES; i++) {
    // Get the motoneuron
    uint16_t leftId  = pgm_read_word_near(LeftBodyMuscles  + i);
    uint16_t rightId = pgm_read_word_near(RightBodyMuscles + i);
    // Get the value
    int16_t leftVal  = getStateNext(leftId);
    int16_t rightVal = getStateNext(rightId);
    // Only positive states
    if (leftVal < 0)  leftVal  = 0;
    if (rightVal < 0) rightVal = 0;
    // Get a grand total
    bodyTotal += (leftVal + rightVal);
    // Reset the motoneuron
    setStateNext(leftId,  0);
    setStateNext(rightId, 0);
  }

  // Gather total for neck muscles
  int32_t leftNeckTotal  = 0;
  int32_t rightNeckTotal = 0;
  for (int i = 0; i < N_NNECKMUSCLES; i++) {
    // Get the motoneuron
    uint16_t leftId  = pgm_read_word_near(LeftNeckMuscles  + i);
    uint16_t rightId = pgm_read_word_near(RightNeckMuscles + i);
    // Get the value
    int16_t leftVal  = getStateNext(leftId);
    int16_t rightVal = getStateNext(rightId);
    // Only positive states
    if (leftVal < 0)  leftVal  = 0;
    if (rightVal < 0) rightVal = 0;
    // Get grand totals
    leftNeckTotal  += leftVal;
    rightNeckTotal += rightVal;
    // Reset the motoneuron
    setStateNext(leftId,  0);
    setStateNext(rightId, 0);
  }

#ifdef DEVEL
  /*
    Serial.print(leftNeckTotal);
    Serial.print(",");
    Serial.print(rightNeckTotal);
    Serial.print(",");
  */
  //Serial.println(bodyTotal);
#endif

  //int16_t normBodyTotal = 255.0 * ((float) bodyTotal) / 600.0;

  // Log A and B type motor neuron activity
  int16_t motorNeuronASum = 0;
  int16_t motorNeuronBSum = 0;

  for (int i = 0; i < N_SIGMOTORB; i++) {
    uint8_t motorBId = pgm_read_word_near(SigMotorNeuronsB + i);
    if (getState(motorBId) > N_THRESHOLD)
      motorNeuronBSum += 1;
  }

  for (int i = 0; i < N_SIGMOTORA; i++) {
    uint8_t motorAId = pgm_read_word_near(SigMotorNeuronsA + i);
    if (getState(motorAId) > N_THRESHOLD)
      motorNeuronASum += 1;
  }

  // Sum (with weights) and add contribution to running average of significant activity
  int16_t motorNeuronSumTotal = motorNeuronBSum - motorNeuronASum;

  avgMotoNeuron = (avgMotoNeuron + 100 * motorNeuronSumTotal) / 2;

  // Set left and right totals, scale neck muscle contribution
  //int32_t leftTotal  = (4 * leftNeckTotal)  + normBodyTotal;
  //int32_t rightTotal = (4 * rightNeckTotal) + normBodyTotal;
  //int16_t leftTotal  = (10 * leftNeckTotal)  + bodyTotal;
  //int16_t rightTotal = (10 * rightNeckTotal) + bodyTotal;

  avgMotoRight  = (12 * avgMotoRight + (20 * rightNeckTotal) + bodyTotal) / 15;
  avgMotoLeft  =  (12 * avgMotoLeft +  (20 * leftNeckTotal)  + bodyTotal) / 15;

#ifdef DEVEL

  //Serial.print(motorNeuronBSum);
  //Serial.print(",");
  //Serial.print(motorNeuronASum);
  //Serial.print(",");
  Serial.print(LeftMotorAvg);
  Serial.print(",");
  Serial.print(RightMotorAvg);
  Serial.print(",");
  //Serial.print(bodyTotal);
  //Serial.print(",");
  Serial.println(MotorNeuronAvg);

#endif

  // Magic number read off from c_matoduino simulation
  if (avgMotoNeuron < 60) _motors->run(-avgMotoRight, -avgMotoLeft); //RunMotors(-rightTotal, -leftTotal);
  else                    _motors->run( avgMotoRight,  avgMotoLeft); //RunMotors( rightTotal,  leftTotal);
}

