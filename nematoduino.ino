/**
  Nematoduino - Arduino UNO-compatible robotic simulation of the C. elegans nematode

  Copyright 2017 Nathan Griffith <nategri@gmail.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Nematoduino.

  Nematoduino is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  Nematoduino is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  Nematoduino.  If not, see <http://www.gnu.org/licenses/>.

  Nematoduino is an Arduino UNO-compatible robotic simulation of the C. elegans nematode.
  At the core of the simulation is a spiking neural network incorporating 300 neuron cells
  of the biological worm's connectome, along with associated muscle cells.
*/


// User settings
#include "UserSettings.h"


#include <avr/pgmspace.h>

#include "motors.h"
#include "sensor.h"

#include "neuro.h"

//
// Global constants
//

// Total number of connected neurons (first word in ROM)
uint16_t const N_MAX = (uint16_t)NeuralROM[0];

// Running average of activity for 'significant' motor neurons
int16_t MotorNeuronAvg = 0;
int16_t RightMotorAvg = 0;
int16_t LeftMotorAvg = 0;

//
// Structs
//

// Struct for representing a neuron connection
struct NeuralConnection {
  uint16_t id;
  int8_t weight;
};

//
// Three sets of neural state arrays
//

// One set for neurons that are connected to others
int8_t CurrConnectedState[N_NTOTAL];
int8_t NextConnectedState[N_NTOTAL];

// Another set for muscles that aren't connected to other cells
int16_t* CurrMuscleState = malloc((N_NTOTAL - N_MAX) * sizeof(int16_t));
int16_t* NextMuscleState = malloc((N_NTOTAL - N_MAX) * sizeof(int16_t));

// Final set to track how many cycles a neuron has been idle
uint8_t* IdleCycles = malloc(N_MAX*sizeof(uint8_t));

// Neuro cycle time
const unsigned long neuroDelay    = 50UL; // Delay between neuro cycles
unsigned long       neuroNextTime = 0UL;  // Time of next neuro cycle

// Sensors
int16_t       snsFront;                   // Front sensor distance
const int16_t snsFrontThreshold = 40;     // Threshold for front sensor activation (cm)

//
// Functions for getting and setting these states
//

void StatesInit() {
  memset(CurrConnectedState, 0, sizeof(CurrConnectedState));
  memset(NextConnectedState, 0, sizeof(NextConnectedState));
  memset(CurrMuscleState, 0, (N_NTOTAL - N_MAX)*sizeof(CurrMuscleState[0]));
  memset(NextMuscleState, 0, (N_NTOTAL - N_MAX)*sizeof(NextMuscleState[0]));

  memset(IdleCycles, 0, N_MAX * sizeof(NextConnectedState[0]));
}

void SetCurrState(uint16_t N_ID, int8_t val) {
  if (N_ID < N_MAX) CurrConnectedState[N_ID] = val;       // Neuron connection
  else              CurrMuscleState[N_ID - N_MAX] = val;  // Muscle connection
}

int16_t GetCurrState(uint16_t N_ID) {
  if (N_ID < N_MAX) return CurrConnectedState[N_ID];      // Neuron connection
  else              return CurrMuscleState[N_ID - N_MAX]; // Muscle connection
}

void SetNextState(uint16_t N_ID, int16_t val) {
  // Neuron connection
  if (N_ID < N_MAX) {
    if (val > 127)        NextConnectedState[N_ID] = 127;   // Upper limit
    else if (val < -128)  NextConnectedState[N_ID] = -128;  // Lower limit
    else                  NextConnectedState[N_ID] = val;   // Use the specified value
  }
  else
    // Muscle connection
    NextMuscleState[N_ID - N_MAX] = val;
}

int16_t GetNextState(uint16_t N_ID) {
  if (N_ID < N_MAX) return NextConnectedState[N_ID];        // Neuron connection
  else              return NextMuscleState[N_ID - N_MAX];   // Muscle connection
}

void AddToNextState(uint16_t N_ID, int8_t val) {
  int16_t currVal = GetNextState(N_ID);
  SetNextState(N_ID, currVal + val);
}

// Copy 'next' state into 'current' state
void CopyStates() {
  memcpy(CurrConnectedState, NextConnectedState, sizeof(NextConnectedState));
  memcpy(CurrMuscleState,    NextMuscleState,    sizeof(NextMuscleState[0]) * (N_NTOTAL - N_MAX));
}

//
// Functions for handling connectome simulation
//

// Parse a word of the ROM into a neuron id and connection weight
NeuralConnection ParseROM(uint16_t romWord) {
  uint8_t* romByte;
  romByte = (uint8_t*)&romWord;

  // The id requires 9 bits
  uint16_t id = romByte[1] + ((romByte[0] & 0b10000000) << 1);

  // The weight can be negative
  uint8_t weightBits = romByte[0] & 0b01111111;
  weightBits = weightBits + ((weightBits & 0b01000000) << 1);
  int8_t weight = (int8_t)weightBits;

  // Return one struct
  NeuralConnection neuralConn = {id, weight};
  return neuralConn;
}

// Propagate each neuron connection weight into the next state
void PingNeuron(uint16_t N_ID) {
  uint16_t address = pgm_read_word_near(NeuralROM + N_ID + 1);
  uint16_t len = pgm_read_word_near(NeuralROM + N_ID + 1 + 1) - pgm_read_word_near(NeuralROM + N_ID + 1);
  for (int i = 0; i < len; i++) {
    NeuralConnection neuralConn = ParseROM(pgm_read_word_near(NeuralROM + address + i));
    AddToNextState(neuralConn.id, neuralConn.weight);
  }
}

void DischargeNeuron(uint16_t N_ID) {
  PingNeuron(N_ID);
  SetNextState(N_ID, 0);
}

// Complete one cycle ('tick') of the nematode neural system
void NeuralCycle() {
  for (int i = 0; i < N_MAX; i++)
    if (GetCurrState(i) > N_THRESHOLD) DischargeNeuron(i);
  ActivateMuscles();
  HandleIdleNeurons();
  CopyStates();
}

// Flush neurons that have been idle for a while
void HandleIdleNeurons() {
  for (int i = 0; i < N_MAX; i++) {
    if (GetNextState(i) == GetCurrState(i)) {
      IdleCycles[i] += 1;
    }
    if (IdleCycles[i] > 10) {
      SetNextState(i, 0);
      IdleCycles[i] = 0;
    }
  }
}

//
// Function for determinining how muscle weights map to motors
//

void ActivateMuscles() {
  int32_t bodyTotal = 0;

  // Gather totals on left and right side muscles
  for (int i = 0; i < N_NBODYMUSCLES; i++) {
    // Get the motoneuron
    uint16_t leftId  = pgm_read_word_near(LeftBodyMuscles  + i);
    uint16_t rightId = pgm_read_word_near(RightBodyMuscles + i);
    // Get the value
    int16_t leftVal  = GetNextState(leftId);
    int16_t rightVal = GetNextState(rightId);
    // Only positive states
    if (leftVal < 0)  leftVal  = 0;
    if (rightVal < 0) rightVal = 0;
    // Get a grand total
    bodyTotal += (leftVal + rightVal);
    // Reset the motoneuron
    SetNextState(leftId,  0);
    SetNextState(rightId, 0);
  }

  // Gather total for neck muscles
  int32_t leftNeckTotal  = 0;
  int32_t rightNeckTotal = 0;
  for (int i = 0; i < N_NNECKMUSCLES; i++) {
    // Get the motoneuron
    uint16_t leftId  = pgm_read_word_near(LeftNeckMuscles  + i);
    uint16_t rightId = pgm_read_word_near(RightNeckMuscles + i);
    // Get the value
    int16_t leftVal  = GetNextState(leftId);
    int16_t rightVal = GetNextState(rightId);
    // Only positive states
    if (leftVal < 0)  leftVal  = 0;
    if (rightVal < 0) rightVal = 0;
    // Get grand totals
    leftNeckTotal  += leftVal;
    rightNeckTotal += rightVal;
    // Reset the motoneuron
    SetNextState(leftId,  0);
    SetNextState(rightId, 0);
  }
  /*
    Serial.print(leftNeckTotal);
    Serial.print(",");
    Serial.print(rightNeckTotal);
    Serial.print(",");
  */
  //Serial.println(bodyTotal);

  //int16_t normBodyTotal = 255.0 * ((float) bodyTotal) / 600.0;

  // Log A and B type motor neuron activity
  int16_t motorNeuronASum = 0;
  int16_t motorNeuronBSum = 0;

  for (int i = 0; i < N_SIGMOTORB; i++) {
    uint8_t motorBId = pgm_read_word_near(SigMotorNeuronsB + i);
    if (GetCurrState(motorBId) > N_THRESHOLD)
      motorNeuronBSum += 1;
  }

  for (int i = 0; i < N_SIGMOTORA; i++) {
    uint8_t motorAId = pgm_read_word_near(SigMotorNeuronsA + i);
    if (GetCurrState(motorAId) > N_THRESHOLD)
      motorNeuronASum += 1;
  }

  // Sum (with weights) and add contribution to running average of significant activity
  int16_t motorNeuronSumTotal = motorNeuronBSum - motorNeuronASum;

  MotorNeuronAvg = (MotorNeuronAvg + 100 * motorNeuronSumTotal) / 2;

  // Set left and right totals, scale neck muscle contribution
  //int32_t leftTotal  = (4 * leftNeckTotal)  + normBodyTotal;
  //int32_t rightTotal = (4 * rightNeckTotal) + normBodyTotal;
  //int16_t leftTotal  = (10 * leftNeckTotal)  + bodyTotal;
  //int16_t rightTotal = (10 * rightNeckTotal) + bodyTotal;

  RightMotorAvg  = (12 * RightMotorAvg + (20 * rightNeckTotal) + bodyTotal) / 15;
  LeftMotorAvg  =  (12 * LeftMotorAvg +  (20 * leftNeckTotal)  + bodyTotal) / 15;


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



  // Magic number read off from c_matoduino simulation
  if (MotorNeuronAvg < 60) RunMotors(-RightMotorAvg, -LeftMotorAvg); //RunMotors(-rightTotal, -leftTotal);
  else                     RunMotors( RightMotorAvg,  LeftMotorAvg); //RunMotors( rightTotal,  leftTotal);
}

//
// Standard Arduino setup and loop functions
//

void setup() {
  // put your setup code here, to run once:

  // Uncomment for serial debugging
  Serial.begin(115200);
  //Serial.println(F("Nematoduino"));

  // Initialize state arrays
  StatesInit();
  // Initialize the motors
  MotorsInit();
  // Initialize the sensor
  SensorInit();
  // Initialize status LED
  pinMode(statusPin, OUTPUT);

  // Loop until something moves ahead
  while (SensorDistance() > 20) delay(100);

  // Start the neuro timer
  neuroNextTime = millis();
}

void loop() {
  //static unsigned long noseTimeout = 5000UL;

  // Neuro cycle
  if (millis() >= neuroNextTime) {
    // Repeat neuro cycle
    neuroNextTime += neuroDelay;


    // put your main code here, to run repeatedly:
    snsFront = SensorDistance();

    /*
      if (millis() > noseTimeout) {
        if (dist == 100)
          dist = 10;
        else if (dist == 10)
          dist = 1000;
        else
          dist = 100;
        noseTimeout += 5000UL;
      }
    */

    // Check if the sensor should activate
    if (snsFront < snsFrontThreshold) {
      // Status LED on
      digitalWrite(statusPin, HIGH);
      // Nose touch neurons
      PingNeuron(N_FLPR);
      PingNeuron(N_FLPL);
      PingNeuron(N_ASHL);
      PingNeuron(N_ASHR);
      PingNeuron(N_IL1VL);
      PingNeuron(N_IL1VR);
      PingNeuron(N_OLQDL);
      PingNeuron(N_OLQDR);
      PingNeuron(N_OLQVR);
      PingNeuron(N_OLQVL);

      PingNeuron(N_IL1L);
      PingNeuron(N_IL1R);
      PingNeuron(N_IL1DL);
      PingNeuron(N_IL1DR);
    }
    else {
      // Status LED off
      digitalWrite(statusPin, LOW);
      // Chemotaxis neurons
      PingNeuron(N_ADFL);
      PingNeuron(N_ADFR);
      PingNeuron(N_ASGR);
      PingNeuron(N_ASGL);
      PingNeuron(N_ASIL);
      PingNeuron(N_ASIR);
      PingNeuron(N_ASJR);
      PingNeuron(N_ASJL);

      PingNeuron(N_AWCL);
      PingNeuron(N_AWCR);
      PingNeuron(N_AWAL);
      PingNeuron(N_AWAR);
    }

    /* TODO
      Left touch sensors: PLML, PVDL, PDEL, PVM and LUAL.
      Right touch sensors: PLMR, PVDR, PDER, PVM and LUAR.
    */

    // Run the neuro cycle
    NeuralCycle();
  }
}
