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
#include "pin_config.h"

#include "Neurons.h"
#include "NeuNet.h"
#include "Sonar.h"
#include "Motor.h"

Sonar  sonar(pinTrigger, pinEcho, 100);
Motor  motors(3, 4, MOTOR34_1KHZ);
NeuNet neunet(&motors);


// Neuro cycle time
const unsigned long neuroDelay    = 50UL; // Delay between neuro cycles
unsigned long       neuroNextTime = 0UL;  // Time of next neuro cycle

// Sensors
int16_t       snsFront;                   // Front sensor distance
const int16_t snsFrontThreshold = 40;     // Threshold for front sensor activation (cm)



/**
  Main Arduino setup function
*/
void setup() {
  // Serial debug
  Serial.begin(115200);

#ifdef DEBUG
  Serial.print(F("N_NTOTAL: "));
  Serial.println(N_NTOTAL);
  Serial.print(F("N_MAX: "));
  Serial.println(N_MAX);
#endif

  /* Initialize the neural network */
  neunet.init();

  // Initialize status LED
  pinMode(statusPin, OUTPUT);

#ifndef DEVEL
  // Loop until something moves ahead
  while (sonar.echo() > 20) delay(100);
#endif

  // Start the neuro timer
  neuroNextTime = millis();

  //motors.run(200, 200);
}

/**
  Main Arduino loop
*/
void loop() {
#ifdef DEVEL
  static unsigned long noseTimeout = 5000UL;
#endif

  // Neuro cycle
  if (millis() >= neuroNextTime) {
    // Repeat neuro cycle
    neuroNextTime += neuroDelay;

#ifndef DEVEL
    snsFront = sonar.echo();
#endif

#ifdef DEVEL
    if (millis() > noseTimeout) {
      if (snsFront == 100)
        snsFront = 10;
      else if (snsFront == 10)
        snsFront = 1000;
      else
        snsFront = 100;
      noseTimeout += 5000UL;
    }
#endif

#ifdef DEBUG
  //Serial.println(snsFront);
#endif


    // Check if the sensor should activate
    if (snsFront < snsFrontThreshold) {
      // Status LED on
      digitalWrite(statusPin, HIGH);
      // Nose touch neurons
      neunet.ping(N_FLPR);
      neunet.ping(N_FLPL);
      neunet.ping(N_ASHL);
      neunet.ping(N_ASHR);
      neunet.ping(N_IL1VL);
      neunet.ping(N_IL1VR);
      neunet.ping(N_OLQDL);
      neunet.ping(N_OLQDR);
      neunet.ping(N_OLQVR);
      neunet.ping(N_OLQVL);

      neunet.ping(N_IL1L);
      neunet.ping(N_IL1R);
      neunet.ping(N_IL1DL);
      neunet.ping(N_IL1DR);
    }
    else {
      // Status LED off
      digitalWrite(statusPin, LOW);
      // Chemotaxis neurons
      neunet.ping(N_ADFL);
      neunet.ping(N_ADFR);
      neunet.ping(N_ASGR);
      neunet.ping(N_ASGL);
      neunet.ping(N_ASIL);
      neunet.ping(N_ASIR);
      neunet.ping(N_ASJR);
      neunet.ping(N_ASJL);

      neunet.ping(N_AWCL);
      neunet.ping(N_AWCR);
      neunet.ping(N_AWAL);
      neunet.ping(N_AWAR);
    }

    /* TODO
      Left touch sensors: PLML, PVDL, PDEL, PVM and LUAL.
      Right touch sensors: PLMR, PVDR, PDER, PVM and LUAR.
    */

    // Run the neuro cycle
    neunet.cycle();
  }
}
