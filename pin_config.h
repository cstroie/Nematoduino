#ifndef pin_config_h
#define pin_config_h

// Button pin (configured so that on is LOW)
const uint8_t buttonPin = 2;

// Transmit and receive pins for distance sensor
const uint8_t tPin = A2;
const uint8_t rPin = A3;

// Pin for status LED (turns on when nose touch neurons stimulated)
const uint8_t statusPin = 13;

#endif
