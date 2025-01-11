#ifndef TCA9548MULTIPLEXER_H
#define TCA9548MULTIPLEXER_H

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <SparkFun_ENS160.h>
#include <AS5600.h>

// Externally declare the global variables for sensor connections
extern Adafruit_AHTX0 aht;
extern SparkFun_ENS160 ens;
extern AS5600 as5600[4];
extern bool as5600Connected[4];
extern bool ensConnected;
extern bool ahtConnected;

// Declare the multiplexer control and initialization functions
void selectMultiplexerPort(uint8_t port);
void initializeMultiplexerSensors();

#endif // TCA9548MULTIPLEXER_H
