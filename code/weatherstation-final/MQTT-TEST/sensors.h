#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AHTX0.h>
#include <SparkFun_ENS160.h>
// #include <AS5600.h>
#include <PubSubClient.h>
// bool as5600Connected[4];

void initializeSensors(PubSubClient &mqttClient);
void readAndPublishAllSensors(PubSubClient &mqttClient);
void IRAM_ATTR hallSensorISR();
uint8_t readI2CRegister(uint8_t deviceAddress, uint8_t memoryAddress);
float readAS5600Angle();
bool testAS5600Connection();
float readInternalTemperature();
void publishSensorData(PubSubClient &mqttClient, const char *topic, float value);
float calculateRPM(uint8_t port, unsigned long sampleDurationMs);

#endif
