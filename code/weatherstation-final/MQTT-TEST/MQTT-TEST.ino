#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AHTX0.h>
#include <SparkFun_ENS160.h> 
#include <AS5600.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"
#include "tca9548Multiplexer.h"

// MQTT topics
const char* espTempTopic = "weatherstation/internal_temperature";
const char* bme280Topic = "weatherstation/bme280";
const char* veml7700Topic = "weatherstation/veml7700";
const char* ens160Topic = "weatherstation/ens160";
const char* aht21Topic = "weatherstation/aht21";
const char* as5600TopicBase = "weatherstation/as5600"; // Add port number dynamically

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Sensors
Adafruit_BME280 bme;
Adafruit_VEML7700 veml;
Adafruit_AHTX0 aht;
SparkFun_ENS160 ens; // SparkFun ENS160 instance
AS5600 as5600[4]; // Array for 4 AS5600 sensors

// Flags to track connected sensors
bool bmeConnected = false;
bool vemlConnected = false;
bool ensConnected = false;
bool ahtConnected = false;
bool as5600Connected[4] = {false, false, false, false}; // One flag per AS5600 sensor

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize WiFi
  connectToWiFi();

  // Initialize MQTT
  client.setServer(mqttBroker, mqttPort);
  client.setCallback(mqttCallback);
  connectToMQTT();

  // Probe directly connected sensors
  bmeConnected = bme.begin(0x76);
  if (bmeConnected) {
    Serial.println("BME280 detected.");
  } else {
    Serial.println("BME280 not detected.");
  }

  vemlConnected = veml.begin(&Wire); // Fixed initialization
  if (vemlConnected) {
    Serial.println("VEML7700 detected.");
  } else {
    Serial.println("VEML7700 not detected.");
  }

  initializeMultiplexerSensors();
}

void loop() {
  // Ensure the MQTT connection is alive
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  // Read and publish ESP32 internal temperature
  float internalTemp = readInternalTemperature();
  if (!isnan(internalTemp)) {
    publishSensorData(espTempTopic, internalTemp);
  }

  // Read and publish directly connected sensors
  if (bmeConnected) {
    float temp = bme.readTemperature();
    float humidity = bme.readHumidity();
    publishSensorData(bme280Topic, temp); // Extend for humidity if needed
  }

  if (vemlConnected) {
    float lux = veml.readLux();
    publishSensorData(veml7700Topic, lux);
  }

  // Read and publish sensors on the multiplexer
  for (int port = 0; port < 4; port++) {
    selectMultiplexerPort(port);
    if (as5600Connected[port]) {
      float angle = as5600[port].getZPosition(); // Retrieve position
      char topic[50];
      snprintf(topic, 50, "%s/port%d", as5600TopicBase, port);
      publishSensorData(topic, angle);
    }
  }

  selectMultiplexerPort(4);
  if (ahtConnected) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    publishSensorData(aht21Topic, temp.temperature);
  }

  if (ensConnected) {
    float airQuality = ens.getAQI();
    publishSensorData(ens160Topic, airQuality);
  }

  delay(5000); // Adjust delay for your application
}


void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("WeatherStation", mqttUser, mqttPassword)) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void publishSensorData(const char* topic, float value) {
  char msg[50];
  snprintf(msg, 50, "%.2f", value);
  client.publish(topic, msg);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
}

float readInternalTemperature() {
  int raw = hallRead();
  float tempC = (raw / 4.4) - 32;
  return tempC;
}