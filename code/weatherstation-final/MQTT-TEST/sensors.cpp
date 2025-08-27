#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include <Wire.h>
#include "mqtt-topics.h"
#include "networking.h"

// #include <WiFi.h>

#include "sensors.h"
#include "tca9548Multiplexer.h"
// #include "config.h"

// Sensor instances
Adafruit_BME280 bme;
Adafruit_VEML7700 veml;
Adafruit_AHTX0 aht;
SparkFun_ENS160 ens;
// AS5600 as5600[4];

// Flags for sensor connections
bool bmeConnected = false;
bool vemlConnected = false;
bool ensConnected = false;
bool ahtConnected = false;
bool as5600Connected[4] = {false};

#define HALL_SENSOR_PIN 23

volatile unsigned long hallPulseCount = 0;
unsigned long lastHallPublishTime = 0;
unsigned long lastHallResetTime = 0;
unsigned long lastHallPulseCount = 0;
unsigned long pulses = 0;
void initializeSensors(PubSubClient &mqttClient)
{
    // rain sensor tipping bucket magnet detector
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, RISING);

    bmeConnected = bme.begin(0x76);
    Serial.println(bmeConnected ? "BME280 detected." : "BME280 not detected.");
    if (bmeConnected)
    {
        publishHADiscovery(mqttClient, "bme280_temp", "BME280 Temperature", MqttTopics::BmeTemp, "°C", "temperature", nullptr);
        publishHADiscovery(mqttClient, "bme280_humidity", "BME280 Humidity", MqttTopics::BmeHumidity, "%", "humidity", nullptr);
        publishHADiscovery(mqttClient, "bme280_pressure", "BME280 Pressure", MqttTopics::BmePressure, "hPa", "pressure", nullptr);
    }

    vemlConnected = veml.begin(&Wire);
    Serial.println(vemlConnected ? "VEML7700 detected." : "VEML7700 not detected.");

    for (int port = 0; port < 4; port++)
    {
        selectMultiplexerPort(port);
        as5600Connected[port] = testAS5600Connection();
        Serial.printf("AS5600 %s on port %d.\n", as5600Connected[port] ? "detected" : "not detected", port);
    }

    selectMultiplexerPort(4);
    ahtConnected = aht.begin();
    Serial.println(ahtConnected ? "AHT21 detected." : "AHT21 not detected.");

    selectMultiplexerPort(4);
    ensConnected = ens.begin();
    ens.setOperatingMode(SFE_ENS160_STANDARD);
    Serial.println(ensConnected ? "ENS160 detected." : "ENS160 not detected.");
}

void readAndPublishAllSensors(PubSubClient &mqttClient)
{
    // Read and publish internal temperature
    // float internalTemp = readInternalTemperature();
    // if (!isnan(internalTemp)) {
    //     publishSensorData(mqttClient, "weatherstation/internal_temperature", internalTemp);
    // }

    // Read and publish BME280 data
    if (bmeConnected)
    {
        publishSensorData(mqttClient, MqttTopics::BmeTemp, bme.readTemperature());
        publishSensorData(mqttClient, MqttTopics::BmeHumidity, bme.readHumidity());
        publishSensorData(mqttClient, MqttTopics::BmePressure, bme.readPressure());
    }

    // Read and publish VEML7700 data
    if (vemlConnected)
    {
        publishSensorData(mqttClient, MqttTopics::VemlLux, veml.readLux());
        publishSensorData(mqttClient, MqttTopics::VemlAls, veml.readALS());
        publishSensorData(mqttClient, MqttTopics::VemlWhite, veml.readWhite());
    }

    // --- Hall sensor pulse counting ---
    unsigned long now = millis();
    if (now - lastHallPublishTime >= 3000)
    { // every second
        pulses = hallPulseCount - lastHallPulseCount;
        lastHallPublishTime = now;
        // Serial.println();
        //  Publish pulse count per second (Hz)
        publishSensorData(mqttClient, MqttTopics::HallPulses, (float)pulses);

        // If you know how many pulses per revolution, you can also publish RPM:
        // For example, if 1 pulse = 1 revolution:
        float ml = pulses * 2.6;
        publishSensorData(mqttClient, MqttTopics::HallMl, ml);
        float mm = ml / 20.106;
        publishSensorData(mqttClient, MqttTopics::HallMm, mm);
    }
    if (now - lastHallResetTime >= 60000)
    { // every second
        // unsigned long pulses = hallPulseCount - lastHallPulseCount;
        lastHallPulseCount = hallPulseCount;
        lastHallResetTime = now;
        // Serial.println();
        //  Publish pulse count per second (Hz)
        // publishSensorData(mqttClient, "weatherstation/hall/pulses_per_sec", (float)pulses);

        // If you know how many pulses per revolution, you can also publish RPM:
        // For example, if 1 pulse = 1 revolution:
        // float ml = pulses * 3.5;
        // publishSensorData(mqttClient, "weatherstation/hall/ml", ml);
    }

    // Read and publish AS5600 data
    for (int port = 0; port < 4; port++)
    {
        selectMultiplexerPort(port);
        if (as5600Connected[port])
        {
            uint16_t angle = readAS5600Angle();
            char angleTopic[50];
            snprintf(angleTopic, sizeof(angleTopic), MqttTopics::As5600Angle, port);
            publishSensorData(mqttClient, angleTopic, angle);

            // Calculate and publish RPM
            float rpm = calculateRPM(port, 1000);
            char rpmTopic[50];
            snprintf(rpmTopic, sizeof(rpmTopic), MqttTopics::As5600Rpm, port);
            publishSensorData(mqttClient, rpmTopic, rpm);
        }
    }

    // Read and publish AHT21 data
    selectMultiplexerPort(4);
    if (ahtConnected)
    {
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
        ens.setTempCompensationCelsius(temp.temperature);
        ens.setRHCompensationFloat(humidity.relative_humidity);
        publishSensorData(mqttClient, MqttTopics::AhtTemp, temp.temperature);
        publishSensorData(mqttClient, MqttTopics::AhtHumidity, humidity.relative_humidity);
    }

    // Read and publish ENS160 data
    selectMultiplexerPort(4);
    if (ensConnected)
    {
        publishSensorData(mqttClient, MqttTopics::EnsState, ens.getFlags());
        publishSensorData(mqttClient, MqttTopics::EnsVoc, ens.getTVOC());
        publishSensorData(mqttClient, MqttTopics::EnsCo2, ens.getECO2());
        publishSensorData(mqttClient, MqttTopics::EnsAirq, ens.getAQI());
    }
}

void IRAM_ATTR hallSensorISR()
{
    Serial.println("bucket tip found");
    hallPulseCount++;
}
// Function to read a byte from a specific memory address on the I2C device
uint8_t readI2CRegister(uint8_t deviceAddress, uint8_t memoryAddress)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(memoryAddress); // Specify the memory address
    if (Wire.endTransmission(false) != 0)
    { // Send the memory address, keep the connection alive
        Serial.println("Failed to send memory address");
        return 0; // Return 0 in case of error
    }

    Wire.requestFrom(deviceAddress, (uint8_t)1); // Request 1 byte of data
    if (Wire.available())
    {
        return Wire.read(); // Read and return the data
    }
    else
    {
        Serial.println("No data available");
        return 0; // Return 0 if no data is available
    }
}

float readAS5600Angle()
{
    uint8_t highByte = readI2CRegister(0x36, 0x0E);
    uint8_t lowByte = readI2CRegister(0x36, 0x0F);
    uint16_t angle = ((highByte & 0x0F) << 8) | lowByte;
    return map(angle, 0, 4095, 0, 360);
}

bool testAS5600Connection()
{
    Wire.beginTransmission(0x36); // AS5600 I2C address
    Wire.write(0x0B);             // Attempt to access the CONF register
    if (Wire.endTransmission() != 0)
    {
        return false; // If there's an error, the AS5600 is not connected
    }

    // Read back the register value to confirm communication
    Wire.requestFrom(0x36, 1); // Request 1 byte from the CONF register
    if (Wire.available())
    {
        uint8_t conf = Wire.read();
        return true; // Successfully read, sensor is connected
    }
    return false; // No data was read
}

// float readInternalTemperature() {
//     int raw = hallRead();
//     return (raw / 4.4) - 32;
// }

void publishSensorData(PubSubClient &mqttClient, const char *topic, float value)
{
    char msg[50];
    snprintf(msg, sizeof(msg), "%.2f", value);
    mqttClient.publish(topic, msg);
}

unsigned long lastTime = 0;
uint16_t lastAngle = 0;

float calculateRPM(uint8_t port, unsigned long sampleDurationMs = 1000)
{
    selectMultiplexerPort(port);

    // Record the initial angle and timestamp
    uint16_t previousAngle = readAS5600Angle();
    unsigned long startTime = micros();
    unsigned long currentTime = startTime;

    int totalAngleChange = 0;

    // Sample the angle continuously during the sampling period
    while (currentTime - startTime < (sampleDurationMs * 1000))
    {
        uint16_t currentAngle = readAS5600Angle();
        int angleDiff = currentAngle - previousAngle;

        // Handle rollover (angle ranges from 0 to 360)
        if (angleDiff < -180)
        {
            // Serial.println("angleDiff < -180");
            // Serial.print("angle1: ");
            // Serial.println(currentAngle);
            // Serial.print("angle2: ");
            // Serial.println(previousAngle);

            angleDiff += 360;
        }
        else if (angleDiff > 180)
        {
            // Serial.println("angleDiff > 180");
            // Serial.print("angle1: ");
            // Serial.println(currentAngle);
            // Serial.print("angle2: ");
            // Serial.println(previousAngle);
            angleDiff -= 360;
        }

        totalAngleChange += angleDiff;
        previousAngle = currentAngle;

        // Update the current time
        currentTime = micros();

        // Sample every millisecond
        delayMicroseconds(1000);
    }
    // Serial.print("total angle change:");
    // Serial.println(totalAngleChange);
    //  Calculate the total revolutions
    float totalRevolutions = totalAngleChange / 360;
    // Serial.print("total revs:");
    // Serial.println(totalRevolutions);
    //  Calculate RPM
    float rpm = (totalRevolutions / (sampleDurationMs / 1000.0)) * 60.0;
    float rpm2 = ((totalAngleChange / (sampleDurationMs / 1000.0)) * 60.0) / 360.00;
    // Serial.print("rpm1:");
    // Serial.println(rpm);
    // Serial.print("rpm2:");
    // Serial.println(rpm2);
    return rpm2;
}