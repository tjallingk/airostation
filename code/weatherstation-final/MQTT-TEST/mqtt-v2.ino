#include <Wire.h>
#include <ElegantOTA.h>
#include <WebServer.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "tca9548Multiplexer.h"
#include "sensors.h"
#include "networking.h"

WebServer server(80);

// MQTT topics
// const char *espTempTopic = "weatherstation/internal_temperature";
// const char *as5600RpmTopic = "weatherstation/as5600/rpm"; // New topic for RPM

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Initialize networking
    connectToWiFi();
    initializeMQTT(mqttClient);

    // Start ElegantOTA web server
    ElegantOTA.begin(&server); // Start ElegantOTA
    Serial.println("ElegantOTA started. Open http://<device_ip>/update in your browser");
    server.begin();

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    // Initialize sensors
    while (!mqttClient.connected())
    {
        reconnectMQTT(mqttClient);
    }
    initializeSensors(mqttClient);
}

void loop()
{
    // Ensure MQTT connection
    if (!mqttClient.connected())
    {
        reconnectMQTT(mqttClient);
    }
    mqttClient.loop();

    // ElegantOTA handler
    server.handleClient();

    // Read and publish data from all sensors
    readAndPublishAllSensors(mqttClient);

    delay(800); // Adjust delay for your application
}

void connectToWiFi()
{
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Connected!");
}

void initializeMQTT(PubSubClient &mqttClient)
{
    mqttClient.setServer(mqttBroker, mqttPort);
}

void reconnectMQTT(PubSubClient &mqttClient)
{
    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("WeatherStation", mqttUser, mqttPassword))
        {
            Serial.println(" Connected!");
        }
        else
        {
            Serial.printf(" Failed, rc=%d. Retrying in 5 seconds...\n", mqttClient.state());
            delay(5000);
        }
    }
}
