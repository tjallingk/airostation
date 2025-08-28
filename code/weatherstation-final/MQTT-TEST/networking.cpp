#include "networking.h"
#include <ArduinoJson.h>

// #include "config.h"

// Helper to publish Home Assistant MQTT discovery config
void publishHADiscovery(PubSubClient &mqttClient, const char *unique_id, const char *name, const char *state_topic, const char *unit, const char *device_class, const char *value_template)
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic), "homeassistant/sensor/weatherstation/%s/config", unique_id);

    StaticJsonDocument<512> doc;
    doc["name"] = name;
    doc["state_topic"] = state_topic;
    doc["unique_id"] = unique_id;
    doc["unit_of_measurement"] = unit;
    if (device_class)
        doc["device_class"] = device_class;
    if (value_template)
        doc["value_template"] = value_template;

    // Device object for Home Assistant
    JsonObject device = doc.createNestedObject("device");
    device["name"] = "weerstation-tuin";
    device["ids"] = "weatherstation";
    device["mf"] = "tjalling";
    device["mdl"] = "ESP32 Weather-sensorStation";
    device["sw"] = "0.0.1";

    char payload[512];
    size_t n = serializeJson(doc, payload, sizeof(payload));
    // Serial.print("publishing to configtopic: ");
    // Serial.println(configTopic);
    // Serial.print("publishing this info: ");
    // Serial.println(payload);

    if (!mqttClient.connected())
    {
        Serial.println("MQTT not connected!");
    }
    //Serial.print("MQTT buffer size: ");
    //Serial.println(mqttClient.getBufferSize()); 
    mqttClient.setBufferSize(1024);
    mqttClient.publish(configTopic, (const uint8_t *)payload, (unsigned int)n, true); // Retain discovery message
    //Serial.println(mqttClient.publish(configTopic, payload));
    // mqttClient.publish(configTopic, "test msg");
}
