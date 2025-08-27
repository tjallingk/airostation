#include "networking.h"
// #include "config.h"

// Helper to publish Home Assistant MQTT discovery config
void publishHADiscovery(PubSubClient &mqttClient, const char *unique_id, const char *name, const char *state_topic, const char *unit, const char *device_class, const char *value_template)
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic), "homeassistant/sensor/%s/config", unique_id);

    // Device object for Home Assistant
    const char *devicePayload =
        "\"dev\":{"
        "\"name\":\"weerstation-tuin\","
        "\"ids\":[\"weatherstation\"],"
        "\"mf\":\"tjalling\","
        "\"mdl\":\"ESP32 Weather-sensorStation\","
        "\"sw\":\"0.0.1\""
        "}";

    char payload[768];
    snprintf(payload, sizeof(payload),
             "{"
             "\"name\":\"%s\","
             "\"state_topic\":\"%s\","
             "\"unique_id\":\"%s\","
             "\"unit_of_measurement\":\"%s\"%s%s,"
             "%s"
             "%s"
             "}",
             name,
             state_topic,
             unique_id,
             unit,
             device_class ? ",\"device_class\":\"" : "",
             device_class ? device_class : "",
             value_template ? ",\"value_template\":\"%s\"" : "",
             devicePayload);

    mqttClient.publish(configTopic, payload, true); // Retain discovery message
}