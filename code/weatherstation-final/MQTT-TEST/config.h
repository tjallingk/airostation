#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
const char* ssid = "wifiname";
const char* password = "password";
// MQTT credentials
const char* mqttBroker = "ip addr";
const int mqttPort = 1883; // Usually 1883
const char* mqttUser = "username to login as";
const char* mqttPassword = "passowrd";

#endif // CONFIG_H