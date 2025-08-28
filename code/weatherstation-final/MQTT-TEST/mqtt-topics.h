#ifndef mqtt_topics_H
#define mqtt_topics_H

// MQTT topics
namespace MqttTopics
{
    constexpr const char *InternalTemp = "weatherstation/internal_temperature";
    constexpr const char *BmeTemp = "weatherstation/bme280/temp";
    constexpr const char *BmeHumidity = "weatherstation/bme280/humidity";
    constexpr const char *BmePressure = "weatherstation/bme280/pressure";
    constexpr const char *VemlLux = "weatherstation/veml7700/lux";
    constexpr const char *VemlAls = "weatherstation/veml7700/als";
    constexpr const char *VemlWhite = "weatherstation/veml7700/white";
    constexpr const char *HallPulses = "weatherstation/hall/pulses_per_sec";
    constexpr const char *HallMl = "weatherstation/hall/ml";
    constexpr const char *HallMm = "weatherstation/hall/mm";
    constexpr const char *As5600Angle = "weatherstation/as5600/port%d/angle";
    constexpr const char *As5600Rpm = "weatherstation/as5600/port%d/rpm";
    constexpr const char *Aht21Temp = "weatherstation/aht21/temp";
    constexpr const char *Aht21Humidity = "weatherstation/aht21/humidity";
    constexpr const char *am2315cTemp = "weatherstation/am2315c/temp";
    constexpr const char *am2315cHumidity = "weatherstation/am2315c/humidity";

    constexpr const char *EnsState = "weatherstation/ens160/state";
    constexpr const char *EnsVoc = "weatherstation/ens160/voc";
    constexpr const char *EnsCo2 = "weatherstation/ens160/co2";
    constexpr const char *EnsAirq = "weatherstation/ens160/airq";
}

#endif // mqtt_topics_H