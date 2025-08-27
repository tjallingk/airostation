#ifndef NETWORK_H
#define NETWORK_H

#include <PubSubClient.h>

void publishHADiscovery(PubSubClient &mqttClient, const char *unique_id, const char *name, const char *state_topic, const char *unit, const char *device_class, const char *value_template);

#endif
