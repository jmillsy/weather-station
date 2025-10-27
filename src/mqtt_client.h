#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <PubSubClient.h>
#include <WiFi.h>

// Function declarations
void setup_mqtt(const char *server, int port, const char *username, const char *password);
void reconnect();
void publishGenericMessage(const char *topic, const char *payload);
void publishGenericMessage(const char *topic, const char *payload, bool retain);
bool is_mqtt_connected();
bool check_for_sleep_pause(const char *device_name);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void clear_retained_message(const char *topic);

// External references for MQTT client access
extern PubSubClient client;
extern String clientId;

#endif // MQTT_CLIENT_H