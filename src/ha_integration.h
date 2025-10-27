#ifndef HA_INTEGRATION_H
#define HA_INTEGRATION_H

#include <Arduino.h>

// TopicInfo structure for Home Assistant topics
struct TopicInfo
{
  const String discovery_topic;
  const String state_topic;
  const String name;
  const String unit;
  const String json_key_name;
};

// Topic definitions
extern const TopicInfo UV_INDEX_TOPIC;
extern const TopicInfo TEMPERATURE_TOPIC;
extern const TopicInfo HUMIDITY_TOPIC;
extern const TopicInfo *TOPICS[];

// Firmware version
extern const char* FIRMWARE_VERSION;
extern const int TOPICS_SIZE;

// Discovery control
extern bool DISCOVERY_ENABLED;

// Function declarations
void publishHomeAssistantConfigMessage();
void pushSensorDataToMQTT(float temperature, float humidity, float uv_index, bool success);

#endif // HA_INTEGRATION_H