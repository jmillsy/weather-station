#include "ha_integration.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include "mqtt_client.h"
#include "config.h"

// Firmware version
const char* FIRMWARE_VERSION = "1.2";

// External variables from main.cpp
extern RTC_DATA_ATTR int bootCount;

// Discovery control
bool DISCOVERY_ENABLED = false;

// Topic definitions
const TopicInfo UV_INDEX_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/uvindex/config",
    .state_topic = "homeassistant/sensor/weatherstation/uvindex/state",
    .name = "UV",
    .unit = "index",
    .json_key_name = "uvindex"};



const TopicInfo TEMPERATURE_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/temp/config",
    .state_topic = "homeassistant/sensor/weatherstation/temp/state",
    .name = "Temperature",
    .unit = "C",
    .json_key_name = "temp"};

const TopicInfo HUMIDITY_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/humidity/config",
    .state_topic = "homeassistant/sensor/weatherstation/humidity/state",
    .name = "Humidity",
    .unit = "%",
    .json_key_name = "humidity"};

// Array of all topics
const TopicInfo *TOPICS[] = {&UV_INDEX_TOPIC,
                             &TEMPERATURE_TOPIC,
                             &HUMIDITY_TOPIC};

const int TOPICS_SIZE = sizeof(TOPICS) / sizeof(TOPICS[0]);

void publishHomeAssistantConfigMessage()
{
  Serial.println("[HA] Publishing Home Assistant discovery messages");
  
  StaticJsonDocument<512> device;
  device["name"] = "Weather Station";
  device["identifiers"] = "backyard-weather-station";

  for (int index = 0; index < TOPICS_SIZE; index++)
  {
    Log.notice(F("[HA] Publishing to topic %s" CR), TOPICS[index]->discovery_topic.c_str());
    
    StaticJsonDocument<512> config;
    config["name"] = TOPICS[index]->name;
    config["state_topic"] = TOPICS[index]->state_topic;
    config["unit_of_measurement"] = TOPICS[index]->unit;
    String unique_id = "bme680" + TOPICS[index]->name;
    unique_id.replace(" ", "");
    unique_id.toLowerCase();
    config["unique_id"] = unique_id;

    String value_template = "{{value_json." + TOPICS[index]->json_key_name + " | float | round(2) }}";
    config["value_template"] = value_template;

    // Add the device object to the main config
    config["device"] = device;

    // Serialize the JSON object to a string
    char configAsJson[512];
    serializeJson(config, configAsJson);

    publishGenericMessage(TOPICS[index]->discovery_topic.c_str(), configAsJson);
  }
  
  Log.notice(F("[HA] Home Assistant discovery messages published" CR));
}

void pushSensorDataToMQTT(float temp, float humidity, float uv_index, bool debug)
{
  Serial.println("[HA] Pushing sensor data to MQTT topics");
  
  StaticJsonDocument<512> state_info;
  state_info[UV_INDEX_TOPIC.json_key_name] = uv_index;
  state_info[TEMPERATURE_TOPIC.json_key_name] = temp;
  state_info[HUMIDITY_TOPIC.json_key_name] = humidity;

  // Only do this if debug is enabled.
  if (debug)
  {
    float percentFreeHeap = (float)ESP.getFreeHeap() / ESP.getHeapSize() * 100.0;
    state_info["debug-boot-count"] = bootCount;
    state_info["debug-free-heap"] = percentFreeHeap;
    state_info["debug-wake_reason"] = esp_sleep_get_wakeup_cause();
    state_info["debug-firmware-version"] = FIRMWARE_VERSION;
  }

  char stateInfoAsJson[512];
  serializeJson(state_info, stateInfoAsJson);

  // Publish to all state topics
  publishGenericMessage(UV_INDEX_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(TEMPERATURE_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(HUMIDITY_TOPIC.state_topic.c_str(), stateInfoAsJson);
  
  Log.notice(F("[HA] Sensor data published to all topics" CR));
}