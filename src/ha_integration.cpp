#include "ha_integration.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include "mqtt_client.h"
#include "config.h"
#include "sensors.h"

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
    .value_template = "{{ value_json.uvindex | float | round(2) }}",
    .name = "UV",
    .unit = "index",
    .json_key_name = "uvindex"};



const TopicInfo TEMPERATURE_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/temp/config",
    .state_topic = "homeassistant/sensor/weatherstation/temp/state",
    .value_template = "{{ value_json.temp | float | round(2) }}",
    .name = "Temperature",
    .unit = "C",
    .json_key_name = "temp"};

const TopicInfo HUMIDITY_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/humidity/config",
    .state_topic = "homeassistant/sensor/weatherstation/humidity/state",
    .value_template = "{{ value_json.humidity | float | round(2) }}",
    .name = "Humidity",
    .unit = "%",
    .json_key_name = "humidity"};

const TopicInfo BATTERY_VOLTAGE_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/battery/config",
    .state_topic = "homeassistant/sensor/weatherstation/battery/state",
    .value_template = "{{ value_json.battery_voltage | float | round(2) }}",
    .name = "Battery Voltage",
    .unit = "V",
    .json_key_name = "battery_voltage"};

const TopicInfo BATTERY_PERCENT_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/battery_percent/config",
    .state_topic = "homeassistant/sensor/weatherstation/battery_percent/state",
    .value_template = "{{ value_json.battery_percent | float | round(1) }}",
    .name = "Battery Level",
    .unit = "%",
    .json_key_name = "battery_percent"};

// Array of all topics
const TopicInfo *TOPICS[] = {&UV_INDEX_TOPIC,
                             &TEMPERATURE_TOPIC,
                             &HUMIDITY_TOPIC,
                             &BATTERY_VOLTAGE_TOPIC,
                             &BATTERY_PERCENT_TOPIC};

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
    config["value_template"] = TOPICS[index]->value_template;
    
    // Generate appropriate unique_id based on sensor type
    String unique_id;
    if (TOPICS[index]->json_key_name == "uvindex") {
        unique_id = "ltr390uv";
    } else if (TOPICS[index]->json_key_name == "battery_voltage") {
        unique_id = "max17048batteryvoltage";
    } else if (TOPICS[index]->json_key_name == "battery_percent") {
        unique_id = "max17048batterypercent";
    } else {
        unique_id = "bme680" + TOPICS[index]->name;
        unique_id.replace(" ", "");
        unique_id.toLowerCase();
    }
    config["unique_id"] = unique_id;

    //String value_template = "{{value_json." + TOPICS[index]->json_key_name + " | float | round(2) }}";
    

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

  // Battery monitoring using MAX17048 fuel gauge (always included)
  std::tuple<float, float> batteryData = get_battery_data();
  float batteryVoltage = std::get<0>(batteryData);
  float batteryPercent = std::get<1>(batteryData);
  
  // Always include battery info
  state_info[BATTERY_VOLTAGE_TOPIC.json_key_name] = round(batteryVoltage * 100.0) / 100.0; // Round to 2 decimals
  state_info[BATTERY_PERCENT_TOPIC.json_key_name] = round(batteryPercent * 10.0) / 10.0; // Round to 1 decimal

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
  publishGenericMessage(BATTERY_VOLTAGE_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(BATTERY_PERCENT_TOPIC.state_topic.c_str(), stateInfoAsJson);
  
  Log.notice(F("[HA] Sensor data published to all topics" CR));
}