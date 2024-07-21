#include <LTR390.h>
#include "Adafruit_MAX1704X.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <ESP32Ping.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include "config.h"

Adafruit_MAX17048 maxlipo;
LTR390 ltr390(LTR390_I2C_ADDRESS);

// Interval in ms of the reads
int counter = 0;
long interval = 1000 * 30;
long lastMessage = 0;

// Declare the custom functions
void setup_wifi();
void setup_sensors();
void publishHomeAssistantConfigMessage();
void reconnect();
void mqtt_connect();
void publishGenericMessage(const char *topic, const char *payload);

// Setting for NTP Time
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// Init WiFi/WiFiUDP, NTP and MQTT Client
WiFiUDP ntpUDP;
WiFiClient espClient;
NTPClient timeClient(ntpUDP);
PubSubClient client(mqtt_server, mqtt_port, espClient);

// Prefix for the MQTT Client Identification
String clientId = "esp32-client-";

bool DISCOVERY_ENABLED = false;

struct TopicInfo
{
  const String discovery_topic;
  const String state_topic;
  const String name;
  const String unit;
  const String json_key_name;
};

const TopicInfo UV_INDEX_TOPIC = {
    .discovery_topic = "homeassistant/sensor/ltr390/uvindex/config",
    .state_topic = "homeassistant/sensor/ltr390/uvindex/state",
    .name = "UV",
    .unit = "index",
    .json_key_name = "uvindex"};

const TopicInfo UV_BATTERY_TOPIC = {
    .discovery_topic = "homeassistant/sensor/ltr390/battpercent/config",
    .state_topic = "homeassistant/sensor/ltr390/battpercent/state",
    .name = "Battery %",
    .unit = "%",
    .json_key_name = "battpercent"};

const TopicInfo UV_BATTERY_VOLT_TOPIC = {
    .discovery_topic = "homeassistant/sensor/ltr390/battvolt/config",
    .state_topic = "homeassistant/sensor/ltr390/battvolt/state",
    .name = "Battery Voltage",
    .unit = "V",
    .json_key_name = "battvolt"};

const TopicInfo *TOPICS[] = {&UV_INDEX_TOPIC,
                             &UV_BATTERY_TOPIC,
                             &UV_BATTERY_VOLT_TOPIC};

const int TOPICS_SIZE = sizeof(TOPICS) / sizeof(TOPICS[0]);

void setup()
{

  // Might be a good thing to explore for battery life
  // setCpuFrequencyMhz(80);

  Serial.begin(115200);
  delay(500);

  // Initialize with log level and log output.
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  // Log ESP Chip information
  Log.notice(F("ESP32 Chip model %s Rev %d" CR), ESP.getChipModel(), ESP.getChipRevision());
  Log.notice(F("This chip has %d cores" CR), ESP.getChipCores());

  // Setup Wifi & MQTT
  setup_wifi();

  // Sensor Setup
  setup_sensors();
}

void setup_sensors()
{
  if (!ltr390.init())
  {
    Serial.println("LTR390 not connected!");
  }

  ltr390.setMode(LTR390_MODE_UVS);
  ltr390.setGain(LTR390_GAIN_18);
  ltr390.setResolution(LTR390_RESOLUTION_20BIT);

  while (!maxlipo.begin())
  {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x"));
  Serial.println(maxlipo.getChipID(), HEX);
}

void setup_wifi()
{
  Log.notice(F("Connecting to WiFi network: %s (password: %s)" CR), ssid, password);

  WiFi.begin(ssid, password);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(clientId.c_str());

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");

  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println("");

  Log.info(F("http://%s/device_info" CR), WiFi.localIP().toString());

  bool success = Ping.ping(mqtt_server, 3);

  if (!success)
  {
    Log.error(F("Ping failed to MQTT server at %s" CR), mqtt_server);
    return;
  }

  Log.notice(F("Ping to MQTT server at %s OK" CR), mqtt_server);

  // When setup wifi ok turn on led board
  digitalWrite(ONBOARD_LED, HIGH);

  // Configure mqtt
  client.setBufferSize(512);

  timeClient.begin();
  timeClient.setTimeOffset(0);

  clientId += String(random(0xffff), HEX);

  publishHomeAssistantConfigMessage();
}

void loop()
{

  long now = millis();

  mqtt_connect();

  if (now - lastMessage > interval)
  {
    lastMessage = now;

    char fullOutput[100];
    sprintf(fullOutput, "V: %.3f \nPercent: %.1f %%\nUV Index: %.1f\n\n", maxlipo.cellVoltage(), maxlipo.cellPercent(), ltr390.getUVI());

    Serial.println(fullOutput);

    float wfac = 1.0;  // wfac
    float gain = 18.0; // gain

    Serial.println(ltr390.readUVS());

    // float uvindex = ( (( ltr390.readUVS() * wfac) / 2300) * 4 * ( 18 / gain ));
    float uvindex = round(ltr390.readUVS() / 2300.0 * 100.0) / 100.0;

    Serial.println(uvindex);

    StaticJsonDocument<512> state_info;
    state_info[UV_INDEX_TOPIC.json_key_name] = uvindex;
    state_info[UV_BATTERY_TOPIC.json_key_name] = maxlipo.cellPercent();
    state_info[UV_BATTERY_VOLT_TOPIC.json_key_name] = maxlipo.cellVoltage();

    char stateInfoAsJson[512];
    serializeJson(state_info, stateInfoAsJson);

    // todo condense into a single topic
    publishGenericMessage(UV_INDEX_TOPIC.state_topic.c_str(), stateInfoAsJson);
    publishGenericMessage(UV_BATTERY_TOPIC.state_topic.c_str(), stateInfoAsJson);
    publishGenericMessage(UV_BATTERY_VOLT_TOPIC.state_topic.c_str(), stateInfoAsJson);
  }
}

void mqtt_connect()
{
  if (!client.connected())
  {
    reconnect();
  }

  client.loop();
}

// void handleDiscoveryOn(AsyncWebServerRequest *request)
// {
//   publishHomeAssistantConfigMessage();
//   DISCOVERY_ENABLED = true;
//   request->send(200, "text/plain", "Discovery turned on");
// }

// void handleDiscoveryOff(AsyncWebServerRequest *request)
// {

//   DISCOVERY_ENABLED = false;

//   // Send blank message to the discovery topic to remove it
//   const int TOPICS_SIZE = sizeof(TOPICS) / sizeof(TOPICS[0]);
//   for (int index = 0; index < TOPICS_SIZE; index++)
//   {
//     publishGenericMessage(TOPICS[index]->discovery_topic.c_str(), "");
//   }

//   request->send(200, "text/plain", "Discovery turned off");
// }

void publishGenericMessage(const char *topic, const char *payload)
{

  mqtt_connect();

  if (!client.connected())
  {
    reconnect();
  }

  Log.info(F("Publishing to topic %s" CR), topic);
  Log.info(F("Payload %s" CR), payload);

  boolean outcome = client.publish(topic, payload);
  Serial.println(outcome ? "Message sent" : "Message failed");
}

void publishHomeAssistantConfigMessage()
{
  StaticJsonDocument<512> device;
  device["name"] = "BME680";
  device["identifiers"] = "023456789";

  const int TOPICS_SIZE = sizeof(TOPICS) / sizeof(TOPICS[0]);
  for (int index = 0; index < TOPICS_SIZE; index++)
  {
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
}

/**
 * Reconnect to MQTT Broker
 */
void reconnect()
{
  Log.info("Reconnecting.." CR);
  // Loop until we're reconnected
  while (!client.connected())
  {
    Log.notice(F("Attempting MQTT connection to %s" CR), mqtt_server);

    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password))
    {
      Log.notice(F("Connected as clientId %s :-)" CR), clientId.c_str());
      // Turn on led board
      digitalWrite(ONBOARD_LED, HIGH);
    }
    else
    {
      Log.error(F("{failed, rc=%d try again in 5 seconds}" CR), client.state());
      // Turn off led board and wait 5 seconds before retrying
      digitalWrite(ONBOARD_LED, LOW);
      delay(5000);
    }
  }
}