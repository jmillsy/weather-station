#include "Adafruit_LTR390.h"
#include "Adafruit_MAX1704X.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <ESP32Ping.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_BME680.h"

Adafruit_NeoPixel neopixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

enum
{
  WHITE,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  OFF
};

uint32_t LED_Colors[8] = {
    // Red, Green, Blue
    0xFFFFFF, // White
    0xFF0000, // Red
    0x00FF00, // Green
    0x0000FF, // Blue
    0xFFC000, // Yellow
    0x00B0FF, // Cyan
    0xD000B0, // Magenta
    0x000000, // Off | Black
};

Adafruit_MAX17048 maxlipo;
Adafruit_LTR390 ltr390 = Adafruit_LTR390();
Adafruit_BME680 bme;

// Declare the custom functions
void pushSensorDataToMQTT(float, float, float, float, float, bool);
void setup_wifi();
void setup_sensors();
void publishHomeAssistantConfigMessage();
void reconnect();
void publishGenericMessage(const char *topic, const char *payload);
void print_startup_info();
std::tuple<float, float> get_battery_fuel();
std::tuple<float, float> get_bme680_data();
float get_uv_index();
void i2c_scan();
void setLedColor(uint32_t color);

// Init WiFi/WiFiUDP, NTP and MQTT Client
WiFiClient espClient;
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
    .discovery_topic = "homeassistant/sensor/weatherstation/uvindex/config",
    .state_topic = "homeassistant/sensor/weatherstation/uvindex/state",
    .name = "UV",
    .unit = "index",
    .json_key_name = "uvindex"};

const TopicInfo UV_BATTERY_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/battpercent/config",
    .state_topic = "homeassistant/sensor/weatherstation/battpercent/state",
    .name = "Battery %",
    .unit = "%",
    .json_key_name = "battpercent"};

const TopicInfo UV_BATTERY_VOLT_TOPIC = {
    .discovery_topic = "homeassistant/sensor/weatherstation/battvolt/config",
    .state_topic = "homeassistant/sensor/weatherstation/battvolt/state",
    .name = "Battery Voltage",
    .unit = "V",
    .json_key_name = "battvolt"};

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

const TopicInfo *TOPICS[] = {&UV_INDEX_TOPIC,
                             &UV_BATTERY_TOPIC,
                             &UV_BATTERY_VOLT_TOPIC,
                             &TEMPERATURE_TOPIC,
                             &HUMIDITY_TOPIC};

const int TOPICS_SIZE = sizeof(TOPICS) / sizeof(TOPICS[0]);

RTC_DATA_ATTR int bootCount = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  clientId += String(random(0xffff), HEX);

  neopixel.begin();
  neopixel.clear();
  neopixel.setBrightness(50);
  neopixel.show();

  print_startup_info();

  // Might be a good thing to explore for battery life
  // setCpuFrequencyMhz(80);

  // Setup Wifi & MQTT
  setup_wifi();

  // Sensor Setup
  setup_sensors();

  float averagePercent, averageVoltage;
  std::tie(averagePercent, averageVoltage) = get_battery_fuel();

  float temp, humidity;
  std::tie(temp, humidity) = get_bme680_data();

  pushSensorDataToMQTT(averagePercent, averageVoltage, get_uv_index(), temp, humidity, true);

  // Sleep to preserve battery
  esp_sleep_enable_timer_wakeup(SECONDS_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop()
{
  // Could be a non-battery debug mode
  //  delay(5000);
  //  setup();
}

void setLedColor(uint32_t color)
{
  neopixel.fill(color);
  neopixel.show();
}

void print_startup_info()
{

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // Log ESP Chip information
  Log.notice(F("ESP32 Chip model %s Rev %d" CR), ESP.getChipModel(), ESP.getChipRevision());
  Log.notice(F("This chip has %d cores" CR), ESP.getChipCores());

  Log.notice(F("Total heap %d" CR), ESP.getHeapSize());
  Log.notice(F("Free heap has %d " CR), ESP.getFreeHeap());
  Log.notice(F("Total PSRAM has %d " CR), ESP.getPsramSize());
  Log.notice(F("Free PSRAM %d " CR), ESP.getFreePsram());
  Log.notice(F("SPIRAM %d " CR), esp_spiram_get_size());

  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void i2c_scan()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
  delay(5000);
}

void setup_sensors()
{

  int attempts = 0;
  while (!ltr390.begin() && attempts < 3)
  {
    attempts++;
    Serial.println(F("Couldnt find ltr390!"));
    delay(2000);
  }
  Serial.print(F("Found LTR390 "));
  ltr390.setMode(LTR390_MODE_UVS);
  ltr390.setGain(LTR390_GAIN_18);
  ltr390.setResolution(LTR390_RESOLUTION_20BIT);
  // ltr390.setThresholds(100, 1000);
  // ltr390.configInterrupt(true, LTR390_MODE_UVS);

  attempts = 0;
  while (!maxlipo.begin() && attempts < 3)
  {
    attempts++;
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x"));
  Serial.println(maxlipo.getChipID(), HEX);

  // BME680
  attempts = 0;
  while (!bme.begin() && attempts < 3)
  {
    attempts++;
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    delay(2000);
  }

  Serial.print(F("Found BME680 "));
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

}

void setup_wifi()
{
  Log.notice(F("Connecting to WiFi network: %s (password: %s)" CR), ssid, password);
  WiFi.begin(ssid, password);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(clientId.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10)
  {
    attempts++;
    delay(500);
    Serial.print(".");
    setLedColor(LED_Colors[YELLOW]);
  }

  setLedColor(LED_Colors[OFF]);

  Serial.println("");

  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println("");

  bool success = Ping.ping(mqtt_server, 3);

  if (!success)
  {
    Log.error(F("Ping failed to MQTT server at %s" CR), mqtt_server);
    setLedColor(LED_Colors[RED]);
    delay(1000);
    setLedColor(LED_Colors[OFF]);
    return;
  }
  else
  {
    Log.notice(F("Ping to MQTT server at %s OK" CR), mqtt_server);
  }

  // Configure mqtt
  client.setBufferSize(512);
  // client.setSocketTimeout(10);

  // We only need to publish config once. But bootCount is not always entirely reliable, so we will publish while bootCount is less than 10
  if (bootCount <= 10)
  {
    publishHomeAssistantConfigMessage();
  }
}

std::tuple<float, float> get_bme680_data()
{
  if (!bme.performReading())
  {
    Log.error(F("Failed to perform reading :-(" CR));
    return std::make_tuple(0.0, 0.0);
  }

  float temperature = bme.temperature;
  float humidity = bme.humidity;
  float pressure = bme.pressure / 100.0;
  float gas_resistance = bme.gas_resistance / 1000.0;

  return std::make_tuple(temperature, humidity);
}

std::tuple<float, float> get_battery_fuel()
{
  // First get battery reading
  float totalPercent = 0.0;
  float totalVoltage = 0.0;
  int numReadings = 10;
  for (int i = 0; i < numReadings; i++)
  {
    float _cell_percent = maxlipo.cellPercent();
    float _cell_voltage = maxlipo.cellVoltage();

    // The first few may report as zero, so lets skip them
    if (_cell_percent > 0 && _cell_voltage > 0)
    {
      totalPercent += _cell_percent;
      totalVoltage += _cell_voltage;
    }
    delay(100); // Delay between readings
  }

  float averagePercent = totalPercent / numReadings;
  float averageVoltage = totalVoltage / numReadings;

  return std::make_tuple(averagePercent, averageVoltage);
}

float get_uv_index()
{

  float totalUVS = 0.0;
  int numReadings = 10;
  for (int i = 0; i < numReadings; i++)
  {
    totalUVS += ltr390.readUVS();
    delay(100);
  }
  float averageUVS = totalUVS / numReadings;

  return round(averageUVS / 2300.0 * 100.0) / 100.0;
}

void pushSensorDataToMQTT(float cell_percent, float cell_voltage, float uv_index, float temp, float humidity, bool debug = false)
{
  StaticJsonDocument<512> state_info;
  state_info[UV_INDEX_TOPIC.json_key_name] = uv_index;
  state_info[UV_BATTERY_TOPIC.json_key_name] = cell_percent;
  state_info[UV_BATTERY_VOLT_TOPIC.json_key_name] = cell_voltage;
  state_info[TEMPERATURE_TOPIC.json_key_name] = temp;
  state_info[HUMIDITY_TOPIC.json_key_name] = humidity;

  // Only do this if debug is enabled.
  if (debug)
  {
    float percentFreeHeap = (float)ESP.getFreeHeap() / ESP.getHeapSize() * 100.0;
    state_info["debug-boot-count"] = bootCount;
    state_info["debug-free-heap"] = percentFreeHeap;
    state_info["debug-wake_reason"] = esp_sleep_get_wakeup_cause();
  }

  char stateInfoAsJson[512];
  serializeJson(state_info, stateInfoAsJson);

  // todo condense into a single topic
  publishGenericMessage(UV_INDEX_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(UV_BATTERY_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(UV_BATTERY_VOLT_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(TEMPERATURE_TOPIC.state_topic.c_str(), stateInfoAsJson);
  publishGenericMessage(HUMIDITY_TOPIC.state_topic.c_str(), stateInfoAsJson);
}

void publishGenericMessage(const char *topic, const char *payload)
{

  reconnect();

  Log.info(F("Publishing to topic %s" CR), topic);
  Log.info(F("Payload %s" CR), payload);

  boolean success = client.publish(topic, payload);

  if (success)
  {
    neopixel.fill(LED_Colors[WHITE]);
    neopixel.show();
    delay(1000);
    neopixel.fill(LED_Colors[OFF]);
    neopixel.show();
  }
  else
  {
    setLedColor(LED_Colors[RED]);
    delay(1000);
  }
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
 * Used to connect or reconnect to the MQTT server
 */
void reconnect()
{
  Log.info("Reconnecting.." CR);

  int maxAttempts = 3;
  int attempt = 0;
  while (!client.connected() && attempt < maxAttempts)
  {
    attempt++;

    Log.notice(F("Attempting MQTT connection to %s" CR), mqtt_server);

    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password))
    {
      Log.notice(F("Connected as clientId %s :-)" CR), clientId.c_str());
    }
    else
    {
      neopixel.setBrightness(250);
      setLedColor(LED_Colors[RED]);
      Log.error(F("{failed, rc=%d try again in 5 seconds}" CR), client.state());
      delay(1000);
    }
  }
}