#include <Arduino.h>
#include <ArduinoLog.h>
#include <ESP32Ping.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "time.h"
#include "config.h"
#include "sensors.h"
#include "mqtt_client.h"
#include "wifi_manager.h"
#include "ha_integration.h"

// External MQTT client reference
extern PubSubClient client;

// Define config variables
#ifdef WIFI_SSID
const char *ssid = STR(WIFI_SSID);
#else
const char *ssid = "your_wifi_ssid";
#endif

#ifdef WIFI_PASSWORD
const char *password = STR(WIFI_PASSWORD);
#else
const char *password = "your_wifi_password";
#endif

#ifdef MQTT_SERVER
const char *mqtt_server = STR(MQTT_SERVER);
#else
const char *mqtt_server = "192.168.1.168";
#endif

#ifdef MQTT_PORT
const int mqtt_port = atoi(STR(MQTT_PORT));
#else
const int mqtt_port = 1883;
#endif

#ifdef MQTT_USERNAME
const char *mqtt_username = STR(MQTT_USERNAME);
#else
const char *mqtt_username = "";
#endif

#ifdef MQTT_PASSWORD
const char *mqtt_password = STR(MQTT_PASSWORD);
#else
const char *mqtt_password = "";
#endif

#ifdef DEVICE_NAME
const char *device_name = STR(DEVICE_NAME);
#else
const char *device_name = "weather-station";
#endif

// Declare the custom functions
void print_startup_info();
void setup_ota();

RTC_DATA_ATTR int bootCount = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  // Early debug output before Log system
  Serial.println("\n\n=== WEATHER STATION BOOT ===");
  Serial.print("Boot count: ");
  Serial.println(bootCount);
  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());
  
  // Debug config values immediately
  Serial.println("--- CONFIG VALUES ---");
  Serial.print("SSID: ");
  Serial.println(ssid ? ssid : "NULL");
  Serial.print("MQTT Server: ");
  Serial.println(mqtt_server ? mqtt_server : "NULL");
  Serial.print("MQTT Port: ");
  Serial.println(mqtt_port);
  Serial.print("MQTT Username: ");
  Serial.println(mqtt_username ? mqtt_username : "NULL");
  Serial.println("====================\n");
  
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  print_startup_info();

  // Might be a good thing to explore for battery life
  // setCpuFrequencyMhz(80);

  // Setup Wifi & MQTT
  Log.notice(F("Starting WiFi setup with SSID: %s" CR), ssid);
  setup_wifi();
  
  Log.notice(F("Starting MQTT setup - Server: %s, Port: %d" CR), mqtt_server, mqtt_port);
  setup_mqtt(mqtt_server, mqtt_port, mqtt_username, mqtt_password);

  // Sensor Setup
  setup_sensors();

  float temp, humidity;
  std::tie(temp, humidity) = get_bme680_data();

  publishHomeAssistantConfigMessage();
  pushSensorDataToMQTT(temp, humidity, get_uv_index(), true);

  // Check for sleep pause request
  Log.notice(F("Checking for sleep pause requests..." CR));
  if (check_for_sleep_pause(device_name)) {
    Log.notice(F("Sleep pause requested! Staying awake for %d minutes" CR), PAUSE_DURATION_MINUTES);
    
    // Setup OTA for firmware updates during pause
    setup_ota();
    
    // Stay awake for the pause duration
    unsigned long pause_start = millis();
    unsigned long pause_duration_ms = PAUSE_DURATION_MINUTES * 60 * 1000;
    
    while (millis() - pause_start < pause_duration_ms) {
      // Handle OTA updates
      ArduinoOTA.handle();
      
      // Keep MQTT connection alive and process messages
      if (client.connected()) {
        client.loop();
      }
      delay(1000);
      
      // Print status every 30 seconds
      if ((millis() - pause_start) % 30000 < 1000) {
        unsigned long remaining_ms = pause_duration_ms - (millis() - pause_start);
        Log.notice(F("Pause active - %lu seconds remaining" CR), remaining_ms / 1000);
      }
    }
    
    Log.notice(F("Pause period completed, proceeding to sleep" CR));
  }

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
  //Log.notice(F("SPIRAM %d " CR), esp_spiram_get_size());

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

void setup_ota() {
  Log.notice(F("Setting up Arduino OTA..." CR));
  
  // Set OTA hostname (use device name)
  ArduinoOTA.setHostname(device_name);
  
  // OTA callbacks for logging
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Log.notice(F("OTA: Start updating %s" CR), type.c_str());
  });
  
  ArduinoOTA.onEnd([]() {
    Log.notice(F("OTA: Update complete!" CR));
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int last_percent = 0;
    unsigned int percent = (progress / (total / 100));
    if (percent != last_percent && percent % 10 == 0) {
      Log.notice(F("OTA Progress: %u%%" CR), percent);
      last_percent = percent;
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Log.error(F("OTA Error[%u]: " CR), error);
    if (error == OTA_AUTH_ERROR) {
      Log.error(F("Auth Failed" CR));
    } else if (error == OTA_BEGIN_ERROR) {
      Log.error(F("Begin Failed" CR));
    } else if (error == OTA_CONNECT_ERROR) {
      Log.error(F("Connect Failed" CR));
    } else if (error == OTA_RECEIVE_ERROR) {
      Log.error(F("Receive Failed" CR));
    } else if (error == OTA_END_ERROR) {
      Log.error(F("End Failed" CR));
    }
  });
  
  ArduinoOTA.begin();
  Log.notice(F("OTA ready! Hostname: %s" CR), device_name);
  Log.notice(F("OTA IP address: %s" CR), WiFi.localIP().toString().c_str());
}
