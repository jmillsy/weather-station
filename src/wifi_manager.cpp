#include "wifi_manager.h"
#include "config.h"
#include <ArduinoLog.h>
#include <ESP32Ping.h>
#include "ha_integration.h"

// External dependencies
extern String clientId; // from mqtt_client.cpp
extern int bootCount;

void setup_wifi()
{
  Serial.println("[WiFi] Starting WiFi manager setup");
  Serial.print("[WiFi] Config check - SSID: ");
  Serial.println(ssid ? ssid : "NULL");
  Serial.print("[WiFi] Config check - Password: ");
  Serial.println(password ? "[SET]" : "NULL");
  
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
  }

  Serial.println("");

  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println("");

  bool success = Ping.ping(mqtt_server, 3);

  if (!success)
  {
    Log.error(F("Ping failed to MQTT server at %s" CR), mqtt_server);
    delay(1000);
    return;
  }
  else
  {
    Log.notice(F("Ping to MQTT server at %s OK" CR), mqtt_server);
  }

  
}

bool is_wifi_connected()
{
  return WiFi.status() == WL_CONNECTED;
}

String get_wifi_ip()
{
  return WiFi.localIP().toString();
}