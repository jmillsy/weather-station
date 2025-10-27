#include "mqtt_client.h"
#include <Arduino.h>
#include <ArduinoLog.h>

// MQTT Client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Static storage for MQTT configuration
static String stored_server;
static int stored_port;
static String stored_username;
static String stored_password;

// Prefix for the MQTT Client Identification
String clientId = "esp32-client-";

// Global variable to track pause requests
static bool pause_requested = false;

// MQTT callback function to handle incoming messages
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Log.notice(F("[MQTT] CALLBACK TRIGGERED - Topic: %s, Length: %d" CR), topic, length);
  
  // Convert payload to string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Log.notice(F("[MQTT] Received message on topic %s: '%s'" CR), topic, message.c_str());
  
  // Check if this is a pause request
  if (message.equalsIgnoreCase("pause") || message.equalsIgnoreCase("1") || message.equalsIgnoreCase("true")) {
    pause_requested = true;
    Log.notice(F("[MQTT] Sleep pause requested! Setting pause_requested = true" CR));
  } else {
    Log.notice(F("[MQTT] Message '%s' does not match pause criteria" CR), message.c_str());
  }
}



void setup_mqtt(const char *server, int port, const char *username, const char *password)
{
  Serial.println("[MQTT] Starting MQTT client setup");
  Serial.print("[MQTT] Received params - Server: ");
  Serial.println(server ? server : "NULL");
  Serial.print("[MQTT] Received params - Port: ");
  Serial.println(port);
  Serial.print("[MQTT] Received params - Username: ");
  Serial.println(username ? username : "NULL");
  
  // Store configuration with proper fallbacks
  stored_server = String(server && strlen(server) > 0 ? server : "192.168.1.168");
  stored_port = (port > 0) ? port : 1883;
  stored_username = String(username && strlen(username) > 0 ? username : "");
  stored_password = String(password && strlen(password) > 0 ? password : "");
  
  // Debug logging
  Log.notice(F("MQTT Setup - Server: %s, Port: %d" CR), stored_server.c_str(), stored_port);
  Log.notice(F("MQTT Setup - Username: %s" CR), stored_username.c_str());
  
  // Generate unique client ID
  clientId += String(random(0xffff), HEX);
  
  // Configure MQTT client
  client.setServer(stored_server.c_str(), stored_port);
  client.setBufferSize(512);
  client.setCallback(mqtt_callback);
  // client.setSocketTimeout(10);
  
  Log.notice(F("MQTT client configured with ID: %s" CR), clientId.c_str());
}

void reconnect()
{
  Log.info("Reconnecting.." CR);
  Log.notice(F("Stored MQTT config - Server: %s, Port: %d" CR), stored_server.c_str(), stored_port);

  int maxAttempts = 3;
  int attempt = 0;
  while (!client.connected() && attempt < maxAttempts)
  {
    attempt++;

    Log.notice(F("Attempting MQTT connection to %s:%d" CR), stored_server.c_str(), stored_port);

    // Attempt to connect
    if (client.connect(clientId.c_str(), stored_username.c_str(), stored_password.c_str()))
    {
      Log.notice(F("Connected as clientId %s :-)" CR), clientId.c_str());
    }
    else
    {
      Log.error(F("{failed, rc=%d try again in 5 seconds}" CR), client.state());
      delay(1000);
    }
  }
}

bool check_for_sleep_pause(const char *device_name) {
  // Reset pause flag
  pause_requested = false;
  
  Log.notice(F("[MQTT] Starting pause check for device: %s" CR), device_name);
  
  // Ensure we're connected to MQTT
  if (!client.connected()) {
    Log.notice(F("[MQTT] Not connected, attempting reconnect..." CR));
    reconnect();
  }
  
  if (!client.connected()) {
    Log.error(F("[MQTT] Cannot connect to check for pause requests" CR));
    return false;
  }
  
  Log.notice(F("[MQTT] MQTT client connected successfully" CR));
  
  // Subscribe to pause topic
  String pause_topic = String(device_name) + "/pause";
  Log.notice(F("[MQTT] Subscribing to pause topic: %s" CR), pause_topic.c_str());
  
  if (client.subscribe(pause_topic.c_str())) {
    Log.notice(F("[MQTT] Successfully subscribed to pause topic" CR));
    
    // Wait for messages - retained messages should be delivered immediately
    // but let's give it more time and add detailed logging
    unsigned long start_time = millis();
    int loop_count = 0;
    while (millis() - start_time < 2000) {  // Increased to 2 seconds for debugging
      client.loop(); // Process incoming messages
      loop_count++;
      delay(50);  // Increased delay for better debugging
      
      if (pause_requested) {
        Log.notice(F("[MQTT] Pause request detected after %d loops!" CR), loop_count);
        
        // Clear the retained message so it doesn't trigger again
        clear_retained_message(pause_topic.c_str());
        
        // Unsubscribe from topic
        client.unsubscribe(pause_topic.c_str());
        return true;
      }
      
      // Log every 500ms during wait
      if ((millis() - start_time) % 500 < 50) {
        Log.notice(F("[MQTT] Waiting for retained message... (%lu ms elapsed)" CR), millis() - start_time);
      }
    }
    
    // Unsubscribe from topic
    client.unsubscribe(pause_topic.c_str());
    Log.notice(F("[MQTT] No pause request received after %d loops and %lu ms" CR), loop_count, millis() - start_time);
  } else {
    Log.error(F("[MQTT] Failed to subscribe to pause topic" CR));
  }
  
  return false;
}

void clear_retained_message(const char *topic) {
  // Publish an empty retained message to clear the retained message
  Log.notice(F("[MQTT] Clearing retained message on topic: %s" CR), topic);
  
  if (client.connected()) {
    // Publishing an empty message with retain=true clears the retained message
    client.publish(topic, "", true);
  }
}

void publishGenericMessage(const char *topic, const char *payload)
{
  reconnect();

  Log.info(F("Publishing to topic %s" CR), topic);
  Log.info(F("Payload %s" CR), payload);

  boolean success = client.publish(topic, payload);

  if (success)
  {
    Serial.println(F("[MQTT] Connected to MQTT broker successfully"));
  }
  else
  {
    delay(1000);
  }
}

void publishGenericMessage(const char *topic, const char *payload, bool retain)
{
  reconnect();

  Log.info(F("Publishing to topic %s (retain=%s)" CR), topic, retain ? "true" : "false");
  Log.info(F("Payload %s" CR), payload);

  boolean success = client.publish(topic, payload, retain);

  if (success)
  {
    Serial.println(F("[MQTT] Message published successfully"));
  }
  else
  {
    delay(1000);
  }
}

bool is_mqtt_connected()
{
  return client.connected();
}