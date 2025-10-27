# Sleep Pause Feature

The weather station now includes a sleep pause feature that allows you to temporarily prevent the device from going into deep sleep mode. This is useful for performing maintenance, OTA updates, or debugging.

## How It Works

1. **On Wake**: After collecting sensor data and publishing to MQTT, the device checks for pause requests
2. **MQTT Topic**: The device subscribes to `{device_name}/pause` topic briefly (1 second)
3. **Retained Messages**: Uses MQTT retained messages so you can send the pause command at any time
4. **Pause Trigger**: Any of these messages will trigger a pause:
   - `pause`
   - `1` 
   - `true`
5. **Pause Duration**: When triggered, the device stays awake for 5 minutes (configurable)
6. **Status Updates**: Every 30 seconds, the device logs remaining pause time
7. **Auto-Clear**: The retained message is automatically cleared after processing

## Usage Examples

### Using the Deploy Script (Recommended):
```bash
# Pause the device (retained message - triggers on next wake)
./deploy.sh pause

# Clear the pause command (resume normal operation)
./deploy.sh clear

# Build and upload new firmware
./deploy.sh deploy

# Show all available commands
./deploy.sh help
```

### Using mosquitto_pub command line:
```bash
# Pause the weather station for 5 minutes (RETAINED MESSAGE)
mosquitto_pub -h your_mqtt_broker -t "weather-station/pause" -m "pause" -r

# Clear the pause (send empty retained message)
mosquitto_pub -h your_mqtt_broker -t "weather-station/pause" -n -r

# Alternative trigger messages:
mosquitto_pub -h your_mqtt_broker -t "weather-station/pause" -m "1" -r
mosquitto_pub -h your_mqtt_broker -t "weather-station/pause" -m "true" -r
```

### Using Home Assistant:
```yaml
# Add to configuration.yaml
mqtt:
  publish:
    service: mqtt.publish
    data:
      topic: "weather-station/pause"
      payload: "pause"
```

### Using Node-RED:
Create an inject node that publishes to `weather-station/pause` with payload `pause`.

## Configuration

- **Pause Duration**: Configured in `config.h` as `PAUSE_DURATION_MINUTES` (default: 5 minutes)
- **Device Name**: The MQTT topic uses the device name from your `.env` file (`DEVICE_NAME`)
- **Check Timeout**: The device waits only 200ms for pause messages (retained messages are delivered immediately)
- **MQTT Broker**: Configure your broker settings in `deploy.sh` for easy pause commands

## Monitoring

During the pause period, you'll see serial output like:
```
[MQTT] Checking for sleep pause requests...
[MQTT] Subscribing to pause topic: weather-station/pause
[MQTT] Pause request detected!
Sleep pause requested! Staying awake for 5 minutes
Pause active - 270 seconds remaining
Pause active - 240 seconds remaining
...
Pause period completed, proceeding to sleep
```

## Technical Details

- The pause check happens after sensor readings and MQTT publishing
- During pause, the MQTT connection is kept alive with `client.loop()`
- The device will still go to sleep after the pause period expires
- If no pause message is received within 3 seconds, normal sleep occurs
- The pause state is not persistent across resets

## Future Use Cases

This feature is designed to support:
- OTA (Over-The-Air) firmware updates
- Remote debugging sessions  
- Extended data collection periods
- Maintenance mode for configuration changes