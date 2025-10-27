#!/bin/bash

# Weather Station Deploy Script
# This script helps with common deployment and maintenance tasks

# Configuration - Update these values for your setup
MQTT_BROKER="192.168.1.168"  # Your MQTT broker IP
DEVICE_NAME="weather-station"  # Your device name from .env
MQTT_PORT="1883"  # MQTT port (usually 1883)
MQTT_USERNAME=""  # MQTT username if required
MQTT_PASSWORD=""  # MQTT password if required

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Weather Station Deploy Script${NC}"
echo "=============================="

# Function to build the project
build_project() {
    echo -e "${YELLOW}Building project...${NC}"
    platformio run
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Build successful${NC}"
    else
        echo -e "${RED}✗ Build failed${NC}"
        exit 1
    fi
}

# Function to deploy (build and upload via OTA)
deploy() {
    echo -e "${YELLOW}Starting deploy process via OTA...${NC}"
    echo -e "${BLUE}Automatically pausing device for OTA upload...${NC}"
    
    # Pause the device first
    echo -e "${YELLOW}Sending sleep pause command (retained)...${NC}"
    
    # Build mosquitto_pub command
    MOSQUITTO_CMD="mosquitto_pub -h $MQTT_BROKER -p $MQTT_PORT -t \"$DEVICE_NAME/pause\" -m \"pause\" -r"
    
    # Add authentication if provided
    if [ ! -z "$MQTT_USERNAME" ] && [ ! -z "$MQTT_PASSWORD" ]; then
        MOSQUITTO_CMD="$MOSQUITTO_CMD -u $MQTT_USERNAME -P $MQTT_PASSWORD"
    fi
    
    echo "Executing: $MOSQUITTO_CMD"
    eval $MOSQUITTO_CMD
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Pause command sent successfully${NC}"
        echo -e "${GREEN}  The device will pause for 5 minutes on next wake${NC}"
        echo -e "${BLUE}  This message is retained - it will trigger on the next device wake${NC}"
    else
        echo -e "${RED}✗ Failed to send pause command${NC}"
        echo "  Make sure mosquitto-clients is installed: brew install mosquitto"
        return 1
    fi
    
    echo -e "${YELLOW}Waiting for device to process pause message and clear topic...${NC}"
    
    # Wait until the retained message is cleared (indicating device processed it)
    local elapsed=0
    local update_interval=10
    local last_update=0
    
    while true; do
        # Check if the topic is empty (message cleared)
        local topic_status=$(mosquitto_sub -h $MQTT_BROKER -p $MQTT_PORT ${MQTT_USERNAME:+-u "$MQTT_USERNAME"} ${MQTT_PASSWORD:+-P "$MQTT_PASSWORD"} -t "$DEVICE_NAME/pause" -W 1 -C 1 2>/dev/null || echo "")
        
        if [ -z "$topic_status" ] || [ "$topic_status" = "" ]; then
            echo -e "${GREEN}✓ Device has processed pause message and cleared topic${NC}"
            break
        fi
        
        # Show status update every 10 seconds
        if [ $((elapsed - last_update)) -ge $update_interval ]; then
            echo -e "${BLUE}  Still waiting for message to clear... (${elapsed}s elapsed)${NC}"
            last_update=$elapsed
        fi
        
        sleep 2
        elapsed=$((elapsed + 2))
    done
    
    # Build first
    build_project
    if [ $? -ne 0 ]; then
        return 1
    fi
    
    echo -e "${YELLOW}Looking for OTA device...${NC}"
    echo -e "${BLUE}Device should appear as: $DEVICE_NAME.local${NC}"
    
    # Upload via OTA - PlatformIO will auto-detect OTA protocol and prompt for password
    platformio run --target upload --upload-port $DEVICE_NAME.local
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ OTA upload successful${NC}"
        echo -e "${GREEN}  Device will restart with new firmware${NC}"
    else
        echo -e "${RED}✗ OTA upload failed${NC}"
        echo -e "${YELLOW}  Troubleshooting:${NC}"
        echo -e "${YELLOW}    1. Check device was successfully paused (should show in logs above)${NC}"
        echo -e "${YELLOW}    2. Verify device and computer are on same network${NC}"
        echo -e "${YELLOW}    3. Test hostname resolution: ping $DEVICE_NAME.local${NC}"
        echo -e "${YELLOW}    4. Check if device is running OTA-enabled firmware${NC}"
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  deploy      - Build and upload firmware via OTA"
    echo "  help        - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 deploy     # Build and upload new firmware via OTA"
}

# Main script logic
case "$1" in
    "deploy")
        deploy
        ;;
    "help"|"--help"|"-h")
        show_usage
        ;;
    "")
        echo -e "${YELLOW}No command specified.${NC}"
        show_usage
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        show_usage
        exit 1
        ;;
esac