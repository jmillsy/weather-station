#ifndef CONFIG_H
#define CONFIG_H

#define ONBOARD_LED 2
#define LTR390_I2C_ADDRESS 0x53


#define ST(A) #A
#define STR(A) ST(A)

#ifdef WIFI_SSID
const char *ssid = STR(WIFI_SSID);
#endif

#ifdef WIFI_PASSWORD
const char *password = STR(WIFI_PASSWORD);
#endif

#ifdef MQTT_SERVER
const char *mqtt_server = STR(MQTT_SERVER);
#endif

#ifdef MQTT_PORT
const int mqtt_port = atoi(STR(MQTT_PORT));
#endif

#ifdef MQTT_USERNAME
const char *mqtt_username = STR(MQTT_USERNAME);
#endif

#ifdef MQTT_PASSWORD
const char *mqtt_password = STR(MQTT_PASSWORD);
#endif

#ifdef DEVICE_NAME
const char *device_name = STR(DEVICE_NAME);
#endif

#ifndef BME680_SENSOR_I2C_ADDRESS
#define BME680_SENSOR_I2C_ADDRESS   0x77
#endif

#define ONBOARD_LED 2

#endif // CONFIG_H