#ifndef CONFIG_H
#define CONFIG_H

#define ONBOARD_LED 21  // Xiao ESP32-S3 built-in LED
#define LTR390_I2C_ADDRESS 0x53

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define SECONDS_TO_SLEEP 60      /* Time ESP32 will go to sleep (in seconds) */
#define PAUSE_DURATION_MINUTES 5  /* Time to pause when pause is requested (in minutes) */

#define ST(A) #A
#define STR(A) ST(A)

// External declarations - definitions are in main.cpp
extern const char *ssid;
extern const char *password;
extern const char *mqtt_server;
extern const int mqtt_port;
extern const char *mqtt_username;
extern const char *mqtt_password;
extern const char *device_name;

#ifndef BME680_SENSOR_I2C_ADDRESS
#define BME680_SENSOR_I2C_ADDRESS   0x77
#endif

#endif // CONFIG_H