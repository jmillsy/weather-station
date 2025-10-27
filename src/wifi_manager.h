#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <WiFiUdp.h>

// Function declarations
void setup_wifi();
bool is_wifi_connected();
String get_wifi_ip();

#endif // WIFI_MANAGER_H