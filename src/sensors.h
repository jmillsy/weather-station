#ifndef SENSORS_H
#define SENSORS_H

#include <tuple>

// Function declarations
void setup_sensors();
std::tuple<float, float> get_bme680_data();
float get_uv_index();
void i2c_scan();
std::tuple<float, float> get_battery_data();

#endif // SENSORS_H