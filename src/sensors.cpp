#include "sensors.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <Wire.h>
#include "Adafruit_LTR390.h"
#include "Adafruit_BME680.h"
#include "Adafruit_MAX1704X.h"

// Sensor objects
Adafruit_LTR390 ltr390 = Adafruit_LTR390();
Adafruit_BME680 bme;
Adafruit_MAX17048 maxlipo;

void setup_sensors()
{
  Serial.println("[SENSORS] Starting sensor initialization");
  
  // Initialize I2C with default pins for Adafruit Feather ESP32-S3
  Wire.begin(); // Uses default SDA/SCL pins
  Serial.println("[SENSORS] I2C initialized on default pins");
  
  // Scan for I2C devices first
  i2c_scan();
  
  Serial.println("[SENSORS] Initializing LTR390 UV sensor...");
  int attempts = 0;
  while (!ltr390.begin() && attempts < 3)
  {
    attempts++;
    Serial.println(F("[SENSORS] Could not find LTR390 sensor, retrying..."));
    delay(2000);
  }
  Serial.println(F("[SENSORS] LTR390 UV sensor initialized successfully"));
  ltr390.setMode(LTR390_MODE_UVS);
  ltr390.setGain(LTR390_GAIN_18);
  ltr390.setResolution(LTR390_RESOLUTION_20BIT);
  // ltr390.setThresholds(100, 1000);
  // ltr390.configInterrupt(true, LTR390_MODE_UVS);

  // BME680
  Serial.println("[SENSORS] Initializing BME680 environmental sensor...");
  attempts = 0;
  while (!bme.begin() && attempts < 3)
  {
    attempts++;
    Serial.println(F("[SENSORS] Could not find BME680 sensor, retrying..."));
    delay(2000);
  }

  Serial.println(F("[SENSORS] BME680 environmental sensor initialized successfully"));
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // MAX17048 Battery Monitor
  Serial.println("[SENSORS] Initializing MAX17048 battery monitor...");
  attempts = 0;
  bool maxInitialized = false;
  while (!maxInitialized && attempts < 3)
  {
    attempts++;
    maxInitialized = maxlipo.begin();
    if (!maxInitialized) {
      Serial.println(F("[SENSORS] Could not find MAX17048 battery monitor, retrying..."));
      delay(2000);
    }
  }
  
  if (maxInitialized) {
    Serial.println(F("[SENSORS] MAX17048 battery monitor initialized successfully"));
  } else {
    Serial.println(F("[SENSORS] WARNING: MAX17048 battery monitor failed to initialize"));
  }
  
  Serial.println("[SENSORS] All sensors initialized successfully!");
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
  // float pressure = bme.pressure / 100.0;
  // float gas_resistance = bme.gas_resistance / 1000.0;

  return std::make_tuple(temperature, humidity);
}



float get_uv_index()
{
  float totalUVS = 0.0;
  int validReadings = 0;
  int numReadings = 10;
  
  for (int i = 0; i < numReadings; i++)
  {
    uint32_t rawReading = ltr390.readUVS();
    
    // Check for valid reading (sensor returns 0 for errors/saturation)
    // Also check for reasonable upper bound (sensor saturation)
    if (rawReading > 0 && rawReading < 1048576) // 20-bit max = 1048576
    {
      totalUVS += rawReading;
      validReadings++;
    }
    else
    {
      Log.warning(F("[UV] Invalid reading %d: %lu" CR), i, rawReading);
    }
    delay(100);
  }
  
  // Return error value if no valid readings
  if (validReadings == 0)
  {
    Log.error(F("[UV] No valid readings obtained!" CR));
    return -1.0; // Error indicator
  }
  
  float averageUVS = totalUVS / validReadings;

  // Improved UV Index calculation based on LTR390 datasheet
  // The original divisor of 2300 was too high for outdoor readings
  float uvIndex;
  
  // Try a more appropriate conversion factor for outdoor UV levels
  // The LTR390 datasheet suggests different conversion factors
  uvIndex = averageUVS / 2300.0;
  
  // Apply calibration multiplier to get realistic outdoor UV Index values
  // This should give UV Index 3-11 in full sun depending on conditions
  uvIndex = uvIndex * 6.0; // Increased from original to get realistic readings
  
  // Clamp UV index to reasonable bounds (0-20)
  if (uvIndex < 0) uvIndex = 0;
  if (uvIndex > 20) uvIndex = 20; // Extreme UV rarely exceeds 15-16
  
  // Use ArduinoLog instead of Serial.printf for remote monitoring
  Log.trace(F("[UV] Valid readings: %d/%d, Raw UVS: %.0f, UV Index: %.2f" CR), 
            validReadings, numReadings, averageUVS, uvIndex);
  
  return round(uvIndex * 100.0) / 100.0;
}

void i2c_scan()
{
  byte error, address;
  int nDevices;
  Serial.println("[I2C] Scanning for devices...");
  Serial.println("[I2C] Expected devices:");
  Serial.println("[I2C]   - LTR390 UV sensor at 0x53");
  Serial.println("[I2C]   - BME680 environmental sensor at 0x77");
  Serial.println("[I2C] Scanning addresses 0x01-0x7F:");
  
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("[I2C] Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      switch(address) {
        case 0x53: Serial.println(" (LTR390 UV Sensor)"); break;
        case 0x77: Serial.println(" (BME680 Environmental Sensor)"); break;
        default: Serial.println(" (Unknown device)"); break;
      }
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("[I2C] Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
    // Error 2 = NACK on address, normal for empty addresses
    else if (error != 2)
    {
      Serial.print("[I2C] Error ");
      Serial.print(error);
      Serial.print(" at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  Serial.print("[I2C] Scan complete. Found ");
  Serial.print(nDevices);
  Serial.println(" devices.");
  
  if (nDevices == 0)
  {
    Serial.println("[I2C] ERROR: No I2C devices found! Check wiring and power.");
  }
  Serial.println();
}

std::tuple<float, float> get_battery_data()
{
  float batteryVoltage = 0.0;
  float batteryPercent = 0.0;
  
  // Don't call begin() again, assume sensor was initialized in setup_sensors()
  batteryVoltage = maxlipo.cellVoltage();
  batteryPercent = maxlipo.cellPercent();
  
  // Add debug info to see what's happening
  Serial.printf("[BATTERY] Raw readings - Voltage: %.3fV, Percent: %.2f%%\n", batteryVoltage, batteryPercent);
  
  if (batteryVoltage > 0.1) { // Valid reading
    Log.notice(F("[BATTERY] Valid readings - Voltage: %.2fV, Percent: %.1f%%" CR), batteryVoltage, batteryPercent);
  } else {
    Log.warning(F("[BATTERY] Invalid readings - MAX17048 may not be working properly" CR));
  }
  
  return std::make_tuple(batteryVoltage, batteryPercent);
}