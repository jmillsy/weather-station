#include "sensors.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <Wire.h>
#include "Adafruit_LTR390.h"
#include "Adafruit_BME680.h"

// Sensor objects
Adafruit_LTR390 ltr390 = Adafruit_LTR390();
Adafruit_BME680 bme;

void setup_sensors()
{
  Serial.println("[SENSORS] Starting sensor initialization");
  
  // Initialize I2C with proper pins for Xiao ESP32-S3
  Wire.begin(5, 6); // SDA=GPIO5, SCL=GPIO6 for Xiao ESP32-S3
  Serial.println("[SENSORS] I2C initialized on pins SDA=5, SCL=6");
  
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
  int numReadings = 10;
  for (int i = 0; i < numReadings; i++)
  {
    totalUVS += ltr390.readUVS();
    delay(100);
  }
  float averageUVS = totalUVS / numReadings;

  return round(averageUVS / 2300.0 * 100.0) / 100.0;
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