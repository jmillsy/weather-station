# Weather Station

This is an ESP32 based weather station, housed in a [Stevenson Screen](https://en.wikipedia.org/wiki/Stevenson_screen) shell, designed in CAD and 3D printed on an Ender S1 Pro.

### Weather Underground
This weather station pushes data to Weather Underground at ID:`KTXAUSTI3700` https://www.wunderground.com/dashboard/pws/KTXAUSTI3700

### Project Architecture
![Untitled Diagram drawio](https://github.com/user-attachments/assets/408bb89c-6178-4416-980f-26c8ac9aedcd)

### Pictures
<img width="530" alt="image" src="https://github.com/user-attachments/assets/12a76539-06fa-422e-8e6f-104c22decf1e">

<img width="419" alt="image" src="https://github.com/user-attachments/assets/6b5557c6-7cd5-420a-80db-c88d3c997775">

### Sensors
 LTR390
 BME680
 Feather ESP32-S2

### Battery

3.7V LIPO, 2000mAh / 7.4Wh charged with a 6V Solar Panel. 

Making this battery powered was the most interesting part of the project. Home weather data is not real-time interesting, so we only need to emit data infrequently. In most ESP32 style apps, you will loop with a delay, but this keeps the CPU running, and burning power. ESP32 makes deep and light sleep easily available, although it makes debugging more challening. With deep sleep the LEDs, wifi, serial connections, all are disabled. For this I relied on the onboard neopixel during operation to understand the state  instead of relying on serial debugging. 

### MQTT
This project is also easily done using ESPHome, but I wanted to dig deeper into a generic MQTT client, so I can have more control, and learn more on the way. 





