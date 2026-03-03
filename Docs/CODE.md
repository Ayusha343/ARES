# CODE — ARES

This file explains how to build and upload the ARES firmware.

Location

- Final firmware sketch: Code/Final_7.0/Final_7.0.ino

Build hints

- Board: WeMos D1 Mini (ESP8266). In Arduino IDE, select "NodeMCU 1.0 (ESP-12E Module)" or the specific WeMos D1 Mini entry if available.
- Flash settings: default 4M/1M or leave as IDE defaults for WeMos D1 Mini.
- Libraries: install via Library Manager the sensor and network libraries required by the sketch (common ones include ThingSpeak, Adafruit_Sensor, Adafruit_MPU6050 or another MPU6050 driver, and a MAX3010x driver). Check include lines at the top of the sketch for exact names.

Upload steps (Arduino IDE)

1. Install required libraries.
2. Open `Code/Final_7.0/Final_7.0.ino`.
3. Edit WiFi credentials and ThingSpeak `channel ID`/`write API key` in the top of the sketch.
4. Select the correct COM port and board, then click Upload.

PlatformIO

- Create a simple `platformio` project for the `espressif8266` platform and set board to `d1_mini`.
- Copy the sketch into the `src/` folder and add library dependencies to `platformio.ini`.

Notes

- The sketch contains placeholder/randomized BPM and SpO2 values used for demonstrations; replace these with live sensor reads for production (see documentation lines referenced in project docs).
- Confirm sensor I²C addresses and pin mappings before first power-up.
