# Hardware — ARES

This file summarizes the core hardware components used in the ARES prototype and points to the full BOM in the project documentation.

Core components

- Microcontroller: WeMos D1 Mini (ESP8266)
- IMU: MPU6050 (GY-521) — accelerometer + gyroscope + temperature
- Pulse oximeter / HR sensor: MAX30105 (or compatible MAX3010x breakout)
- GPS: GY-NEO-6MV2 (u-blox NEO-6M)
- Battery: 3.8V Li-Po (≈3.6 Ah used in prototype)
- Charger: TP4056 charging module
- Vibration motor, LDR (light sensor), LEDs for visibility
- Voltage divider for battery sense (R1 = 12kΩ, R2 = 2.7kΩ in prototype)

Power & connectivity notes

- The prototype uses a single Li-Po cell and a TP4056 charger. A buck or boost regulator can be added depending on peripheral voltage requirements.
- I²C bus connects MPU6050 and MAX30105 to the WeMos D1 Mini on SDA/SCL pins (D2/D1).

Where to find full BOM and photos

See the complete Bill of Materials, casing notes and prototype photos in: [Docs/ARES_Project_Documentation_Final.pdf](Docs/ARES_Project_Documentation_Final.pdf)
