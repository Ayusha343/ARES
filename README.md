# ARES — Animals on Roads — Enhancement of Safety

A short summary and pointers for the ARES project (IoT-based road-animal safety system).

- **Documentation:** [Project Documentation](Docs/ARES_Project_Documentation_Final.md)
- **Final firmware:** Code/Final_7.0/Final_7.0.ino
- **Hardware summary:** Hardware.md

## Quick Start

1. Open `Code/Final_7.0/Final_7.0.ino` in the Arduino IDE or PlatformIO.
2. Select board: WeMos D1 Mini (ESP8266). Use 3.3V operating voltage and appropriate flash settings for ESP8266.
3. Install required libraries (I2C, ThingSpeak, sensor drivers for MPU6050 and MAX30105) via Library Manager.
4. Configure WiFi credentials and ThingSpeak keys in the sketch, then compile and upload.

## Useful Links

- Full project documentation: [Docs/ARES_Project_Documentation_Final.pdf](Docs/ARES_Project_Documentation_Final.pdf)
- Firmware source: `Code/Final_7.0/Final_7.0.ino`

## Contributing

Please open an issue or pull request for improvements. For firmware changes, include build instructions and list library versions used.

---
Generated/updated by project maintainer.
