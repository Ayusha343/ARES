# FIRMWARE — ARES (Final notes)

Final sketch: `Code/Final_7.0/Final_7.0.ino`

Configuration

- WiFi: locate and set `WIFI_SSID` and `WIFI_PASSWORD` (or equivalent variables) in the sketch.
- ThingSpeak: set `channel ID` and `write API key` before uploading.
- Pin mapping: review the top of the sketch for pin assignments (LEDs, vibro motor, LDR, battery sense pin A0).

Behavioral notes

- Accident detection uses delta-acceleration and gyroscope thresholds (prototype values: ΔAc ≈ 15 m/s², Gyro threshold ≈ 4.3 rad/s).
- The sketch currently contains randomized placeholder values for BPM and SpO2 when a live MAX30105 is not available; replace those lines to enable real readings.

Troubleshooting

- If ThingSpeak uploads fail, verify WiFi connectivity and ThingSpeak rate limits (the sketch uses a small delay to respect rate limits).
- If sensors fail to initialize, check I²C wiring and addresses and ensure pull-ups are present on SDA/SCL.

Reference

- Full project documentation: [Docs/ARES_Project_Documentation_Final.pdf](Docs/ARES_Project_Documentation_Final.pdf)
