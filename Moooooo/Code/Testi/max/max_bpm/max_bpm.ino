#include <Wire.h>
#include "MAX30105.h"


MAX30105 particleSensor;


void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }


  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running

}

void loop() {
  long irValue = particleSensor.getIR();

  Serial.print("IR=");
  Serial.print(irValue);
  
  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}