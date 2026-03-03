#include <Wire.h>

#include <MAX30105.h>
MAX30105 particleSensor;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == 0)
  { 
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
 // particleSensor.setup(0); //Turn off LEDs

  particleSensor.enableDIETEMPRDY(); 
}
void loop()
{
  float temperature = particleSensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);

  float temperatureF = particleSensor.readTemperatureF();

  Serial.print(" temperatureF=");
  Serial.print(temperatureF, 4);

  Serial.println();
}