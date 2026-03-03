int batteryPin = A0;  
float fullVoltage = 4.2;
float emptyVoltage = 3.0;
float R1 = 12000;//11910
float R2 = 2700;//2640
int charge = D7;
int batt;

void setup()
{
  pinMode(charge,OUTPUT);
  pinMode(A0,INPUT);
  Serial.begin(9600);
}
void loop()
{
  digitalWrite(charge,HIGH);
}
void battery()
{
  int rawValue = analogRead(batteryPin);
  // Convert the raw value to voltage (adjust based on your voltage divider)
  float voltage = (rawValue * 3.3 )/ 1024.0;
  
  float Vin = (voltage * (R1 + R2) / R2)-0.28;
  batt = (Vin/fullVoltage)*100;

  Serial.print("Battery Percentage: ");
  Serial.print(batt);
  Serial.println("%");
  Serial.print("Battery Voltage: ");
  Serial.println(Vin);
}
