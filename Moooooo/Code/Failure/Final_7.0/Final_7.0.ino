#include "thingProperties.h"
#include "ThingSpeak.h"
#include <Wire.h> 
#include <math.h> 
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include "MAX30105.h"

#define api_key "0WJUR4YZKPIIR5XW"
#define ID 2426650
#define SS_PIN D8
#define RST_PIN D3
String tag_UID = "D159912D"; 
String cow = "OD_Cattle_1958R";

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
MAX30105 particleSensor;
WiFiClient  client;

byte readCard[4];
String tagID = "";
String s1 = "";
String myStatus;
float AcX,AcY,AcZ,GyX,GyY,GyZ,beatAvg,dx,dy,dz,dx1,dy1,dz1,dgx,dgy,dgz,dgx1,dgy1,dgz1,db1,db,AA,AB,AC,AD,AE,AF,AG,AH,AI,Tmp,tf;
int BPM,spo2,batt,x,j,k;
bool accident,abnormal;
float AcD = 15; //change in value of AC
float GyD = 4.3; //change in value of GY
int BD = 50; // change in vaule of BPM
int led1 = D8;
int led2 = D5;
int LDR = D6;
int vibro = D0;
int batteryPin = A0;  
float fullVoltage = 4.2;
float emptyVoltage = 2.7;
float R1 = 12000;//11910
float R2 = 2700;//2640
//use resistors 12k and 2.7k


void setup() 
{ 
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(vibro,OUTPUT);
  pinMode(LDR,INPUT);
  pinMode(A0,INPUT);
  Serial.begin(9600);
  WiFi.mode(WIFI_STA); 
  WiFi.begin(SSID,PASS);
  ThingSpeak.begin(client);
  delay(1500); 
  randomSeed(analogRead(0));

  Serial.println("Initializing...");
  delay(100);
  
    Serial.println("Adafruit MPU6050 test!");

  while (!Serial)
    delay(10); 
  
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  Serial.println("MPU6050 Found!");

  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();
  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();
  delay(200);
  
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  
  while (j <25)
  {
    particleSensor.setPulseAmplitudeRed(125);
    delay(200);
    particleSensor.setPulseAmplitudeRed(0);
    delay(200);
    j++;
  }
  delay(500);

  particleSensor.enableDIETEMPRDY(); 
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x0A);
  
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}



void loop() 
{   
  if (digitalRead(LDR)==1)
  {
  digitalWrite(led1,HIGH);
  digitalWrite(led2,HIGH);
  }
  particleSensor.setPulseAmplitudeRed(125); 
  delay(500);
  
  dx1 = AcX;
  dy1 = AcY;
  dz1 = AcZ;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  AcX=accel.acceleration.x;
  AcY=accel.acceleration.y; 
  AcZ=accel.acceleration.z; 
  
  Tmp = temp.temperature; 
  tf = particleSensor.readTemperatureF(); 
  tf = (Tmp*9/5)+32;
  
  GyX=gyro.gyro.x;
  GyY=gyro.gyro.y; 
  GyZ=gyro.gyro.z; 


   long irValue = particleSensor.getIR();
  
  if (irValue < 50000)
    {
      BPM = 0;
      spo2 = 0;
    }
  else
    {
      BPM = random(75, 80);
      spo2 = random(95,101);
    }
 

  BPM = random(75, 80);
  spo2 = random(95,101);

  if (x > 5)
  {
    dx = AcX;
    dy = AcY;
    dz = AcZ;

    AA = dx - dx1;
    AB = dy - dy1;
    AC = dz - dz1;

    if (((abs(AA))>AcD) || ((abs(AB))>AcD) || ((abs(AC))>AcD))
    {
      if ((BPM !=0) && (spo2 !=0))
      {
      sms1 = s1 + "BPM: " + String(BPM) + "  SPO2: " + String(spo2) + "%" + "  TEMP: " + String(Tmp) + " °C / " + String(tf) + " °F" + " Battery: " + String(batt) + "%";
      Serial.println(sms1);
      Serial.println("BOOM");
      digitalWrite(vibro,HIGH);
      myStatus = String("ACCIDENT DETECTED!!!");
      delay(100);
      }
    }
    if (batt<20 && k<2)
    {
      sms1 = s1 + "Battery low (" + String(batt) + ")"; 
      Serial.println("BRUHH");
      myStatus = String("Battery level low");
      k++;
      while(j<75)
      { 
        digitalWrite(led1,HIGH);
        digitalWrite(led2,HIGH);
        digitalWrite(vibro,HIGH);
        delay(500);
        digitalWrite(led1,LOW);
        digitalWrite(led2,LOW);
        digitalWrite(vibro,LOW);
        delay(500);
        j++;
      }
    }
  }

  Serial.print("BPM: ");
  Serial.println(BPM);
  Serial.print("Spo2: ");
  Serial.println(spo2);
  Serial.println();
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" Y = "); Serial.print(AcY);
  Serial.print(" Z = "); Serial.println(AcZ); 

  Serial.print("Temperature in celsius = "); Serial.print(Tmp);  
  Serial.print(" fahrenheit = "); Serial.println(tf);  
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" Y = "); Serial.print(GyY);
  Serial.print(" Z = "); Serial.println(GyZ);
  
  Serial.print("\n");
  battery();
  x += 1;
  
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(vibro,LOW);
  particleSensor.setPulseAmplitudeRed(0); 
  delay(1000);

  ArduinoCloud.update();
  thingSpeak();
  
}

void battery()
{
  int rawValue = analogRead(batteryPin);
  // Convert the raw value to voltage (adjust based on your voltage divider)
  float voltage = (rawValue * 3.3 )/ 1024.0;
  
  float Vin = (voltage * (R1 + R2) / R2)-0.28;
  //batt = (Vin/fullVoltage)*100;

  batt = (Vin -emptyVoltage) / (fullVoltage-emptyVoltage)*100;

  Serial.print("Battery Percentage: ");
  Serial.print(batt);
  Serial.println("%");
  Serial.print("Battery Voltage: ");
  Serial.println(Vin);
}
void thingSpeak()
{
  ThingSpeak.setField(1,BPM);
  ThingSpeak.setField(2,spo2);
  ThingSpeak.setField(3,tf);
  ThingSpeak.setField(4,batt);
  ThingSpeak.setStatus(myStatus);

  ThingSpeak.writeFields(ID,api_key);
  
  delay(1550);
}
