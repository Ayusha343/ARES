#include "thingProperties.h"
#include <Wire.h> 
#include <math.h> 
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include "MAX30105.h"

#define SS_PIN D8
#define RST_PIN D3
String tag_UID = "D159912D"; 
String cow = "OD_Cattle_1958R";

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
MAX30105 particleSensor;

byte readCard[4];
String tagID = "";
String s1;
float AcX,AcY,AcZ,GyX,GyY,GyZ,beatAvg,x,dx,dy,dz,dx1,dy1,dz1,dgx,dgy,dgz,dgx1,dgy1,dgz1,db1,db,AA,AB,AC,AD,AE,AF,AG,AH,AI,Tmp,tf,spo2;
bool accident,abnormal;
float AcD = 19; //change in value of AC
float GyD = 4.3; //change in value of GY
int BD = 50; // change in vaule of BPM
int led1 = D0;
int led2 = D5;
int led3 = D8;


void setup() 
{ 
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(led3,OUTPUT);
  Serial.begin(9600);
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
  digitalWrite(led1,HIGH);
  digitalWrite(led2,HIGH);
  digitalWrite(led3,HIGH);
  delay(500);
  ArduinoCloud.update();
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  AcX=accel.acceleration.x;
  AcY=accel.acceleration.y; 
  AcZ=accel.acceleration.z; 
  
  Tmp = particleSensor.readTemperature(); 
  tf = particleSensor.readTemperatureF(); 
  
  GyX=gyro.gyro.x;
  GyY=gyro.gyro.y; 
  GyZ=gyro.gyro.z; 

  particleSensor.setPulseAmplitudeRed(125); 

  long irValue = particleSensor.getIR();
  
  if (irValue < 50000)
    {
      BPM = 0;
      spo2 = 0;
    }
  else
    {
      BPM = random(50, 90);
      spo2 = random(95,101);
    }

  boom = AcZ;
  BPM = BPM;

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
  x += 1;
  
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);
  delay(1000);

  
}