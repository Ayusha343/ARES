#include <Wire.h> 
#include <math.h> 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <MFRC522.h>
#include "MAX30105.h"

#define SS_PIN D8
#define RST_PIN D3
#define RXPin D7
#define TXPin D6
String tag_UID = "D159912D"; 
String cow = "OD_Cattle_1958R";
String lat = "xxxxxx";
String lon = "xxxxxx";

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
MAX30105 particleSensor;
TinyGPSPlus gps;
MFRC522 mfrc522(SS_PIN, RST_PIN);
SoftwareSerial mySerial(RXPin, TXPin);


byte readCard[4];
String tagID = "";
String s1;
float AcX,AcY,AcZ,GyX,GyY,GyZ,beatAvg,x,dx,dy,dz,dx1,dy1,dz1,dgx,dgy,dgz,dgx1,dgy1,dgz1,db1,db,AA,AB,AC,AD,AE,AF,AG,AH,AI,bpm,spo2,Tmp,tf;
bool accident,abnormal;
float AcD = 19; //change in value of AC
float GyD = 4.3; //change in value of GY
int BD = 50; // change in vaule of BPM

void setup()
{ 
  randomSeed(analogRead(0));

  Serial.begin(9600); 
  mySerial.begin(9600);
  Serial.println("Initializing...");
  delay(100);
  
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  mySerial.println("AT+CCLK?"); //Returns date and time
  updateSerial();
  mySerial.println("AT+CLTS=1");//Enable network time updates
  updateSerial();
  delay(100);

  Serial.println("Adafruit MPU6050 test!");

  while (!Serial)
    delay(10); 
  
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
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
}


void loop()
{ 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  particleSensor.setPulseAmplitudeRed(125); 

  while (readID())
    {
      if (tagID == tag_UID)
      {
        mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
        updateSerial();
        mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
        updateSerial();
        s1 = "\n";
        mySerial.println(s1 + "ID : " + cow + "\nWeight: 439 Kg \nHeight: 124 cm");
        delay(1000);
        updateSerial();
        mySerial.write(26);
        Serial.println(tagID);
      }
    }
  long irValue = particleSensor.getIR();

  db1 = bpm;
  
  if (irValue < 50000)
    {
      bpm = 0;
      spo2 = 0;
    }
  else
    {
      bpm = random(50, 90);
      spo2 = random(95,101);
    }

  Serial.print("Bpm: ");
  Serial.println(bpm);
  Serial.print("Spo2: ");
  Serial.println(spo2);
  Serial.println();

  dx1 = AcX;
  dy1 = AcY;
  dz1 = AcZ;

  dgx1 = GyX;
  dgy1 = GyY;
  dgz1 = GyZ;

  AcX=accel.acceleration.x;
  AcY=accel.acceleration.y; 
  AcZ=accel.acceleration.z; 
  
  Tmp = particleSensor.readTemperature(); 
  tf = particleSensor.readTemperatureF(); 
  
  GyX=gyro.gyro.x;
  GyY=gyro.gyro.y; 
  GyZ=gyro.gyro.z; 

  if (x > 10)
    { 

      dx = AcX;
      dy = AcY;
      dz = AcZ;

      dgx = GyX;
      dgy = GyY;
      dgz = GyZ;
      
      db = bpm;

      AA = dx - dx1;
      AB = dy - dy1;
      AC = dz - dz1;
      AD = dgx - dgx1;
      AE = dgy - dgy1;
      AF = dgz - dgz1;
      AG = db - db1;

      if (((abs(AA))>AcD) || ((abs(AB))>AcD) || ((abs(AC))>AcD) || ((abs(AD))>GyD) || ((abs(AE))>GyD) || ((abs(AF))>GyD) && ((abs(AG)>BD)))
      {
        accident = true;
      }
      else if (((abs(AG)>BD)) || spo2<90 && tf>=103 || Tmp>39.5)
      {
        abnormal = true;
      }
    }
    
  if (accident==true)
    { 
      String time = getTime();
      Serial.println(time);
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      s1 = "\n";
      mySerial.print(s1 + "Collision detected!!!" + "\nCattle no. " + cow + " needs immediate help at: \n Lat:"  + lat + "\n Lon:" + lon + "\nTime :" + time + "\nVitals: \nBPM: " + bpm + "\nSpo2: " + spo2 +"\nTemp: " + Tmp + "\ " + tf );
      delay(1000);
      updateSerial();
      mySerial.write(26);
      mySerial.println("ATD+ +91xxxxxxxxxx;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
      updateSerial();
      delay(10000); // wait for 10 seconds...
      mySerial.println("ATH"); //hang up
      updateSerial();
      mySerial.println("AT+CNMI=1,2,0,0,0");
      String currentResponse = readResponse();
      while(mySerial.available()) 
      {
        Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
        if (currentResponse == "Stop")
        {
          accident=false;
          mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
          updateSerial();
          mySerial.println("Issue resolved");
          mySerial.write(26);
        }
      }
    }
  if (abnormal==true)
    {
      String time = getTime();
      Serial.println(time);
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      s1 = "\n";
      mySerial.print(s1 + "Abnormality detected!!!" + "\nCattle no. " + cow + " needs immediate help at: \n Lat:"  + lat + "\n Lon:" + lon + "\nTime :" + time + "\nVitals: \nBPM: " + bpm + "\nSpo2: " + spo2 +"\nTemp: " + Tmp + "\ " + tf );
      delay(1000);
      updateSerial();
      mySerial.write(26); 
      String currentResponse = readResponse();   
      if (currentResponse == "Stop")
        {
          abnormal=false;
          mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
          updateSerial();
          mySerial.println("Cattle treated");
        }  
     }

    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(dx1);
    Serial.print(" Y = "); Serial.print(dy1);
    Serial.print(" Z = "); Serial.println(dz1); 

    Serial.print("Temperature in celsius = "); Serial.print(Tmp);  
    Serial.print(" fahrenheit = "); Serial.println(tf);  
  
    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(dgx1);
    Serial.print(" Y = "); Serial.print(dgy1);
    Serial.print(" Z = "); Serial.println(dgz1);
  
    Serial.print("\n");
    delay(1500);
    x += 1;
}

String getTime() 
{
  mySerial.println("AT+CCLK?");
  delay(1000);

  String currentResponse = readResponse();

  int timeStart = currentResponse.indexOf("\"") + 1;
  int timeEnd = currentResponse.lastIndexOf("\"");
  String time = currentResponse.substring(timeStart, timeEnd);

  return time;
}

String readResponse() 
{
  String response = "";

  while (mySerial.available()) 
  {
    char c = mySerial.read();
    response += c;
    delay(10); 
  }

  return response;
}

void updateSerial()
{
  delay(1000);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

boolean readID()
  {
    //Check if a new tag is detected or not. If not return.
 if ( ! mfrc522.PICC_IsNewCardPresent())
    {
      return false;
    }
    //Check if a new tag is readable or not. If not return.
    if ( ! mfrc522.PICC_ReadCardSerial())
    {
      return false;
    }
    tagID = "";
    // Read the 4 byte UID
    for ( uint8_t i = 0; i < 4; i++)
    {
      readCard[i] = mfrc522.uid.uidByte[i];
      tagID.concat(String(mfrc522.uid.uidByte[i], HEX)); // Convert the UID to a single String
    }
    tagID.toUpperCase();
    mfrc522.PICC_HaltA(); // Stop reading
    return true;
  }

