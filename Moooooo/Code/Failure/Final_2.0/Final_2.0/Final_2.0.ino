#include <Wire.h> 
#include <math.h> 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <MFRC522.h>
#include "MAX30105.h"

#define MAX_BRIGHTNESS 255
#define SS_PIN D8
#define RST_PIN D3

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
MFRC522 mfrc522(SS_PIN, RST_PIN);
MAX30105 particleSensor;
TinyGPSPlus gps;


byte readCard[4];
String tag_UID = "D159912D";  // Replace this with the UID of your tag!!!
String tagID = "";
float Tmp,tf;
bool accident,abnormal;
char lat[] = "xxxxxx";
char lon[] = "xxxxxx";
float AcD = 19; //change in value of AC
float GyD = 4.3; //change in value of GY
int BD = 50; // change in vaule of BPM
int RXPin = 2;
int TXPin = 3;
String cow,s1;
float AcX,AcY,AcZ,GyX,GyY,GyZ,beatAvg,x,dx,dy,dz,dx1,dy1,dz1,dgx,dgy,dgz,dgx1,dgy1,dgz1,db1,db,AA,AB,AC,AD,AE,AF,AG,AH,AI,bpm,spo2;


//SoftwareSerial gpsSerial(RXPin, TXPin);
SoftwareSerial mySerial(D7,D6);

void setup()
{ 
  randomSeed(analogRead(0));

  Serial.begin(9600); 
  //gpsSerial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Initializing...");
  delay(500);
  

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  mySerial.println("AT+CCLK?"); //Check whether it has registered in the network
  updateSerial();
  delay(500);

  Serial.println("Adafruit MPU6050 test!");
  delay(500);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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
  delay(500);
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  delay(500);

  particleSensor.enableDIETEMPRDY(); 
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
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

/*  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected"));
    while(true);
  }
*/
  readID();
  while (readID())
  {
    if (tagID == tag_UID)
    {
      cow = "OD_Cattle_1958R";
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      s1 = "\n";
      mySerial.println(s1 + "ID : " + cow + "\nWeight: 439 Kg \nHeight: 124 cm");
      updateSerial();
      mySerial.write(26);
      Serial.println(tagID);
    }
    Serial.println(tagID);
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

    //read accelerometer data
    AcX=accel.acceleration.x;
    AcY=(accel.acceleration.y); 
    AcZ=accel.acceleration.z; 
  
    //read temperature data 
    Tmp = particleSensor.readTemperature(); 
    tf = particleSensor.readTemperatureF(); 
  
    //read gyroscope data
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
      mySerial.println("AT+CCLK?");
      while(mySerial.available()) 
      {
        Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
        String time = mySerial.read();
        tim = time.substring(7,25);
      }
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      s1 = "\n";
      mySerial.print(s1 + "Collision detected!!!" + "\nCattle no. " + cow + " needs immediate help at: \n Lat:"  + lat + "\n Lon:" + lon + "\nTime :" + tim + "\nVitals: \nBPM: " + bpm + "\nSpo2: " + spo2 +"\nTemp: " + Tmp + "\ " + tf );
      updateSerial();
      mySerial.write(26);
      mySerial.println("ATD+ +91xxxxxxxxxx;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
      updateSerial();
      delay(10000); // wait for 20 seconds...
      mySerial.println("ATH"); //hang up
      updateSerial();
      mySerial.println("AT+CNMI=1,2,0,0,0");
      while(mySerial.available()) 
      {
        Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
        char buffer[20];  // Adjust the size as needed
        mySerial.readBytesUntil('\n', buffer, sizeof(buffer));
        if (strcmp(buffer, "Stop") == 0)
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
      mySerial.println("AT+CCLK?");
      while(mySerial.available()) 
      {
        Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
        time = mySerial.read();
        tim = time.substring(7,25);
      }
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      s1 = "\n";
      mySerial.print(s1 + "Abnormality detected!!!" + "\nCattle no. " + cow + " needs immediate help at: \n Lat:"  + lat + "\n Lon:" + lon + "\nTime :" + tim + "\nVitals: \nBPM: " + bpm + "\nSpo2: " + spo2 +"\nTemp: " + Tmp + "\ " + tf );
      updateSerial();
      mySerial.write(26);    
      char buffer[20];  // Adjust the size as needed
      mySerial.readBytesUntil('\n', buffer, sizeof(buffer));
      if (strcmp(buffer, "Stop") == 0)
        {
          abnormal=false;
          mySerial.println("AT+CMGS=\"+91xxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
          updateSerial();
          mySerial.println("Cattle treated");
        }  
      digitalWrite(10,1);
    }

    Serial.print("Accelerometer: ");
    Serial.print("X = "); 
    Serial.print(accel.acceleration.x);
    Serial.print(" Y = "); 
    Serial.print(dy1);
    Serial.print(" Z = "); 
    Serial.println(dz1); 

    Serial.print("Temperature in celsius = "); 
    Serial.print(Tmp);  
    Serial.print(" fahrenheit = "); 
    Serial.println(tf);  
  
    Serial.print("Gyroscope: ");
    Serial.print("X = "); 
    Serial.print(dgx1);
    Serial.print(" Y = ");
    Serial.print(dgy1);
    Serial.print(" Z = "); 
    Serial.println(dgz1);
  
    Serial.print("\n");
    delay(1500);
    x += 1;

    particleSensor.setPulseAmplitudeRed(0); 
}

/*void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print(F("Latitude: "));
    Serial.println(gps.location.lat(), 6);
    Serial.print(F("Longitude: "));
    Serial.println(gps.location.lng(), 6);
    Serial.print(F("Altitude: "));
    Serial.println(gps.altitude.meters());
    lat=gps.location.lat();
    lon=gps.location.lng();
  }
  else
  {
    Serial.println(F("Location: Not Available"));
  }
  
  Serial.println();
  Serial.println();
  delay(1000);
}
*/

void updateSerial()
{
  delay(500);
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
