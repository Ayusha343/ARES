#include <Wire.h> 
#include <math.h> 

const int MPU = 0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; 
int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal; 
double t,tx,tf,pitch,roll;

void setup()
{
    Wire.begin(); 
    Wire.beginTransmission(MPU); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
    Serial.begin(9600); 
}

void loop()
{
    Wire.beginTransmission(MPU); 
    Wire.write(0x3B); 
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU,14,true); 

    //Acceleration data correction
    AcXcal = 0;
    AcYcal = 0;
    AcZcal = 0;

    //Temperature correction
    tcal = -1600;

    //Gyro correction
    GyXcal = 0;
    GyYcal = 0;
    GyZcal = 0;


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
  
    //read temperature data 
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L) 
  
    //read gyroscope data
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L) 

    //temperature calculation
    tx = Tmp + tcal;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //fahrenheit

    //get pitch/roll
    getAngle(AcX,AcY,AcZ);
  
    //printing values to serial port
    Serial.print("Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" Roll = "); Serial.println(roll);
  
    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(AcX + AcXcal);
    Serial.print(" Y = "); Serial.print(AcY + AcYcal);
    Serial.print(" Z = "); Serial.println(AcZ + AcZcal); 

    Serial.print("Temperature in celsius = "); Serial.print(t);  
    Serial.print(" fahrenheit = "); Serial.println(tf);  
  
    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(GyX + GyXcal);
    Serial.print(" Y = "); Serial.print(GyY + GyYcal);
    Serial.print(" Z = "); Serial.println(GyZ + GyZcal);
  
    Serial.print("\n");
    delay(1500);
}

//function to convert accelerometer values into pitch and roll
void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}