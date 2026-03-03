#define xPin A0
#define yPin A3
#define zPin A6
float xRaw = 0;
float yRaw = 0;
float zRaw = 0;

const int sampul = 50;

void setup() 
{
  pinMode(xPin,INPUT);
  pinMode(yPin,INPUT);
  pinMode(zPin,INPUT);
  Serial.begin(115200);
}


void loop() 
{
  for (int i=0;i<sampul;i++)
  {
    xRaw = xRaw + analogRead(xPin);
    yRaw = yRaw + analogRead(yPin);
    zRaw = zRaw + analogRead(zPin);
  }
  xRaw /= sampul;
  yRaw /= sampul;
  zRaw /= sampul;

  Serial.print((String)"x: "+xRaw+"\t");
  Serial.print((String)"y: "+yRaw+"\t");
  Serial.print((String)"z: "+zRaw+"\t \n");

  delay(200);
}
