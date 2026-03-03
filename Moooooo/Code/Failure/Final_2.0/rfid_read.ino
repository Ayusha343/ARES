#include <SPI.h>
#include <MFRC522.h>
#define SS_PIN D8
#define RST_PIN D3
#define LED D2
byte readCard[4];
String tag_UID = "D159912D";  // Replace this with the UID of your tag!!!
String tagID = "";
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
void setup()
{ 
  Serial.begin(9600);
  pinMode(LED, OUTPUT);// initialize digital pin LED_BUILTIN as an output.
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  SPI.begin(); // SPI bus
  mfrc522.PCD_Init(); // Initialise MFRC522
}
void loop()
{
  //Wait until new tag is available
  while (readID())
  {
    if (tagID == tag_UID)
    {
      digitalWrite(LED, !digitalRead(LED));  // Turn on or off the onboard led
    }
    Serial.println(tagID);
  }
  
}
  //Read new tag if available
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