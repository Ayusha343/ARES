/*SSIDs: Dr Ranjan Sahoo   MSME-Incubation   Ranjan's Galaxy Z Fold4*/
/*PASS: Dr Ranjan 1181    P@ssw0rd   ijtt9258*/

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <ESP8266WiFi.h>

const char DEVICE_LOGIN_NAME[]  = "32eb54c8-ac0d-4fd8-8935-30d793fda57d";

const char SSID[] = "Dr Ranjan Sahoo";    // Network SSID (name)
const char PASS[] = "Dr Ranjan 1181";    // Network password 
const char DEVICE_KEY[]  = "5DS7WJEK4YEGGJ6DU4US";    // Secret device password


String sms1;

void initProperties()
{

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(sms1, READ,ON_CHANGE,NULL);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
