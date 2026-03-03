/*SSIDs: Dr Ranjan Sahoo   MSME-Incubation*/
/*PASS: Dr Ranjan 1181    P@ssw0rd*/

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "32eb54c8-ac0d-4fd8-8935-30d793fda57d";

const char SSID[] = "MSME-Incubation";    // Network SSID (name)
const char PASS[] = "P@ssw0rd";    // Network password 
const char DEVICE_KEY[]  = "5DS7WJEK4YEGGJ6DU4US";    // Secret device password


int BPM;

void initProperties()
{

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
