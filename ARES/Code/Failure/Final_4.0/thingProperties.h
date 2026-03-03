/*SSIDs: Dr Ranjan Sahoo*/
/*PASS: Dr Ranjan 1181*/

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "32eb54c8-ac0d-4fd8-8935-30d793fda57d";

const char SSID[] = "Dr Ranjan Sahoo";    // Network SSID (name)
const char PASS[] = "Dr Ranjan 1181";    // Network password 
const char DEVICE_KEY[]  = "5DS7WJEK4YEGGJ6DU4US";    // Secret device password


int boom = 0;
int BPM;

void initProperties()
{

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  //ArduinoCloud.addProperty(boom, READ, ON_CHANGE, NULL, 19);
  ArduinoCloud.addProperty(BPM, READ,ON_CHANGE,NULL,50);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
