//
// nodeId = 624409705
//
#include "painlessMesh.h"
#include "BluetoothSerial.h"


#define   MESH_PREFIX     "kennet"
#define   MESH_PASSWORD   "kennet123"

painlessMesh  mesh;
BluetoothSerial SerialBT;

bool garlandS;

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    SerialBT.print("hello");

    mesh.sendSingle(2224853816,"garland_echo");
  }
 
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    ESP.restart();
  }
}

void receivedCallback( uint32_t from, String &msg ) {

  String str1 = msg.c_str();
  String str2 = "garland_off";
  String str3 = "garland_on";

  if (str1.equals(str2)) {
    garlandS = 0;
    //Serial.print(garlandS);
    SerialBT.print("garland0");
  }
  if (str1.equals(str3)) {
    garlandS = 1;
    //Serial.print(garlandS);
    SerialBT.print("garland1");
  }
}

void setup() {
  Serial.begin(9600);
  Serial.print("start");

  mesh.init( MESH_PREFIX, MESH_PASSWORD );
  mesh.onReceive(&receivedCallback);

  SerialBT.register_callback(callback);

  SerialBT.begin("Kennet'MASH'"); 

}

void loop() {

  mesh.update();

  if (SerialBT.available()) {

    String str = (SerialBT.readString());
    str.trim();
    Serial.print (str);

    mesh.sendBroadcast(str);

  }
  

}