//
// nodeId = 624409705
#include "painlessMesh.h"
#include "BluetoothSerial.h"


#define   MESH_PREFIX     "kennet"
#define   MESH_PASSWORD   "kennet123"

painlessMesh  mesh;
BluetoothSerial SerialBT;

// void sendMessage(String msg) {
//   mesh.sendBroadcast( msg );
// }

// void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
//   if (event == ESP_SPP_SRV_OPEN_EVT) {
    
//     Serial.println("Client Connected has address:");

//     for (int i = 0; i < 6; i++) {
//       Serial.printf("%02X", param->srv_open.rem_bda[i]);
//       if (i < 5) {
//         Serial.print(":");
//       }
//     }
//   }
// }

void setup() {
  Serial.begin(9600);

  mesh.init( MESH_PREFIX, MESH_PASSWORD );

//  SerialBT.register_callback(callback);

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