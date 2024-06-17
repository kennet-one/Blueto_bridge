#include "painlessMesh.h"
#include "BluetoothSerial.h"

#define MESH_PREFIX     "buhtan"
#define MESH_PASSWORD   "buhtan123"
#define MESH_PORT 5555


painlessMesh mesh;
BluetoothSerial SerialBT;


void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    SerialBT.print("hello");


  }

 
  if(event == ESP_SPP_CLOSE_EVT ){            //ребут при дісконекті
    Serial.println("Client disconnected");
    ESP.restart();
  }
}

void receivedCallback(uint32_t from, String &msg) {
  String str1 = msg.c_str();
  SerialBT.print(str1);
}

void setup() {
  Serial.begin(115200);

  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  
  SerialBT.register_callback(callback);



  SerialBT.begin("pies'MASH'");
  Serial.println("The device started, now you can pair it with Bluetooth!");
}

void loop() {
  mesh.update();

  // Передача даних з Bluetooth в mesh
  if (SerialBT.available()) {
    String str = SerialBT.readString();
    str.trim();
    mesh.sendBroadcast(str);
  }
}
