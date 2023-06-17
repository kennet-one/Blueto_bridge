#include "BluetoothSerial.h"
#include "painlessMesh.h"

#define   MESH_PREFIX     "kennet"
#define   MESH_PASSWORD   "kennet123"
#define   MESH_PORT       5555

BluetoothSerial SerialBT;
painlessMesh  mesh;



void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    
    Serial.println("Client Connected has address:");

    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", param->srv_open.rem_bda[i]);
      if (i < 5) {
        Serial.print(":");
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  mesh.init( MESH_PREFIX, MESH_PASSWORD );

  SerialBT.register_callback(callback);

  if (!SerialBT.begin("Kennet'MASH'")) {
  }
  else {
    Serial.println("Bluetooth initialized");
  }
  // Виведення ідентифікатора вузла
  uint32_t nodeId = mesh.getNodeId();
  Serial.print("Node ID: ");
  Serial.println(nodeId);
}

void loop() {

  mesh.update();

  while (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

}