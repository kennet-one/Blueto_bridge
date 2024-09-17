//
// nodeId = 624409705
//
#include "painlessMesh.h"
#include "BluetoothSerial.h"
#include <ArduinoJson.h>

#define   MESH_PREFIX     "kennet"
#define   MESH_PASSWORD   "kennet123"
#define   MESH_PORT       5555

Scheduler userScheduler;
painlessMesh  mesh;
BluetoothSerial SerialBT;

bool debugi = false;

void newConnectionCallback(uint32_t nodeId) {
  if (debugi == true){
    SerialBT.printf("New Connection, nodeId = %u\n", nodeId);
  }
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    SerialBT.print("hello");
  }
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    ESP.restart();
  }
}

void receivedCallback( uint32_t from, String &msg ) {
  SerialBT.print(msg);
}

void setup() {
  Serial.begin(115200);

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  SerialBT.register_callback(callback);
  SerialBT.begin("Kennet'MASH'");

  // Створення задачі для обробки Bluetooth
  xTaskCreatePinnedToCore(
    bluetoothTask,   // Функція задачі
    "BluetoothTask", // Ім'я задачі
    4096,            // Розмір стеку в байтах
    NULL,            // Параметри задачі
    1,               // Пріоритет задачі
    NULL,            // Вказівник на ідентифікатор задачі
    1);              // Ядро процесора (0 або 1)
}

void loop() {
  mesh.update();
}

// Додавання функції formatNodeList
String formatNodeList(String json) {
  String formattedList = "nodes: ";
  const size_t capacity = JSON_ARRAY_SIZE(10) + 10*JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    SerialBT.println("Failed to parse JSON");
    return "";
  }

  JsonArray array = doc.as<JsonArray>();
  for (JsonObject obj : array) {
    uint32_t nodeId = obj["nodeId"];
    formattedList += String(nodeId) + ",";
  }

  if (formattedList.endsWith(",")) {
    formattedList.remove(formattedList.length() - 1);
  }

  return formattedList;
}

// Задача для обробки Bluetooth
void bluetoothTask(void * parameter) {
  const int bufferSize = 256;
  char buffer[bufferSize];

  for(;;) {
    if (SerialBT.available()) {
      int len = SerialBT.readBytesUntil('\n', buffer, bufferSize - 1);
      buffer[len] = '\0';
      String str = String(buffer);
      str.trim();

      if (str.length() == 0) {
        // Порожнє повідомлення
      } else if (str.equals("dbg1")) {
        debugi = true;
        String jsonNodeList = mesh.subConnectionJson();
        String formattedNodeList = formatNodeList(jsonNodeList);
        SerialBT.println(formattedNodeList);
      } else if (str.equals("dbg0")) {
        debugi = false;
      } else {
        mesh.sendBroadcast(str);
      }
    }
    taskYIELD();

  }
}
