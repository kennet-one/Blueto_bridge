//
// nodeId = 624409705
//
#include "painlessMesh.h"
#include "BluetoothSerial.h"


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

 
  if(event == ESP_SPP_CLOSE_EVT ){            //ребут при дісконекті
    Serial.println("Client disconnected");
    ESP.restart();
  }
}

void receivedCallback( uint32_t from, String &msg ) {
  String str1 = msg.c_str();
  SerialBT.print(str1);
}

String formatNodeList(String json) {
  String formattedList = "nodes: ";
  int start = 0;
  int end = 0;
  bool firstNode = true; // Прапор для додавання коми між нодами

  // Проходимо по JSON-рядку, щоб витягти ID нод
  while ((start = json.indexOf("\"nodeId\":", end)) != -1) {
    start += 9; // Пропускаємо "nodeId":
    end = json.indexOf(",", start); // Знаходимо кінець ID або кінець об'єкта

    // Якщо не знайшли кому, шукаємо кінець об'єкта
    if (end == -1) {
      end = json.indexOf("}", start);
    }

    // Витягуємо ID ноди
    String nodeId = json.substring(start, end);
    nodeId.trim();
    nodeId.replace("}", "");
    nodeId.replace("]", "");

    // Додаємо ID ноди у форматований список
    if (!firstNode) {
      formattedList += ","; // Додаємо розділювач між елементами
    }
    formattedList += nodeId;
    firstNode = false;
  }

//  formattedList += "]"; // Закриваємо форматований список
  return formattedList;
}

void setup() {
  Serial.begin(115200);
  //Serial.print("start");

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  SerialBT.register_callback(callback);

  SerialBT.begin("Kennet'MASH'"); 
}


void loop() {
  mesh.update();

  if (SerialBT.available()) {
    String str = SerialBT.readString();
    str.trim();

    // Ігноруємо порожні повідомлення
    if (str.length() == 0) {
      return;
    }

    // Перевірка на команди debug
    if (str.equals("dbg1")) {
      debugi = true;
      String jsonNodeList = mesh.subConnectionJson();
      // Форматуємо рядок у бажаний формат
      String formattedNodeList = formatNodeList(jsonNodeList);

      SerialBT.println(formattedNodeList);

    } else if (str.equals("dbg0")) {
      debugi = false;
    } else {
      // Відправляємо повідомлення в mesh-мережу
      mesh.sendBroadcast(str);
    }
  }
}
