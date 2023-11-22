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

bool redledP;
int modeR;

unsigned long pM = 0; // Змінна для зберігання часу останньої відправки
const int interval = 10; // Інтервал затримки в мілісекундах

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    SerialBT.print("hello");

    unsigned long cM = millis();

    if (cM - pM >= interval) {
      pM = cM; // Зберігання часу останньої відправки
      
      mesh.sendSingle(2224853816,"garland_echo");
      pM = millis(); 
      mesh.sendSingle(624315197,"red_led_echo");
      pM = millis();
      mesh.sendSingle(635035530,"bedside_echo");
      pM = millis();
      mesh.sendSingle(985208077,"sens_echo");
    }
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


void setup() {
  Serial.begin(115200);
  //Serial.print("start");

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
    //Serial.print (str);

    mesh.sendBroadcast(str);
  }
}
