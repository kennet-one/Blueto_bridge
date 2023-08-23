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

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    SerialBT.print("hello");

    mesh.sendSingle(2224853816,"garland_echo");
    mesh.sendSingle(624315197,"red_led_echo");
  }
 
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    ESP.restart();
  }
}

void receivedCallback( uint32_t from, String &msg ) {

  String str1 = msg.c_str();
  Serial.print(str1);
  String str2 = "garland_off";
  String str3 = "garland_on";

  String str4 = "redled_off";
  String str5 = "redled_on";

  String modKeyR = "01";                         // "01_mode_0"
  String briKeyR = "02"; 

  if (str1.substring(0, 2) == modKeyR) {
    if (str1.endsWith(String("0"))) {      SerialBT.print("redled,_,0");
    } else if (str1.endsWith(String("1"))) {      SerialBT.print("redled,_,1");
    } else if (str1.endsWith(String("2"))) {      SerialBT.print("redled,_,2");
    } else if (str1.endsWith(String("3"))) {      SerialBT.print("redled,_,3");
    } else if (str1.endsWith(String("4"))) {      SerialBT.print("redled,_,4");
    } else if (str1.endsWith(String("5"))) {      SerialBT.print("redled,_,5");
    } else if (str1.endsWith(String("6"))) {      SerialBT.print("redled,_,6");
    } else if (str1.endsWith(String("7"))) {      SerialBT.print("redled,_,7");
    } else if (str1.endsWith(String("8"))) {      SerialBT.print("redled,_,8");
    } else if (str1.endsWith(String("9"))) {      SerialBT.print("redled,_,9");
    }
  }

  if (str1.substring(0, 2) == briKeyR) {
    if (str1.length() == 3) {SerialBT.print("redled,_,_,0");}

    if (str1.length() == 4) {
      if (str1.endsWith(String("26"))) {SerialBT.print("redled,_,_,1");
      } else if (str1.endsWith(String("51"))) {SerialBT.print("redled,_,_,2");
      } else if (str1.endsWith(String("77"))) {SerialBT.print("redled,_,_,3");
      }
    if (str1.length() == 5) {
      if (str1.endsWith(String("102"))) {SerialBT.print("redled,_,_,4");
      } else if (str1.endsWith(String("128"))) {SerialBT.print("redled,_,_,5");
      } else if (str1.endsWith(String("153"))) {SerialBT.print("redled,_,_,6");
      } else if (str1.endsWith(String("179"))) {SerialBT.print("redled,_,_,7");
      } else if (str1.endsWith(String("204"))) {SerialBT.print("redled,_,_,8");
      } else if (str1.endsWith(String("230"))) {SerialBT.print("redled,_,_,9");
      } else if (str1.endsWith(String("255"))) {SerialBT.print("redled,_,_,M");
      }
    }
  }
}

  if (str1.equals(str2)) {
    garlandS = 0;
    SerialBT.print("garland0");
  }
  if (str1.equals(str3)) {
    garlandS = 1;
    SerialBT.print("garland1");
  }

  if (str1.equals(str4)) {
    redledP = 0;
    SerialBT.print("redled,0");
  }
  if (str1.equals(str5)) {
    redledP = 1;
    SerialBT.print("redled,1");
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