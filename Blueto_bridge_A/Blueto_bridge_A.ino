#include "painlessMesh.h"
#include "mash_parameter.h"  // Ваш файл із параметрами мережі, якщо є

Scheduler userScheduler;
painlessMesh mesh;

bool debugi = false;

// === CALLBACKS FOR MESH ===
void newConnectionCallback(uint32_t nodeId) {
  if (debugi) {
    Serial2.printf("New Mesh Connection, nodeId = %u\n", nodeId);
  }
}

void receivedCallback(uint32_t from, String &msg) {
  // Продублюємо отримане повідомлення у Serial
  //Serial2.print(msg);
  Serial2.print(msg);
}


// === SETUP ===
void setup() {
  Serial2.begin(512000);  // Швидкість UART: 921600. Можна змінити (але вище 1M не завжди стабільно)
  Serial2.setTimeout(100); 
  delay(100);

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  // Можна додати додатковий Serial.println, щоб пересвідчитись, що все працює
  //Serial.println("Mesh node started...");
}

// === LOOP ===
void loop() {
  mesh.update();

  // Перевіряємо, чи є вхідні дані через UART (приходять від Пристрою B)
  if (Serial2.available()) {
    String str = Serial2.readString();
    str.trim();
    if (str.length() == 0) return; // якщо нічого не надійшло

    // Перевірка на debug-режим
    if (str.equals("dbg1")) {
      debugi = true;
      // Можна, наприклад, отримати список нодів
      String jsonNodeList = mesh.subConnectionJson();
      // ... Якщо бажаєте змінити формат – робіть як у вашій старій функції formatNodeList()
      Serial2.print(jsonNodeList); // потім Пристрій B побачить це і передасть у Bluetooth
    } 
    else if (str.equals("dbg0")) {
      debugi = false;
    } 
    else if (str.equals("kyy")) {
      Serial2.print("ky");  // надсилаємо назад, Пристрій B передасть далі в Bluetooth
    } 
    else {
      // Будь-які інші команди вважаємо повідомленнями, які треба розслати в mesh
      mesh.sendBroadcast(str);
    }
  }
}
