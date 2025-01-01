#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    //Serial.println("Client Connected to Bluetooth");
    SerialBT.print("hello\n");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    //Serial.println("Client disconnected from Bluetooth");
    ESP.restart();  // якщо ви дійсно хочете ребутати після відключення
  }
}

void setup() {
  // Запускаємо апаратний Serial для UART-зв’язку з Пристроєм A
  Serial2.begin(512000);
  Serial2.setTimeout(100);
  delay(100);

  // Запускаємо Bluetooth
  SerialBT.begin("Kennet'MASH'"); 
  SerialBT.register_callback(callback);

  //Serial.println("Bluetooth-місток запущено...");
}

void loop() {
  // Якщо в Bluetooth вхідні дані, читаємо їх і передаємо в UART (Serial)
  if (SerialBT.available()) {
    while (SerialBT.available()) {
      int btData = SerialBT.read();
      Serial2.write(btData); 
    }
  }

  // Якщо в UART (Serial) прийшли дані від Пристрою A, передаємо їх у Bluetooth
  if (Serial2.available()) {
    while (Serial2.available()) {
      String str = Serial2.readString();
      str.trim();
      SerialBT.print(str);
    }
  }
}
