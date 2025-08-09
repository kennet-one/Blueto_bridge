#include <SPI.h>
#include <BluetoothSerial.h>

// ======= Піни (VSPI) =======
static const int PIN_SCK   = 18;
static const int PIN_MISO  = 19;
static const int PIN_MOSI  = 23;
static const int PIN_CS    = 5;
static const int PIN_READY = 13;        // з SLAVE GPIO4
static const uint32_t SPI_HZ = 2000000; // 2 МГц

static const int LED_PIN = 2;           // опц. моргалка

BluetoothSerial SerialBT;

// ======= 32B протокол =======
static const uint8_t FRAME_SIZE = 32;
static const uint8_t MAGIC      = 0xA5;
enum : uint8_t { T_NOP=0, T_TEXT=1, T_DOUBLE=2 };

uint8_t  tx_frame[FRAME_SIZE];
uint8_t  rx_frame[FRAME_SIZE];
uint16_t seq_tx = 0;

char   pending_text[24] = {0};
bool   have_pending = false;
bool   doubleMode   = false;
double pending_double = 0.0;

volatile bool bt_connected = false;
bool hello_sent = false;

// ======= helpers =======
static inline uint8_t checksum(const uint8_t* f){ uint8_t c=0; for (int i=0;i<=28;i++) c^=f[i]; return c; }
static inline uint8_t safeLen24(const char* s){ uint8_t n=0; while (n<24 && s[n]) n++; return n; }

void buildFrame(uint8_t type, uint16_t seq, const uint8_t* data, uint8_t len){
  if (len>24) len=24;
  memset(tx_frame, 0, FRAME_SIZE);
  tx_frame[0]=MAGIC; tx_frame[1]=type;
  tx_frame[2]=(uint8_t)(seq & 0xFF);
  tx_frame[3]=(uint8_t)(seq >> 8);
  tx_frame[4]=len;
  if (len) memcpy(&tx_frame[5], data, len);
  tx_frame[29]=checksum(tx_frame);
}
bool parseFrame(const uint8_t* f, uint8_t& type, uint16_t& seq, uint8_t* data, uint8_t& len){
  if (f[0]!=MAGIC) return false;
  if (checksum(f)!=f[29]) return false;
  type=f[1]; seq=(uint16_t)f[2] | ((uint16_t)f[3]<<8);
  len=f[4]; if (len>24) len=24;
  if (data && len) memcpy(data,&f[5],len);
  return true;
}
void makeNOP()               { buildFrame(T_NOP,    ++seq_tx, nullptr, 0); }
void makeText(const char* s) { buildFrame(T_TEXT,   ++seq_tx, (const uint8_t*)s, safeLen24(s)); }
void makeDouble(double v)    { uint8_t b[8]; memcpy(b,&v,8); buildFrame(T_DOUBLE, ++seq_tx, b, 8); }

void bt_cb(esp_spp_cb_event_t e, esp_spp_cb_param_t*){
  if (e==ESP_SPP_SRV_OPEN_EVT){ bt_connected = true;  hello_sent = false; }
  if (e==ESP_SPP_CLOSE_EVT)   { bt_connected = false; hello_sent = false; }
}

void pullInputFromBTorUSB(){
  if (SerialBT.available()){
    String in = SerialBT.readString(); in.trim();
    if (in.length()){
      if (in.startsWith("d ")) { pending_double=in.substring(2).toDouble(); doubleMode=true; }
      else { in.toCharArray(pending_text,sizeof(pending_text)); doubleMode=false; }
      have_pending=true;
    }
  }
  if (Serial.available()){
    String in = Serial.readString(); in.trim();
    if (in.length()){
      if (in.startsWith("d ")) { pending_double=in.substring(2).toDouble(); doubleMode=true; }
      else { in.toCharArray(pending_text,sizeof(pending_text)); doubleMode=false; }
      have_pending=true;
    }
  }
}

void spiExchange32(){
  if (have_pending){
    if (doubleMode) makeDouble(pending_double);
    else            makeText(pending_text);
    have_pending=false;
  } else {
    makeNOP();
  }

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  for (uint8_t i=0;i<FRAME_SIZE;i++) rx_frame[i] = SPI.transfer(tx_frame[i]);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  memset(tx_frame, 0, FRAME_SIZE);
}

void handleRx(){
  uint8_t type,len,data[24]; uint16_t seq;
  if (!parseFrame(rx_frame,type,seq,data,len)) return;

  if (type==T_TEXT && len){
    String s; s.reserve(len); for(uint8_t i=0;i<len;i++) s+=(char)data[i];
    // лише LF, без CR — щоб у Python не з’являлись порожні рядки
    SerialBT.print(s); SerialBT.print('\n');
    Serial.printf("[S->] TEXT(%u): %s\n", seq, s.c_str());
  } else if (type==T_DOUBLE && len==8){
    double dv; memcpy(&dv,data,8);
    Serial.printf("[S->] DOUBLE(%u): %.10f\n", seq, dv);
  }
}

void setup(){
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  Serial.printf("ROLE: MASTER, MAC:%llX\n", ESP.getEfuseMac());

  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  pinMode(PIN_READY, INPUT_PULLDOWN);

  SerialBT.setTimeout(20);
  Serial.setTimeout(20);
  SerialBT.begin("Kennet'MASH'");
  SerialBT.register_callback(bt_cb);

  Serial.println("MASTER: BT+SPI готовий.");
}

void loop() {
  // hello одноразово
  if (SerialBT.hasClient() && bt_connected && !hello_sent) { SerialBT.print("hello\n"); hello_sent = true; }

  pullInputFromBTorUSB();

  // обмін починаємо, якщо (а) SLAVE має дані АБО (б) у нас є пакет
  if (have_pending || digitalRead(PIN_READY)==HIGH) {
    do {
      spiExchange32();
      handleRx();
      // коротка пауза лише щоб SLAVE встиг поставити наступну чергу
      delayMicroseconds(200);
    } while (digitalRead(PIN_READY)==HIGH); // зливаємо "хвіст" без лишніх очікувань
  }
}
