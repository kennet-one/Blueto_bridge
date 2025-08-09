#include <ESP32SPISlave.h>
#include <painlessMesh.h>
#include "mash_parameter.h"   // MESH_PREFIX, MESH_PASSWORD, MESH_PORT

// ======= Піни (VSPI) =======
static const int PIN_SCK   = 18;
static const int PIN_MISO  = 19;
static const int PIN_MOSI  = 23;
static const int PIN_CS    = 5;
static const int PIN_READY = 4;         // -> MASTER 13
static const int LED_PIN   = 2;

ESP32SPISlave slave;
Scheduler     userScheduler;
painlessMesh  mesh;

// ======= 32B протокол =======
static const uint8_t FRAME_SIZE = 32;
static const uint8_t MAGIC      = 0xA5;
enum : uint8_t { T_NOP=0, T_TEXT=1, T_DOUBLE=2 };

uint8_t  tx_frame[FRAME_SIZE];      // DMA in-flight буфер (НЕ чіпати поза циклом)
uint8_t  rx_frame[FRAME_SIZE];

uint8_t  stage_frame[FRAME_SIZE];   // стадж для наступної транзакції
volatile bool stage_pending = false;

uint16_t seq_tx = 0;
bool     queued = false;

inline void setReady(bool r){ digitalWrite(PIN_READY, r?HIGH:LOW); }

static inline uint8_t checksum(const uint8_t* f){ uint8_t c=0; for(int i=0;i<=28;i++) c^=f[i]; return c; }
static inline void buildFrameTo(uint8_t* buf, uint8_t type, uint16_t seq, const uint8_t* data, uint8_t len){
  if (len>24) len=24;
  memset(buf,0,FRAME_SIZE);
  buf[0]=MAGIC; buf[1]=type;
  buf[2]=(uint8_t)(seq & 0xFF);
  buf[3]=(uint8_t)(seq >> 8);
  buf[4]=len;
  if (len) memcpy(&buf[5], data, len);
  buf[29]=checksum(buf);
}
bool parseFrame(const uint8_t* f, uint8_t& type, uint16_t& seq, uint8_t* data, uint8_t& len){
  if (f[0]!=MAGIC) return false;
  if (checksum(f)!=f[29]) return false;
  type=f[1]; seq=(uint16_t)f[2] | ((uint16_t)f[3]<<8);
  len=f[4]; if (len>24) len=24;
  if (data && len) memcpy(data,&f[5],len);
  return true;
}

// ---- формування кадрів у STAGE (безпечне) ----
void stageText(const String& s){
  uint8_t b[24]; uint8_t n=(uint8_t)min(24,(int)s.length());
  memcpy(b, s.c_str(), n);
  buildFrameTo(stage_frame, T_TEXT, ++seq_tx, b, n);
  stage_pending = true;
}
void stageDouble(double v){
  uint8_t b[8]; memcpy(b,&v,8);
  buildFrameTo(stage_frame, T_DOUBLE, ++seq_tx, b, 8);
  stage_pending = true;
}
void stageNOP(){
  buildFrameTo(stage_frame, T_NOP, ++seq_tx, nullptr, 0);
  stage_pending = true;
}

// ---- постановка транзакції (TX+RX 32 байти) ----
void queueOnce(){
  if (!queued){
    slave.queue(tx_frame, rx_frame, FRAME_SIZE);
    slave.trigger();
    setReady(true);     // READY тільки після queue!
    queued = true;
  }
}

// ======= Mesh callbacks =======
void receivedCallback(uint32_t from, String &msg){
  // лог — видно у Моніторі порта
  Serial.printf("[MESH <- %u] %s\n", from, msg.c_str());
  // БЕЗПЕЧНО: кладемо у stage, не чіпаємо DMA-буфер
  stageText(msg);
}
void newConnectionCallback(uint32_t /*nodeId*/){}

// ======= setup =======
void setup(){
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.printf("ROLE: SLAVE,  MAC:%llX\n", ESP.getEfuseMac());

  pinMode(PIN_READY, OUTPUT);
  setReady(false);

  // Mesh
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  // SPI SLAVE
  slave.setQueueSize(1);
  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // перше повідомлення (вітання) — одразу в stage
  stageText("Hello from SLAVE!");

  // згенеруємо перший TX кадр із stage і поставимо чергу
  if (stage_pending){ memcpy(tx_frame, stage_frame, FRAME_SIZE); stage_pending=false; }
  else              { stageNOP(); memcpy(tx_frame, stage_frame, FRAME_SIZE); stage_pending=false; }
  queueOnce();

  Serial.println("SLAVE: mesh+SPI готовий.");
}

// ======= loop =======
void loop(){
  // моргалка 2 Гц
  static uint32_t t=0; if (millis()-t>500){ digitalWrite(LED_PIN, !digitalRead(LED_PIN)); t=millis(); }

  mesh.update();

  // якщо з якихось причин черга зникла — підставимо NOP і знову піднімемо READY
  if (!queued){
    if (!stage_pending) stageNOP();
    memcpy(tx_frame, stage_frame, FRAME_SIZE); stage_pending=false;
    queueOnce();
  }

  if (slave.hasTransactionsCompletedAndAllResultsReady(1)){
    setReady(false);         // на час обробки
    queued = false;

    // обробляємо, що прислав MASTER
    uint8_t type,len,data[24]; uint16_t seq;
    if (parseFrame(rx_frame,type,seq,data,len)){
      if (type==T_TEXT && len){
        String cmd; cmd.reserve(len); for(uint8_t i=0;i<len;i++) cmd+=(char)data[i];
        Serial.printf("[M->] %s\n", cmd.c_str());
        // тільки розсилаємо у mesh — міст сам відповідей не генерує
        mesh.sendBroadcast(cmd);
      } else if (type==T_DOUBLE && len==8){
        double dv; memcpy(&dv,data,8);
        // за потреби: mesh.sendBroadcast(String("double:")+String(dv,6));
      }
    }

    // ПІСЛЯ завершення транзакції: готуємо НАСТУПНИЙ кадр
    if (!stage_pending) stageNOP();                 // якщо нічого не накопичили — NOP
    memcpy(tx_frame, stage_frame, FRAME_SIZE);      // копіюємо stage -> tx для наступної DMA
    stage_pending = false;

    // і негайно ставимо нову чергу
    queueOnce();
  }
}
