#include <ESP32SPISlave.h>
#include <painlessMesh.h>
#include <WiFi.h>
#include "mash_parameter.h"   // MESH_PREFIX, MESH_PASSWORD, MESH_PORT

// ===== Pins (VSPI) =====
static const int PIN_SCK   = 18;
static const int PIN_MISO  = 19;
static const int PIN_MOSI  = 23;
static const int PIN_CS    = 5;
static const int PIN_READY = 4;   // -> MASTER 13
static const int LED_PIN   = 2;

ESP32SPISlave slave;
Scheduler     userScheduler;
painlessMesh  mesh;

// ===== 32B frame =====
static const uint8_t FRAME_SIZE = 32;
static const uint8_t MAGIC      = 0xA5;
enum : uint8_t { T_NOP=0, T_TEXT=1, T_DOUBLE=2 };

uint8_t  tx_frame[FRAME_SIZE];   // DMA in-flight
uint8_t  rx_frame[FRAME_SIZE];

// ---- Lock-free ring queue (16 frames) ----
constexpr uint8_t QSIZE = 16;
uint8_t q[QSIZE][FRAME_SIZE];
volatile uint8_t qHead = 0, qTail = 0;
portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;

inline uint8_t qInc(uint8_t i){ i++; if(i>=QSIZE) i=0; return i; }
inline bool qEmpty(){ return qHead==qTail; }
void qPushFrame(const uint8_t* f){
  portENTER_CRITICAL(&qMux);
  uint8_t next = qInc(qHead);
  if (next == qTail) qTail = qInc(qTail);   // overwrite oldest if full
  memcpy(q[qHead], f, FRAME_SIZE);
  qHead = next;
  portEXIT_CRITICAL(&qMux);
}
bool qPopFrame(uint8_t* out){
  portENTER_CRITICAL(&qMux);
  if (qEmpty()){ portEXIT_CRITICAL(&qMux); return false; }
  memcpy(out, q[qTail], FRAME_SIZE);
  qTail = qInc(qTail);
  portEXIT_CRITICAL(&qMux);
  return true;
}

// ===== ACK/SEQ state (stop-and-wait) =====
uint16_t tx_seq = 0;                 // наш лічильник відправок (SLAVE->MASTER)
uint16_t inflight_seq = 0;
bool     awaiting_ack = false;
uint8_t  inflight_frame[FRAME_SIZE];

uint16_t rx_last_ok = 0;             // останній коректно прийнятий SEQ від MASTER (ACK)
uint16_t rx_last_delivered = 0;      // антидубль: останній доставлений у mesh/локально

bool     queued = false;
inline void setReady(bool r){ digitalWrite(PIN_READY, r?HIGH:LOW); }

// ===== протокол helpers =====
static inline uint8_t checksum(const uint8_t* f){ uint8_t c=0; for(int i=0;i<=28;i++) c^=f[i]; return c; }
static inline void buildFrameTo(uint8_t* buf, uint8_t type, uint16_t seq, const uint8_t* data, uint8_t len, uint16_t ackseq){
  if (len>24) len=24;
  memset(buf,0,FRAME_SIZE);
  buf[0]=MAGIC; buf[1]=type;
  buf[2]=(uint8_t)(seq & 0xFF);
  buf[3]=(uint8_t)(seq >> 8);
  buf[4]=len;
  if (len) memcpy(&buf[5], data, len);
  buf[29]=checksum(buf);          // XOR 0..28
  buf[30]=(uint8_t)(ackseq & 0xFF);
  buf[31]=(uint8_t)(ackseq >> 8);
}
bool parseFrame(const uint8_t* f, uint8_t& type, uint16_t& seq, uint8_t* data, uint8_t& len, uint16_t& ackseq){
  if (f[0]!=MAGIC) return false;
  if (checksum(f)!=f[29]) return false;
  type=f[1];
  seq  = (uint16_t)f[2] | ((uint16_t)f[3]<<8);
  len  = f[4]; if (len>24) len=24;
  if (data && len) memcpy(data,&f[5],len);
  ackseq = (uint16_t)f[30] | ((uint16_t)f[31]<<8);
  return true;
}

// enqueue helpers (strip CR/LF at end)
void enqueueText(const String& sIn){
  String s=sIn;
  while (s.length() && (s[s.length()-1]=='\n' || s[s.length()-1]=='\r')) s.remove(s.length()-1);
  if (!s.length()) return;
  uint8_t b[24]; uint8_t n=(uint8_t)min(24,(int)s.length());
  memcpy(b, s.c_str(), n);
  uint8_t fr[FRAME_SIZE]; buildFrameTo(fr, T_TEXT, ++tx_seq, b, n, rx_last_ok);
  qPushFrame(fr);
}
void enqueueDouble(double v){
  uint8_t b[8]; memcpy(b,&v,8);
  uint8_t fr[FRAME_SIZE]; buildFrameTo(fr, T_DOUBLE, ++tx_seq, b, 8, rx_last_ok);
  qPushFrame(fr);
}
void enqueueNOP(){
  uint8_t fr[FRAME_SIZE]; buildFrameTo(fr, T_NOP, ++tx_seq, nullptr, 0, rx_last_ok);
  qPushFrame(fr);
}

// ---- SPI queue/trigger ----
void queueOnce(){
  if (!queued){
    slave.queue(tx_frame, rx_frame, FRAME_SIZE);
    slave.trigger();
    setReady(true);       // рівневий READY
    queued = true;
  }
}

// ===== Mesh callbacks =====
void receivedCallback(uint32_t from, String &msg){
  Serial.printf("[MESH <- %u] %s\n", from, msg.c_str());
  enqueueText(msg);            // у чергу — без торкання DMA
}
void newConnectionCallback(uint32_t){}

// ---- Локальна обробка службових команд моста ----
bool selfHandle(String cmd){
  cmd.trim();
  if (!cmd.length()) return true; // порожні не пускаємо в mesh
  if (cmd == "ky") { enqueueText("kyy"); return true; }
  return false;                   // не обробили тут — підемо в mesh
}

// --- ACK-aware підготовка TX кадра ---
void prepareTxFrameWithAck(){
  const uint16_t ack_to_send = rx_last_ok;

  if (awaiting_ack){
    // повтор: inflight + оновити поля ACK
    memcpy(tx_frame, inflight_frame, FRAME_SIZE);
    tx_frame[30]=(uint8_t)(ack_to_send & 0xFF);
    tx_frame[31]=(uint8_t)(ack_to_send >> 8);
  } else {
    // новий кадр з черги або NOP
    if (!qPopFrame(tx_frame)) {
      // якщо черга порожня — згенеруємо NOP з поточним ACK
      buildFrameTo(tx_frame, T_NOP, ++tx_seq, nullptr, 0, ack_to_send);
    } else {
      // вийняли кадр, але підставимо актуальний ACK
      tx_frame[30]=(uint8_t)(ack_to_send & 0xFF);
      tx_frame[31]=(uint8_t)(ack_to_send >> 8);
    }
    // зафіксувати "в польоті" якщо не NOP
    bool nonEmpty = (tx_frame[1]!=T_NOP || tx_frame[4]!=0);
    if (nonEmpty){
      memcpy(inflight_frame, tx_frame, FRAME_SIZE);
      inflight_seq = (uint16_t)tx_frame[2] | ((uint16_t)tx_frame[3]<<8);
      awaiting_ack = true;
    } else {
      awaiting_ack = false;
    }
  }
}

// ===== setup =====
void setup(){
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.printf("ROLE: SLAVE,  MAC:%llX\n", ESP.getEfuseMac());

  pinMode(PIN_READY, OUTPUT); setReady(false);

  WiFi.setSleep(false);                 // менша латентність Wi-Fi
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  slave.setQueueSize(1);
  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // перший кадр (привіт)
  enqueueText("Hello from SLAVE!");
  // підготуємо перший TX кадр (із ACK=0) і поставимо чергу
  prepareTxFrameWithAck();
  queueOnce();

  Serial.println("SLAVE: mesh+SPI (ACK) готовий.");
}

// ===== loop =====
void loop(){
  static uint32_t t=0; if (millis()-t>500){ digitalWrite(LED_PIN, !digitalRead(LED_PIN)); t=millis(); }

  mesh.update();

  if (!queued){
    prepareTxFrameWithAck();
    queueOnce();
  }

  if (slave.hasTransactionsCompletedAndAllResultsReady(1)){
    setReady(false);
    queued = false;

    // прийняте від MASTER
    uint8_t type,len,data[24]; uint16_t rseq, rack;
    if (parseFrame(rx_frame,type,rseq,data,len,rack)){
      // підтвердження нашого кадра?
      if (awaiting_ack && rack == inflight_seq){
        awaiting_ack = false;
      }

      // антидубль
      bool is_dup = (rseq == rx_last_delivered);
      if (!is_dup){
        rx_last_delivered = rseq;
        rx_last_ok        = rseq;   // піднімемо ACK у наших наступних кадрах

        if (type==T_TEXT && len){
          String cmd; cmd.reserve(len); for(uint8_t i=0;i<len;i++) cmd+=(char)data[i];
          Serial.printf("[M->] %s\n", cmd.c_str());

          // 1) спочатку пробуємо локально (ky -> kyy)
          if (!selfHandle(cmd)) {
            // 2) якщо не обробили локально — широкомовно у mesh
            mesh.sendBroadcast(cmd);
          }

        } else if (type==T_DOUBLE && len==8){
          double dv; memcpy(&dv,data,8);
          // за потреби: mesh.sendBroadcast(String("double:")+String(dv,6));
        }
      }
    }

    // підготуємо наступний TX кадр (з урахуванням ACK/ретрансу) і знову ставимо чергу
    prepareTxFrameWithAck();
    queueOnce();
  }
}
