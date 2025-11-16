#include <ESP32SPISlave.h>
#include <WiFi.h>
#include <painlessMesh.h>
#include "mash_parameter.h"   // MESH_PREFIX, MESH_PASSWORD, MESH_PORT
#include "CRC.h"             

// ===== Pins (VSPI) =====
static const int PIN_SCK   = 18;
static const int PIN_MISO  = 19;
static const int PIN_MOSI  = 23;
static const int PIN_CS    = 5;
static const int PIN_READY = 4;   // -> MASTER 13

ESP32SPISlave slave;
Scheduler     userScheduler;
painlessMesh  mesh;

// ===== 40B frame =====
static const uint8_t FRAME_SIZE = 40;
static const uint8_t MAGIC      = 0xA5;
enum : uint8_t { T_NOP=0, T_TEXT=1, T_DOUBLE=2 };

// Розклад 40B:
// 0:MAGIC 1:TYPE 2..3:SEQ 4:LEN(0..32) 5..36:DATA(<=32)
// 37:CHK (XOR 0..36)  38..39:ACKSEQ(lo,hi)

uint8_t  tx_frame[FRAME_SIZE];   // DMA in-flight
uint8_t  rx_frame[FRAME_SIZE];

// ---- Lock-free ring queue (16 frames) для SPI ----
constexpr uint8_t QSIZE = 16;
uint8_t spiQ[QSIZE][FRAME_SIZE];
volatile uint8_t spiQHead = 0, spiQTail = 0;
portMUX_TYPE spiQMux = portMUX_INITIALIZER_UNLOCKED;

inline uint8_t spiQInc(uint8_t i){
    i++;
    if (i >= QSIZE) i = 0;
    return i;
}
inline bool spiQEmpty(){
    return spiQHead == spiQTail;
}

void qPushFrame(const uint8_t* f){
  portENTER_CRITICAL(&spiQMux);
  uint8_t next = spiQInc(spiQHead);
  if (next == spiQTail) spiQTail = spiQInc(spiQTail);   // overwrite oldest if full
  memcpy(spiQ[spiQHead], f, FRAME_SIZE);
  spiQHead = next;
  portEXIT_CRITICAL(&spiQMux);
}

bool qPopFrame(uint8_t* out){
  portENTER_CRITICAL(&spiQMux);
  if (spiQEmpty()){
    portEXIT_CRITICAL(&spiQMux);
    return false;
  }
  memcpy(out, spiQ[spiQTail], FRAME_SIZE);
  spiQTail = spiQInc(spiQTail);
  portEXIT_CRITICAL(&spiQMux);
  return true;
}

// ===== ACK/SEQ state (stop-and-wait по SPI) =====
uint16_t tx_seq = 0;                 // SLAVE->MASTER
uint16_t inflight_seq = 0;
bool     awaiting_ack = false;
uint8_t  inflight_frame[FRAME_SIZE];

uint16_t rx_last_ok = 0;             // останній коректно прийнятий SEQ від MASTER (ACK)
uint16_t rx_last_delivered = 0;      // антидубль

bool     queued = false;

inline void setReady(bool r){ digitalWrite(PIN_READY, r?HIGH:LOW); }

// ===== протокол (SPI 40B) =====
static inline uint8_t checksum37(const uint8_t* f){
  uint8_t c=0;
  for(int i=0;i<=36;i++) c^=f[i];
  return c;
}

static inline void buildFrameTo(
    uint8_t* buf,
    uint8_t type,
    uint16_t seq,
    const uint8_t* data,
    uint8_t len,
    uint16_t ackseq
){
  if (len>32) len=32;
  memset(buf,0,FRAME_SIZE);
  buf[0]=MAGIC;
  buf[1]=type;
  buf[2]=(uint8_t)(seq & 0xFF);
  buf[3]=(uint8_t)(seq >> 8);
  buf[4]=len;
  if (len) memcpy(&buf[5], data, len);
  buf[37]=checksum37(buf);          // XOR 0..36
  buf[38]=(uint8_t)(ackseq & 0xFF);
  buf[39]=(uint8_t)(ackseq >> 8);
}

bool parseFrame(
    const uint8_t* f,
    uint8_t& type,
    uint16_t& seq,
    uint8_t* data,
    uint8_t& len,
    uint16_t& ackseq
){
  if (f[0]!=MAGIC) return false;
  if (checksum37(f)!=f[37]) return false;
  type  = f[1];
  seq   = (uint16_t)f[2] | ((uint16_t)f[3]<<8);
  len   = f[4];
  if (len>32) len=32;
  if (data && len) memcpy(data,&f[5],len);
  ackseq = (uint16_t)f[38] | ((uint16_t)f[39]<<8);
  return true;
}

// ==== enqueue helpers для SPI (strip CR/LF в кінці) ====
void enqueueText(const String& sIn){
  String s=sIn;
  while (s.length() && (s[s.length()-1]=='\n' || s[s.length()-1]=='\r'))
    s.remove(s.length()-1);
  if (!s.length()) return;

  uint8_t b[32];
  uint8_t n=(uint8_t)min(32,(int)s.length());
  memcpy(b, s.c_str(), n);
  uint8_t fr[FRAME_SIZE];
  buildFrameTo(fr, T_TEXT, ++tx_seq, b, n, rx_last_ok);
  qPushFrame(fr);
}

void enqueueDouble(double v){
  uint8_t b[8]; memcpy(b,&v,8);
  uint8_t fr[FRAME_SIZE];
  buildFrameTo(fr, T_DOUBLE, ++tx_seq, b, 8, rx_last_ok);
  qPushFrame(fr);
}

void enqueueNOP(){
  uint8_t fr[FRAME_SIZE];
  buildFrameTo(fr, T_NOP, ++tx_seq, nullptr, 0, rx_last_ok);
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

// ---- Локальна обробка службових команд моста (з боку MASTER) ----
bool selfHandle(String cmd){
  cmd.trim();
  if (!cmd.length()) return true;

  // локальна службова команда: ky -> kyy назад у MASTER
  if (cmd == "ky") {
    enqueueText("kyy");
    return true;
  }

  return false;   // не обробили локально — піде в mesh з доданим CRC
}

// --- ACK-aware підготовка TX кадра ---
void prepareTxFrameWithAck(){
  const uint16_t ack_to_send = rx_last_ok;

  if (awaiting_ack){
    // повторно шлемо inflight_frame з оновленим ACK
    memcpy(tx_frame, inflight_frame, FRAME_SIZE);
    tx_frame[38]=(uint8_t)(ack_to_send & 0xFF);
    tx_frame[39]=(uint8_t)(ack_to_send >> 8);
  } else {
    // беремо з SPI-черги або NOP
    if (!qPopFrame(tx_frame)) {
      buildFrameTo(tx_frame, T_NOP, ++tx_seq, nullptr, 0, ack_to_send);
    } else {
      tx_frame[38]=(uint8_t)(ack_to_send & 0xFF);
      tx_frame[39]=(uint8_t)(ack_to_send >> 8);
    }
    // якщо кадр не NOP — виставляємо як "inflight"
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

// ====== VARIANT A: handleBody + qPop (Mesh -> SPI) ======
void handleBody(const String &body){
  // body — вже БЕЗ "*CRC", CRC перевірився в CRC.h
  String cmd = body;
  cmd.trim();
  if (!cmd.length()) return;

  // усе, що прийшло з Mesh, відправляємо в бік MASTER (Windows) через SPI
  enqueueText(cmd);
}

// ===== setup =====
void setup(){
  Serial.begin(115200);
  Serial.printf("ROLE: SLAVE,  MAC:%llX\n", ESP.getEfuseMac());

  pinMode(PIN_READY, OUTPUT);
  setReady(false);

  WiFi.setSleep(false);                 // менша латентність Wi-Fi

  // Mesh + CRC
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);    // із CRC.h (перевіряє CRC, кладе в qPop-чергу)

  // SPI SLAVE
  slave.setQueueSize(1);
  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Перше привітання в бік MASTER
  enqueueText("Hello from SLAVE!");

  // Згенерувати перший TX кадр і поставити в SPI
  prepareTxFrameWithAck();
  queueOnce();

  Serial.println("SLAVE: mesh+SPI (ACK, 40B, CRC-only mesh) готовий.");
}

// ===== loop =====
void loop(){
  // 1) Оновити mesh
  mesh.update();

  // 2) Забрати всі готові повідомлення з CRC-черги (VARIANT A: qPop(String))
  for (uint8_t i = 0; i < 4; ++i){
    String body;
    if (!qPop(body)) break;    // qPop() визначений у CRC.h
    handleBody(body);
  }

  // 3) Якщо з якоїсь причини черга по SPI пуста — підставляємо наступний кадр (NOP або дані)
  if (!queued){
    prepareTxFrameWithAck();
    queueOnce();
  }

  // 4) Обробка завершених SPI-транзакцій
  if (slave.hasTransactionsCompletedAndAllResultsReady(1)){
    setReady(false);
    queued = false;

    // прийняте від MASTER
    uint8_t type,len,data[32];
    uint16_t rseq, rack;
    if (parseFrame(rx_frame,type,rseq,data,len,rack)){
      // ACK від MASTER
      if (awaiting_ack && rack == inflight_seq){
        awaiting_ack = false;
      }

      bool is_dup = (rseq == rx_last_delivered);
      if (!is_dup){
        rx_last_delivered = rseq;
        rx_last_ok        = rseq;

        if (type==T_TEXT && len){
          String cmd; cmd.reserve(len);
          for(uint8_t i=0;i<len;i++) cmd += (char)data[i];

          // 1) Локальні службові команди (ky → kyy)
          if (!selfHandle(cmd)) {
            // 2) В Mesh — додаємо CRC в CRC.h (sendB)
            sendB(cmd);
          }
        } else if (type==T_DOUBLE && len==8){
          double dv; memcpy(&dv,data,8);
          // за потреби можна слати double в Mesh як текст:
          // sendB(String("double:")+String(dv,6));
        }
      }
    }

    // 5) Готуємо наступний TX кадр (з ACK) і ставимо в чергу
    prepareTxFrameWithAck();
    queueOnce();
  }
}
