#include <SPI.h>
#include <BluetoothSerial.h>

// ===== Pins (VSPI) =====
static const int PIN_SCK   = 18;
static const int PIN_MISO  = 19;
static const int PIN_MOSI  = 23;
static const int PIN_CS    = 5;
static const int PIN_READY = 13;        // <- SLAVE GPIO4
static const int LED_PIN   = 2;

static const uint32_t SPI_HZ = 4000000; // 4 MHz (можна 8 MHz при коротких дротах)

BluetoothSerial SerialBT;

// ===== 32B frame =====
static const uint8_t FRAME_SIZE = 32;
static const uint8_t MAGIC      = 0xA5;
enum : uint8_t { T_NOP=0, T_TEXT=1, T_DOUBLE=2 };

// Кадрові буфери
uint8_t  tx_frame[FRAME_SIZE];
uint8_t  rx_frame[FRAME_SIZE];

// --- BT/USB input (newline-framed, no timeouts) ---
static char    bt_buf[24];  static uint8_t bt_len  = 0;
static char    usb_buf[24]; static uint8_t usb_len = 0;

char   pending_text[24] = {0};
bool   have_pending = false;
bool   doubleMode   = false;
double pending_double = 0.0;

volatile bool bt_connected = false;
bool hello_sent = false;

// READY interrupt (optional)
volatile bool readyFlag = false;
void IRAM_ATTR readyISR() { readyFlag = true; }

// ===== ACK/SEQ state (stop-and-wait) =====
uint16_t tx_seq = 0;                 // наш лічильник відправок
uint16_t inflight_seq = 0;           // SEQ кадра "в польоті"
bool     awaiting_ack = false;       // чекаємо підтвердження нашого кадра
uint8_t  inflight_frame[FRAME_SIZE]; // копія відправленого кадра для повторів

uint16_t rx_last_ok = 0;             // останній коректно прийнятий SEQ від SLAVE (шлемо в ACK)
uint16_t rx_last_delivered = 0;      // останній вже доставлений у BT SEQ від SLAVE (для антидублів)

// ===== utils =====
static inline uint8_t checksum(const uint8_t* f){ uint8_t c=0; for(int i=0;i<=28;i++) c^=f[i]; return c; }
static inline uint8_t safeLen24(const char* s){ uint8_t n=0; while (n<24 && s[n]) n++; return n; }

// build із ACKSEQ у [30..31] (чексум лишається як був: XOR(0..28))
void buildFrame(uint8_t type, uint16_t seq, const uint8_t* data, uint8_t len, uint16_t ackseq){
  if (len>24) len=24;
  memset(tx_frame, 0, FRAME_SIZE);
  tx_frame[0]=MAGIC; tx_frame[1]=type;
  tx_frame[2]=(uint8_t)(seq & 0xFF);
  tx_frame[3]=(uint8_t)(seq >> 8);
  tx_frame[4]=len;
  if (len) memcpy(&tx_frame[5], data, len);
  tx_frame[29]=checksum(tx_frame);        // XOR 0..28
  tx_frame[30]=(uint8_t)(ackseq & 0xFF);  // ACKSEQ low/high
  tx_frame[31]=(uint8_t)(ackseq >> 8);
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
void makeNOP(uint16_t ackseq)               { buildFrame(T_NOP,    tx_seq, nullptr, 0, ackseq); }
void makeText(const char* s, uint16_t ackseq){ buildFrame(T_TEXT,   ++tx_seq, (const uint8_t*)s, safeLen24(s), ackseq); }
void makeDouble(double v, uint16_t ackseq)   { uint8_t b[8]; memcpy(b,&v,8); buildFrame(T_DOUBLE, ++tx_seq, b, 8, ackseq); }

// BT callback
void bt_cb(esp_spp_cb_event_t e, esp_spp_cb_param_t*){
  if (e==ESP_SPP_SRV_OPEN_EVT){ bt_connected = true;  hello_sent = false; }
  if (e==ESP_SPP_CLOSE_EVT)   { bt_connected = false; hello_sent = false; }
}

// --- byte pumps (формуємо пакет лише на '\n') ---
void btPump() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (bt_len==0) break;                 // ігноримо порожні
      bt_buf[bt_len]=0;
      if (bt_len>=2 && bt_buf[0]=='d' && bt_buf[1]==' ') { pending_double=atof(bt_buf+2); doubleMode=true; }
      else { memcpy(pending_text, bt_buf, min((int)sizeof(pending_text),(int)bt_len+1)); doubleMode=false; }
      bt_len=0; have_pending=true; break;
    }
    if (bt_len < sizeof(bt_buf)-1) bt_buf[bt_len++]=c;
    else { bt_buf[bt_len]=0; memcpy(pending_text, bt_buf, sizeof(pending_text)); bt_len=0; doubleMode=false; have_pending=true; break; }
  }
}
void usbPump() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (usb_len==0) break;
      usb_buf[usb_len]=0;
      if (usb_len>=2 && usb_buf[0]=='d' && usb_buf[1]==' ') { pending_double=atof(usb_buf+2); doubleMode=true; }
      else { memcpy(pending_text, usb_buf, min((int)sizeof(pending_text),(int)usb_len+1)); doubleMode=false; }
      usb_len=0; have_pending=true; break;
    }
    if (usb_len < sizeof(usb_buf)-1) usb_buf[usb_len++]=c;
    else { usb_buf[usb_len]=0; memcpy(pending_text, usb_buf, sizeof(pending_text)); usb_len=0; doubleMode=false; have_pending=true; break; }
  }
}

// --- SPI + ACK ---
void prepareTxFrameWithAck(){
  const uint16_t ack_to_send = rx_last_ok;

  if (awaiting_ack){
    // Повтор: inflight_frame + оновлений ACK у [30..31]
    memcpy(tx_frame, inflight_frame, FRAME_SIZE);
    tx_frame[30]=(uint8_t)(ack_to_send & 0xFF);
    tx_frame[31]=(uint8_t)(ack_to_send >> 8);
  } else {
    // Новий кадр або NOP
    if (have_pending){
      if (doubleMode) makeDouble(pending_double, ack_to_send);
      else            makeText(pending_text,    ack_to_send);
      // зафіксуємо "в польоті" лише якщо не NOP
      memcpy(inflight_frame, tx_frame, FRAME_SIZE);
      inflight_seq  = (uint16_t)tx_frame[2] | ((uint16_t)tx_frame[3]<<8);
      awaiting_ack  = (tx_frame[1]!=T_NOP || tx_frame[4]!=0);
      have_pending  = false;
    } else {
      makeNOP(ack_to_send);
      awaiting_ack = false; // за NOP не чекаємо ACK
    }
  }
}

void spiExchange32(){
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  for (uint8_t i=0;i<FRAME_SIZE;i++) rx_frame[i]=SPI.transfer(tx_frame[i]);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void handleRx(){
  uint8_t type,len,data[24]; uint16_t rseq, rack;
  if (!parseFrame(rx_frame,type,rseq,data,len,rack)) return;

  // Підтверджено наш кадр?
  if (awaiting_ack && rack == inflight_seq){
    awaiting_ack = false;
  }

  // Антидубль вхідних
  bool is_dup = (rseq == rx_last_delivered);
  if (!is_dup){
    rx_last_delivered = rseq;
    rx_last_ok        = rseq;  // піднімемо ACK

    if (type==T_TEXT && len){
      String s; s.reserve(len);
      for(uint8_t i=0;i<len;i++) s+=(char)data[i];
      while (s.length() && (s[s.length()-1]=='\n' || s[s.length()-1]=='\r')) s.remove(s.length()-1);
      if (s.length()){
        SerialBT.write((const uint8_t*)s.c_str(), s.length());
        SerialBT.write('\n');
        SerialBT.flush();
        Serial.printf("[S->] TEXT(%u): %s\n", rseq, s.c_str());
      }
    } else if (type==T_DOUBLE && len==8){
      double dv; memcpy(&dv,data,8);
      Serial.printf("[S->] DOUBLE(%u): %.10f\n", rseq, dv);
    }
  }
}

void setup(){
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.printf("ROLE: MASTER, MAC:%llX\n", ESP.getEfuseMac());

  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  pinMode(PIN_READY, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_READY), readyISR, RISING);

  SerialBT.begin("Kennet'MASH'");
  SerialBT.register_callback(bt_cb);

  Serial.println("MASTER: BT+SPI готовий.");
}

void loop(){
  if (SerialBT.hasClient() && bt_connected && !hello_sent){
    SerialBT.print("hello\n");
    hello_sent = true;
  }

  static uint32_t t=0; if (millis()-t>1000){ digitalWrite(LED_PIN, !digitalRead(LED_PIN)); t=millis(); }

  btPump();
  usbPump();

  if (have_pending || readyFlag || digitalRead(PIN_READY)==HIGH) {
    do {
      readyFlag=false;
      prepareTxFrameWithAck();
      spiExchange32();
      handleRx();
      delayMicroseconds(200);  // дати SLAVE поставити наступну чергу
    } while (digitalRead(PIN_READY)==HIGH);
    memset(tx_frame,0,FRAME_SIZE);
  }
}
